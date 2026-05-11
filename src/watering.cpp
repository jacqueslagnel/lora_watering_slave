/**
 * @file watering.cpp
 * @brief Implementation of valve actuation, flow counting, and leak monitoring.
 *
 * This file implements the Watering class by configuring board GPIOs, driving
 * the valve control bridge, counting flow sensor pulses, managing timed
 * watering, and monitoring post-close pulses for leak detection.
 */
// for 1 inch: Hz=[0.208* Q]±5 % (Q L/min). 288 impulsions per liter
// min  pulses l measured  l weight
// 2	2143	7	8.4
// 2	2167	7
// 2	2158	7	8.5
// 2	2177	7	8.5
// Average
// 2	2161.25		8.5
// 1	1080.625		4.25
// pulses /liter=	254.2647059
#include "watering.h"
#include "board_config.h"
#include "watchdog_simple.h"
#include <Arduino.h>
#include <nrf_gpio.h>

/**
 * @brief Debug serial stream used by this module.
 */
#define DBG Serial

/**
 * @brief Simulated boost activation duration used by the watering driver, in milliseconds.
 */
#define WATERING_BOOST_SIMULATION_MS 200

/**
 * @brief Valve driver step duration, in milliseconds.
 */
static const uint32_t VALVE_STEP_MS = 200;

// Measured calibration for 1 inch flow sensor.
// Average measured value: about 254 pulses per litre.
/**
 * @brief Flow meter calibration, in pulses per litre.
 */
static const uint32_t FLOW_PULSES_PER_LITRE = 254;

#ifndef WATERING_BOOST_STABILIZE_MS
/**
 * @brief Delay after enabling boost before valve actuation, in milliseconds.
 */
#define WATERING_BOOST_STABILIZE_MS 100
#endif

#ifndef WATERING_PULSE_MS
/**
 * @brief Main valve actuation pulse duration, in milliseconds.
 */
#define WATERING_PULSE_MS 200
#endif

#ifndef WATERING_POST_PULSE_MS
/**
 * @brief Post-actuation settling pulse duration, in milliseconds.
 */
#define WATERING_POST_PULSE_MS 50
#endif

#ifndef WATERING_FLOW_INTERRUPT_MODE
/**
 * @brief Interrupt trigger mode used for the flow sensor input.
 */
#define WATERING_FLOW_INTERRUPT_MODE FALLING
#endif

/**
 * @brief Active Watering instance used by the static flow interrupt callback.
 */
Watering *Watering::instance = nullptr;

/**
 * @brief Constructs the watering controller with inactive valve and counters reset.
 */
Watering::Watering()
    : valveOpen(false),
      wateringActive(false),
      programmedDurationSec(0),
      wateringStartMs(0),
      flowPulses(0),
      postCloseRawPulses(0),
      postClosePulses(0),
      postCloseIgnoredPulses(0),
      postCloseMonitorActive(false),
      postCloseIgnoreDone(false),
      postCloseLeakDetected(false),
      postCloseMonitorStartMs(0)
{
}

/**
 * @brief Initializes GPIOs, driver idle state, flow input, and counters.
 */
void Watering::begin()
{
    instance = this;

    pinOutBegin(WATERING_PIN_STANDBY);
    pinOutBegin(WATERING_PIN_DRV_IN1);
    pinOutBegin(WATERING_PIN_DRV_IN2);
    pinOutBegin(WATERING_PIN_BOOST_EN);
    nrf_gpio_pin_clear(WATERING_PIN_BOOST_EN);

    pinOutBegin(FLOWPOWER);
    nrf_gpio_pin_clear(FLOWPOWER);

    drvIdle();

    // Important:
    // Do not use PULLUP while Ve_3V3 is OFF.
    // Otherwise the flow sensor rail is back-powered through the GPIO.
    nrf_gpio_cfg_input(WATERING_PIN_FLOW, NRF_GPIO_PIN_NOPULL);

    valveOpen = false;
    wateringActive = false;
    programmedDurationSec = 0;
    wateringStartMs = 0;

    noInterrupts();
    flowPulses = 0;
    postCloseRawPulses = 0;
    postClosePulses = 0;
    postCloseIgnoredPulses = 0;
    interrupts();

    postCloseMonitorActive = false;
    postCloseIgnoreDone = false;
    postCloseLeakDetected = false;
    postCloseMonitorStartMs = 0;
}

/**
 * @brief Updates timed watering and post-close leak monitoring.
 */
void Watering::loop()
{
    updateWateringState();

    if (postCloseMonitorActive)
    {
        uint32_t now = millis();
        uint32_t elapsed = now - postCloseMonitorStartMs;

        // First seconds after close are ignored because water flow does not stop instantly.
        if (!postCloseIgnoreDone && elapsed >= POST_CLOSE_IGNORE_MS)
        {
            noInterrupts();
            postCloseIgnoredPulses = postCloseRawPulses;
            interrupts();

            postCloseIgnoreDone = true;

            DBG.print("POST_CLOSE ignored pulses = ");
            DBG.println(postCloseIgnoredPulses);
        }

        // End of post-close monitoring window.
        if (elapsed >= POST_CLOSE_MONITOR_MS)
        {
            uint32_t raw;

            noInterrupts();
            raw = postCloseRawPulses;
            interrupts();

            uint32_t suspiciousPulses = 0;

            if (postCloseIgnoreDone && raw >= postCloseIgnoredPulses)
            {
                suspiciousPulses = raw - postCloseIgnoredPulses;
            }

            noInterrupts();
            postClosePulses = suspiciousPulses;
            interrupts();

            if (postClosePulses >= POST_CLOSE_LEAK_MIN_PULSES)
            {
                postCloseLeakDetected = true;
            }

            DBG.print("POST_CLOSE raw pulses = ");
            DBG.println(raw);

            DBG.print("POST_CLOSE suspicious pulses = ");
            DBG.println(postClosePulses);

            DBG.print("POST_CLOSE leak = ");
            DBG.println(postCloseLeakDetected ? 1 : 0);

            postCloseMonitorActive = false;

            // Now we can really stop the flow sensor and return to low power.
            detachFlowInterrupt();
        }
    }
}

bool Watering::openValve()
{
    if (valveOpen)
    {
        return false;
    }

    // If a new watering starts during a previous post-close monitor window,
    // cancel that monitoring window.
    postCloseMonitorActive = false;
    postCloseIgnoreDone = false;

    enableBoost9V();
    delay(WATERING_BOOST_SIMULATION_MS);
    drvOpenPulse();

    // Power flow sensor, configure pullup, and attach interrupt.
    attachFlowInterrupt();

    disableBoost9V();

    valveOpen = true;
    return true;
}

bool Watering::closeValve()
{
    if (!valveOpen && !wateringActive)
    {
        return true;
    }

    enableBoost9V();
    delay(WATERING_BOOST_SIMULATION_MS);
    drvClosePulse();
    disableBoost9V();

    valveOpen = false;
    wateringActive = false;
    programmedDurationSec = 0;
    wateringStartMs = 0;

    // Start post-close monitoring.
    // Do NOT detach the flow interrupt here.
    // Flow sensor must remain powered for POST_CLOSE_MONITOR_MS.
    noInterrupts();
    postCloseRawPulses = 0;
    postClosePulses = 0;
    postCloseIgnoredPulses = 0;
    interrupts();

    postCloseIgnoreDone = false;
    postCloseLeakDetected = false;
    postCloseMonitorActive = true;
    postCloseMonitorStartMs = millis();

    DBG.println("POST_CLOSE monitor started");

    return true;
}

bool Watering::startWatering(uint16_t durationSec)
{
    if (durationSec == 0 || wateringActive || valveOpen)
    {
        return false;
    }

    resetFlowCounter();

    programmedDurationSec = durationSec;
    wateringStartMs = millis();
    wateringActive = true;

    if (!openValve())
    {
        wateringActive = false;
        programmedDurationSec = 0;
        wateringStartMs = 0;
        return false;
    }

    return true;
}

bool Watering::abortWatering()
{
    if (valveOpen || wateringActive)
    {
        closeValve();
    }
    else
    {
        valveOpen = false;
        wateringActive = false;
        programmedDurationSec = 0;
        wateringStartMs = 0;
    }

    return true;
}

bool Watering::isValveOpen() const
{
    return valveOpen;
}

bool Watering::isWateringActive() const
{
    return wateringActive;
}

uint16_t Watering::getProgrammedDurationSec() const
{
    return programmedDurationSec;
}

uint16_t Watering::getRemainingDurationSec() const
{
    if (!wateringActive)
    {
        return 0;
    }

    uint32_t elapsedSec = (millis() - wateringStartMs) / 1000UL;
    if (elapsedSec >= programmedDurationSec)
    {
        return 0;
    }

    return (uint16_t)(programmedDurationSec - elapsedSec);
}

uint32_t Watering::getFlowPulses() const
{
    noInterrupts();
    uint32_t p = flowPulses;
    interrupts();
    return p;
}

uint16_t Watering::getFlowPulses16() const
{
    uint32_t p = getFlowPulses();
    if (p > 65535UL)
    {
        p = 65535UL;
    }
    return (uint16_t)p;
}

uint16_t Watering::getLitres16() const
{
    uint32_t litres = getFlowPulses() / FLOW_PULSES_PER_LITRE;
    if (litres > 65535UL)
    {
        litres = 65535UL;
    }
    return (uint16_t)litres;
}

uint32_t Watering::getPostClosePulses() const
{
    noInterrupts();
    uint32_t p = postClosePulses;
    interrupts();
    return p;
}

uint8_t Watering::isPostCloseLeakDetected() const
{
    return postCloseLeakDetected ? 1 : 0;
}

void Watering::resetFlowCounter()
{
    noInterrupts();
    flowPulses = 0;
    interrupts();
}

void Watering::onFlowPulseISR()
{
    if (instance != nullptr)
    {
        if (instance->wateringActive)
        {
            instance->flowPulses++;
        }
        else if (instance->postCloseMonitorActive)
        {
            instance->postCloseRawPulses++;
        }
    }
}

void Watering::enableBoost9V()
{
    pinHigh(WATERING_PIN_BOOST_EN);
    delay(WATERING_BOOST_STABILIZE_MS);
}

void Watering::disableBoost9V()
{
    pinLow(WATERING_PIN_BOOST_EN);
}

void Watering::drvIdle()
{
    pinLow(WATERING_PIN_STANDBY);
    pinLow(WATERING_PIN_DRV_IN1);
    pinLow(WATERING_PIN_DRV_IN2);
}

void Watering::drvOpenPulse()
{
    pinLow(WATERING_PIN_STANDBY);
    pinHigh(WATERING_PIN_DRV_IN1);
    pinLow(WATERING_PIN_DRV_IN2);
    pinHigh(WATERING_PIN_STANDBY);

    delay(WATERING_PULSE_MS);

    drvIdle();
    delay(WATERING_POST_PULSE_MS);
}

void Watering::drvClosePulse()
{
    pinLow(WATERING_PIN_STANDBY);
    pinLow(WATERING_PIN_DRV_IN1);
    pinHigh(WATERING_PIN_DRV_IN2);
    pinHigh(WATERING_PIN_STANDBY);

    delay(WATERING_PULSE_MS);

    drvIdle();
    delay(WATERING_POST_PULSE_MS);
}

void Watering::attachFlowInterrupt()
{
    int irq = digitalPinToInterrupt(WATERING_PIN_FLOW);
    if (irq >= 0)
    {
        nrf_gpio_cfg_output(FLOWPOWER);
        nrf_gpio_pin_set(FLOWPOWER);
        delay(5);

        // Enable pullup only when Ve_3V3 is ON.
        nrf_gpio_cfg_input(WATERING_PIN_FLOW, NRF_GPIO_PIN_PULLUP);

        detachInterrupt(irq);
        attachInterrupt(irq, Watering::onFlowPulseISR, WATERING_FLOW_INTERRUPT_MODE);
    }
}

void Watering::detachFlowInterrupt()
{
    int irq = digitalPinToInterrupt(WATERING_PIN_FLOW);
    if (irq >= 0)
    {
        detachInterrupt(irq);
    }

    // Avoid back-powering the flow sensor rail.
    nrf_gpio_cfg_input(WATERING_PIN_FLOW, NRF_GPIO_PIN_NOPULL);

    nrf_gpio_cfg_output(FLOWPOWER);
    nrf_gpio_pin_clear(FLOWPOWER);
}

void Watering::updateWateringState()
{
    if (!wateringActive)
    {
        return;
    }

    uint32_t elapsedMs = millis() - wateringStartMs;
    uint32_t targetMs = (uint32_t)programmedDurationSec * 1000UL;

    if (elapsedMs >= targetMs)
    {
        closeValve();
    }
}

void Watering::pinOutBegin(uint32_t pin)
{
    nrf_gpio_cfg_output(pin);
    nrf_gpio_pin_clear(pin);
}

void Watering::pinHigh(uint32_t pin)
{
    nrf_gpio_pin_set(pin);
}

void Watering::pinLow(uint32_t pin)
{
    nrf_gpio_pin_clear(pin);
}
