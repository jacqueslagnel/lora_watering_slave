/**
 * @file watering.h
 * @brief Watering actuator and flow-sensor controller interface.
 *
 * This file declares the Watering class, which controls the valve driver,
 * boost supply, flow pulse counting, timed watering, post-close monitoring, and
 * leak indication used by the LoRa slave node.
 */
#pragma once

#include <Arduino.h>

/**
 * @class Watering
 * @brief Controls the watering valve and flow metering state.
 *
 * Watering owns the valve state, timed watering state, flow pulse counters, and
 * post-close leak monitoring. It also provides static wrappers for GPIO control
 * and the flow interrupt callback.
 */
class Watering
{
public:
    /**
     * @brief Constructs a Watering controller with default inactive state.
     */
    Watering();

    /**
     * @brief Initializes the watering GPIOs and flow interrupt state.
     */
    void begin();
    /**
     * @brief Runs one watering state update iteration.
     */
    void loop();

    /**
     * @brief Opens the watering valve.
     *
     * @return true if the open sequence is accepted, false otherwise.
     */
    bool openValve();
    /**
     * @brief Closes the watering valve.
     *
     * @return true if the close sequence is accepted, false otherwise.
     */
    bool closeValve();
    /**
     * @brief Starts timed watering.
     *
     * @param[in] durationSec Requested watering duration in seconds.
     * @return true if timed watering starts, false otherwise.
     */
    bool startWatering(uint16_t durationSec);
    /**
     * @brief Aborts the current timed watering operation.
     *
     * @return true if abort handling is accepted, false otherwise.
     */
    bool abortWatering();

    /**
     * @brief Returns the cached valve-open state.
     *
     * @return true if the valve is considered open, false otherwise.
     */
    bool isValveOpen() const;
    /**
     * @brief Returns whether timed watering is active.
     *
     * @return true if watering is active, false otherwise.
     */
    bool isWateringActive() const;

    /**
     * @brief Returns the programmed watering duration.
     *
     * @return Duration in seconds.
     */
    uint16_t getProgrammedDurationSec() const;
    /**
     * @brief Returns the remaining watering duration.
     *
     * @return Remaining duration in seconds.
     */
    uint16_t getRemainingDurationSec() const;

    /**
     * @brief Returns the full flow pulse counter.
     *
     * @return Flow pulse count.
     */
    uint32_t getFlowPulses() const;
    /**
     * @brief Returns the flow pulse counter truncated to 16 bits.
     *
     * @return Flow pulse count as uint16_t.
     */
    uint16_t getFlowPulses16() const;
    /**
     * @brief Returns the water volume value encoded on 16 bits.
     *
     * @return Water volume value as uint16_t.
     */
    uint16_t getLitres16() const;

    /**
     * @brief Returns pulses counted during post-close monitoring.
     *
     * @return Post-close pulse count.
     */
    uint32_t getPostClosePulses() const;
    /**
     * @brief Returns the post-close leak detection flag.
     *
     * @return 1 when a leak is detected, 0 otherwise.
     */
    uint8_t isPostCloseLeakDetected() const;

    /**
     * @brief Resets flow pulse counters used for watering measurement.
     */
    void resetFlowCounter();

    /**
     * @brief Static interrupt service entry point for flow pulses.
     */
    static void onFlowPulseISR();

    /**
     * @brief Configures a pin as an output.
     *
     * @param[in] pin Pin identifier.
     */
    static void pinOutBegin(uint32_t pin);

    /**
     * @brief Drives a pin high.
     *
     * @param[in] pin Pin identifier.
     */
    static void pinHigh(uint32_t pin);

    /**
     * @brief Drives a pin low.
     *
     * @param[in] pin Pin identifier.
     */
    static void pinLow(uint32_t pin);

    /**
     * @brief Detaches the flow sensor interrupt.
     */
    void detachFlowInterrupt();
    /**
     * @brief Attaches the flow sensor interrupt.
     */
    void attachFlowInterrupt();

private:
    static Watering *instance; ///< Active instance used by the static interrupt callback.

    bool valveOpen;                 ///< true when the valve is considered open.
    bool wateringActive;            ///< true while timed watering is active.
    uint16_t programmedDurationSec; ///< Programmed watering duration in seconds.
    uint32_t wateringStartMs;       ///< millis() timestamp when timed watering started.
    volatile uint32_t flowPulses;   ///< Flow pulses counted during watering.

    volatile uint32_t postCloseRawPulses;                       ///< Raw pulses counted after valve close.
    volatile uint32_t postClosePulses;                          ///< Pulses retained for post-close leak detection.
    volatile uint32_t postCloseIgnoredPulses;                   ///< Pulses ignored during the post-close grace period.
    bool postCloseMonitorActive;                                ///< true while post-close monitoring is active.
    bool postCloseIgnoreDone;                                   ///< true after the post-close ignore window has elapsed.
    bool postCloseLeakDetected;                                 ///< true when post-close pulses exceed the leak threshold.
    uint32_t postCloseMonitorStartMs;                           ///< millis() timestamp when post-close monitoring started.
    static constexpr uint32_t POST_CLOSE_IGNORE_MS = 5000UL;    ///< Initial post-close ignore window, in milliseconds.
    static constexpr uint32_t POST_CLOSE_MONITOR_MS = 30000UL;  ///< Post-close monitoring duration, in milliseconds.
    static constexpr uint32_t POST_CLOSE_LEAK_MIN_PULSES = 5UL; ///< Pulse threshold used to flag a leak.

    /**
     * @brief Enables the 9 V boost supply used for valve actuation.
     */
    void enableBoost9V();
    /**
     * @brief Disables the 9 V boost supply.
     */
    void disableBoost9V();
    /**
     * @brief Places the valve driver in its idle state.
     */
    void drvIdle();
    /**
     * @brief Sends the driver pulse used to open the valve.
     */
    void drvOpenPulse();
    /**
     * @brief Sends the driver pulse used to close the valve.
     */
    void drvClosePulse();
    /**
     * @brief Updates timed watering and post-close monitoring state.
     */
    void updateWateringState();
};
