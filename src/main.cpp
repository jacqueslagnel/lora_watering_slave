/**
 * @file main.cpp
 * @brief Arduino entry point for the LoRa watering slave node.
 *
 * This file initializes early watering power pins, optional BLE OTA boot mode,
 * watchdog supervision, and the LoRaSlave protocol controller.
 */
#include <Arduino.h>
#include "ble_ota.h"
#include "lora_slave.h"
#include "watchdog_simple.h"

/**
 * @brief Debug serial stream.
 */
#define DBG Serial
/**
 * @brief Debug serial baud rate.
 */
#define DBG_BAUD 115200
/**
 * @brief Maximum wait time for USB serial connection, in milliseconds.
 */
#define DBG_WAIT_MS 4000

/**
 * @brief Watchdog timeout used by the application, in milliseconds.
 */
static constexpr uint32_t WATCHDOG_TIMEOUT_MS = 30000UL;

/**
 * @brief Global LoRa slave protocol controller.
 */
LoRaSlave loraSlave;

/**
 * @brief Arduino setup entry point.
 *
 * Performs early GPIO shutdown for watering power paths, starts the debug
 * serial interface, checks for BLE OTA boot request, starts the watchdog, and
 * initializes the LoRa slave controller.
 */
void setup()
{
  delay(10);
  nrf_gpio_cfg_output(FLOWPOWER);
  nrf_gpio_pin_clear(FLOWPOWER);
  nrf_gpio_cfg_output(WATERING_PIN_BOOST_EN);
  nrf_gpio_pin_clear(WATERING_PIN_BOOST_EN);

  DBG.begin(DBG_BAUD);

  uint32_t t0 = millis();
  while (!DBG && (millis() - t0 < DBG_WAIT_MS))
  {
    delay(10);
  }

  if (BleOta::isRequestedAtBoot())
  {
    BleOta::clearBootRequest();

    Serial.println("Booting slave in BLE OTA mode");
    char bleName[20];
    snprintf(bleName, sizeof(bleName), "NODE_%u_OTA", LoRaSlave::NODE_ID);
    BleOta::begin(bleName);

    return;
  }

  watchdogBegin(WATCHDOG_TIMEOUT_MS);
  watchdogFeed();

  loraSlave.begin();

  nrf_gpio_cfg_output(WATERING_PIN_BOOST_EN);
  nrf_gpio_pin_clear(WATERING_PIN_BOOST_EN);
}

/**
 * @brief Arduino main loop entry point.
 *
 * Services BLE OTA mode when active; otherwise delegates the cycle to LoRaSlave.
 */
void loop()
{
  if (BleOta::isActive())
  {
    BleOta::loop();
    watchdogFeed();
    __WFI();
    return;
  }
  loraSlave.loop();
}

/*  nrf_gpio_cfg_output(g_ADigitalPinMap[FLOWPOWER]);
  nrf_gpio_pin_clear(g_ADigitalPinMap[FLOWPOWER]); // LOW
  while (1)
  {
    nrf_gpio_pin_clear(g_ADigitalPinMap[FLOWPOWER]); // LOW
    DBG.println("FLOWPOWER=0");
    delay(2000);

    // nrf_gpio_pin_set(FLOWPOWER);
    nrf_gpio_pin_set(g_ADigitalPinMap[FLOWPOWER]); // LOW
    DBG.println("FLOWPOWER=1");
    delay(2000);
  }

// ou OK
 nrf_gpio_cfg_output(FLOWPOWER);
 nrf_gpio_pin_clear(FLOWPOWER); // LOW
 while (1)
 {
   nrf_gpio_pin_clear(FLOWPOWER); // LOW
   DBG.println("FLOWPOWER=0");
   delay(2000);

   // nrf_gpio_pin_set(FLOWPOWER);
   nrf_gpio_pin_set(FLOWPOWER); // LOW
   DBG.println("FLOWPOWER=1");
   delay(2000);
 }

 DBG.print("After loraSlave.begin(), P0.21 raw = ");
 DBG.println(nrf_gpio_pin_out_read(FLOWPOWER));
 */
