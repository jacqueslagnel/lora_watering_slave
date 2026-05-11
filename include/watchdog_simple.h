/**
 * @file watchdog_simple.h
 * @brief Minimal interface for driving the nRF52 hardware watchdog.
 */
#pragma once

#include <Arduino.h>

/**
 * @brief Initializes and starts the hardware watchdog.
 *
 * @param[in] timeoutMs Maximum time without a feed before reset, in milliseconds.
 * @post watchdogIsRunning() returns true if initialization was performed.
 * @warning Once the nRF watchdog is started, it generally cannot be stopped
 * before a hardware reset.
 */
void watchdogBegin(uint32_t timeoutMs = 20000);
/**
 * @brief Feeds the watchdog if it has been started.
 */
void watchdogFeed();
/**
 * @brief Indicates whether the watchdog was started through this interface.
 *
 * @return true if watchdogBegin() has already started the watchdog, false otherwise.
 */
bool watchdogIsRunning();
