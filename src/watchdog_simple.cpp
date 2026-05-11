/**
 * @file watchdog_simple.cpp
 * @brief nRF52 watchdog helper implementation.
 */
#include "watchdog_simple.h"
#include <nrf.h>

/**
 * @brief Returns the hardware watchdog run status.
 *
 * @return true if the NRF_WDT peripheral is running, false otherwise.
 */
bool watchdogIsRunning()
{
    return ((NRF_WDT->RUNSTATUS & WDT_RUNSTATUS_RUNSTATUS_Msk) != 0);
}

/**
 * @brief Initializes and starts the nRF watchdog peripheral.
 *
 * @warning The hardware watchdog cannot be stopped after it is started.
 */
void watchdogBegin(uint32_t timeoutMs)
{
    if (watchdogIsRunning())
    {
        return;
    }

    uint32_t ticks = (timeoutMs * 32768UL) / 1000UL;

    if (ticks < 15UL)
    {
        ticks = 15UL;
    }

    if (ticks > 0xFFFFFFUL)
    {
        ticks = 0xFFFFFFUL;
    }

    NRF_WDT->CONFIG =
        (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |
        (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);

    NRF_WDT->CRV = ticks;
    NRF_WDT->RREN = WDT_RREN_RR0_Msk;
    NRF_WDT->TASKS_START = 1;
}

/**
 * @brief Reloads watchdog channel RR0 when the watchdog is active.
 */
void watchdogFeed()
{
    if (!watchdogIsRunning())
    {
        return;
    }

    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}
