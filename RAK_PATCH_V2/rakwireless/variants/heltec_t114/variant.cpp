#include "variant.h"
#include "nrf.h"
#include "wiring_constants.h"
#include "nrf_gpio.h"

// Arduino pin number -> nRF GPIO mapping
const uint32_t g_ADigitalPinMap[] = {
    // 0  LED_BUILTIN -> P1.03
    NRF_GPIO_PIN_MAP(1, 3),

    // 1  I2C SDA -> P0.26
    NRF_GPIO_PIN_MAP(0, 26),

    // 2  I2C SCL -> P0.27
    NRF_GPIO_PIN_MAP(0, 27),

    // 3  A0 / BAT_ADC -> P0.04
    NRF_GPIO_PIN_MAP(0, 4),

    // 4  BAT_ADC_EN -> P0.06
    NRF_GPIO_PIN_MAP(0, 6),

    // 5  LORA_SCK -> P0.19
    NRF_GPIO_PIN_MAP(0, 19),

    // 6  LORA_MOSI -> P0.22
    NRF_GPIO_PIN_MAP(0, 22),

    // 7  LORA_MISO -> P0.23
    NRF_GPIO_PIN_MAP(0, 23),

    // 8  LORA_NSS -> P0.24
    NRF_GPIO_PIN_MAP(0, 24),

    // 9  LORA_DIO1 -> P0.20
    NRF_GPIO_PIN_MAP(0, 20),

    // 10 LORA_RESET -> P0.25
    NRF_GPIO_PIN_MAP(0, 25),

    // 11 LORA_BUSY -> P0.17
    NRF_GPIO_PIN_MAP(0, 17),

    // 12 VEXT_CTRL -> P0.21
    NRF_GPIO_PIN_MAP(0, 21),

    // 13 USER_BUTTON -> P1.11
    NRF_GPIO_PIN_MAP(1, 11),

    // 14 NFC1 / P0.09
    NRF_GPIO_PIN_MAP(0, 9),

    // 15 NFC2 / P0.10
    NRF_GPIO_PIN_MAP(0, 10),

    // 16 P1.04
    NRF_GPIO_PIN_MAP(1, 4),

    // 17 P1.05
    NRF_GPIO_PIN_MAP(1, 5),

    // 18 P1.06
    NRF_GPIO_PIN_MAP(1, 6),

    // 19 TFT_EN -> P0.03
    NRF_GPIO_PIN_MAP(0, 3),

    // 20 TFT_LED_EN -> P0.15
    NRF_GPIO_PIN_MAP(0, 15),
    
    // 21 P1.01
    NRF_GPIO_PIN_MAP(1, 1),

    // 22 P1.10
    NRF_GPIO_PIN_MAP(1, 10)};

void initVariant(void)
{
    // VEXT OFF
    nrf_gpio_cfg_output(g_ADigitalPinMap[PIN_VEXT_CTRL]);
    nrf_gpio_pin_clear(g_ADigitalPinMap[PIN_VEXT_CTRL]); // LOW

    // TFT OFF
    nrf_gpio_cfg_output(g_ADigitalPinMap[PIN_TFT_EN]);
    nrf_gpio_pin_set(g_ADigitalPinMap[PIN_TFT_EN]); // HIGH

    // TFT backlight OFF
    nrf_gpio_cfg_output(g_ADigitalPinMap[PIN_TFT_LED_EN]);
    nrf_gpio_pin_set(g_ADigitalPinMap[PIN_TFT_LED_EN]); // HIGH

    // Battery divider OFF
    nrf_gpio_cfg_output(g_ADigitalPinMap[PIN_BAT_ADC_EN]);
    nrf_gpio_pin_clear(g_ADigitalPinMap[PIN_BAT_ADC_EN]); // LOW
}
