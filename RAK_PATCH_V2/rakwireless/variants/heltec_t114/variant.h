#pragma once

#include <stdint.h>
#include "nrf_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "WVariant.h"

#ifdef __cplusplus
}
#endif

#define VARIANT_MCK        (64000000ul)

#define PINS_COUNT         (23)
#define NUM_DIGITAL_PINS   (23)
#define NUM_ANALOG_INPUTS  (1)
#define NUM_ANALOG_OUTPUTS (0)

// LEDs
#define PIN_LED1           (0)
#define LED_BUILTIN        PIN_LED1
#define LED_GREEN          PIN_LED1
#define LED_BLUE           PIN_LED1
#define LED_STATE_ON       0

// UART
#define PIN_SERIAL1_RX     (255)
#define PIN_SERIAL1_TX     (255)
#define PIN_SERIAL_RX      PIN_SERIAL1_RX
#define PIN_SERIAL_TX      PIN_SERIAL1_TX

// I2C
#define PIN_WIRE_SDA       (1)
#define PIN_WIRE_SCL       (2)

// Analog
#define PIN_A0             (3)
static const uint8_t A0 = PIN_A0;

// SPI aliases required by core/libs
#define PIN_SPI_SCK        (5)
#define PIN_SPI_MOSI       (6)
#define PIN_SPI_MISO       (7)
#define PIN_SPI_SS         (8)

static const uint8_t SS   = PIN_SPI_SS;
static const uint8_t SCK  = PIN_SPI_SCK;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;

// Board-specific named pins
static const uint8_t PIN_BAT_ADC         = PIN_A0; // P0.04
static const uint8_t PIN_BAT_ADC_EN      = 4;      // P0.06

static const uint8_t PIN_LORA_SCK        = 5;      // P0.19
static const uint8_t PIN_LORA_MOSI       = 6;      // P0.22
static const uint8_t PIN_LORA_MISO       = 7;      // P0.23
static const uint8_t PIN_LORA_NSS        = 8;      // P0.24
static const uint8_t PIN_LORA_DIO1       = 9;      // P0.20
static const uint8_t PIN_LORA_RESET      = 10;     // P0.25
static const uint8_t PIN_LORA_BUSY       = 11;     // P0.17

static const uint8_t PIN_VEXT_CTRL       = 12;     // P0.21
static const uint8_t PIN_USER_BUTTON     = 13;     // P1.11
static const uint8_t PIN_NFC1            = 14;     // P0.09
static const uint8_t PIN_NFC2            = 15;     // P0.10
static const uint8_t PIN_P1_04           = 16;     // P1.04
static const uint8_t PIN_P1_05           = 17;     // P1.05
static const uint8_t PIN_P1_06           = 18;     // P1.06

static const uint8_t PIN_TFT_EN          = 19;     // P0.03
static const uint8_t PIN_TFT_LED_EN      = 20;     // P0.15
static const uint8_t PIN_P1_01           = 21;     // P1.01
static const uint8_t PIN_P1_10           = 22;     // P1.10

// Raw nRF GPIO numbers, if needed
#define PIN_LORA_SCK_RAW      NRF_GPIO_PIN_MAP(0, 19)
#define PIN_LORA_DIO1_RAW     NRF_GPIO_PIN_MAP(0, 20)
#define PIN_VEXT_CTRL_RAW     NRF_GPIO_PIN_MAP(0, 21)
#define PIN_LORA_MOSI_RAW     NRF_GPIO_PIN_MAP(0, 22)
#define PIN_LORA_MISO_RAW     NRF_GPIO_PIN_MAP(0, 23)
#define PIN_LORA_NSS_RAW      NRF_GPIO_PIN_MAP(0, 24)
#define PIN_LORA_RESET_RAW    NRF_GPIO_PIN_MAP(0, 25)
#define PIN_LORA_BUSY_RAW     NRF_GPIO_PIN_MAP(0, 17)

#define PIN_BAT_ADC_RAW       NRF_GPIO_PIN_MAP(0, 4)
#define PIN_BAT_ADC_EN_RAW    NRF_GPIO_PIN_MAP(0, 6)
#define PIN_TFT_EN_RAW        NRF_GPIO_PIN_MAP(0, 3)
#define PIN_TFT_LED_EN_RAW    NRF_GPIO_PIN_MAP(0, 15)
