/**
 * @file board_config.h
 * @brief Board-specific pin and calibration configuration.
 *
 * This header selects the LoRa, battery ADC, LED, and watering hardware
 * constants for either the Heltec T114 or the RAK4631 build. Exactly one of
 * BOARD_T114 or BOARD_RAK4631 must be provided by PlatformIO build flags.
 */
#pragma once

// ============================================================================
// board_config.h — Abstraction des constantes et broches spécifiques à chaque
// carte.  Exactement l'un de BOARD_T114 ou BOARD_RAK4631 doit être défini via
// build_flags dans platformio.ini.
// ============================================================================

#if !defined(BOARD_T114) && !defined(BOARD_RAK4631)
#error "Aucune carte sélectionnée. Définissez BOARD_T114 ou BOARD_RAK4631 dans build_flags."
#endif

// ============================================================================
// Heltec T114
// ============================================================================
#ifdef BOARD_T114

// LoRa : T114 utilise un bus SPI dédié et une init explicite via hwConfig
/**
 * @brief Enables a dedicated SPI bus for the Heltec T114 LoRa radio.
 */
#define LORA_USE_CUSTOM_SPI 1 // SPIClass SPI_LORA sur NRF_SPIM2
/**
 * @brief Selects explicit LoRa hardware initialization through hwConfig.
 */
#define LORA_USE_HW_CONFIG_INIT 1 // lora_hardware_init(hwConfig)
/**
 * @brief Requests Radio.Sleep() before channel configuration.
 */
#define LORA_SLEEP_BEFORE_CHANNEL 1 // Radio.Sleep() requis avant SetChannel
/**
 * @brief Selects interrupt-driven radio processing for this board.
 */
#define LORA_NEEDS_IRQ_PROCESS 0 // Interruptions matérielles, pas de polling

// Paramètres hwConfig SX1262 T114
/**
 * @brief Enables DIO2 RF switch control for the T114 SX1262 hardware.
 */
#define LORA_USE_DIO2_ANT_SWITCH true
/**
 * @brief Enables DIO3 TCXO control for the T114 SX1262 hardware.
 */
#define LORA_USE_DIO3_TCXO true
/**
 * @brief Disables DIO3 antenna switch control for the T114 SX1262 hardware.
 */
#define LORA_USE_DIO3_ANT_SWITCH false
/**
 * @brief Selects DC-DC mode instead of LDO mode for the T114 radio configuration.
 */
#define LORA_USE_LDO false
/**
 * @brief Disables RXEN antenna power control for the T114 radio configuration.
 */
#define LORA_USE_RXEN_ANT_PWR false
/**
 * @brief TCXO voltage setting used by the T114 radio configuration.
 */
#define LORA_TCXO_VOLTAGE TCXO_CTRL_3_3V

// Batterie ADC : T114 a une broche d'activation et un diviseur ~4.9x
/**
 * @brief Indicates that the battery ADC circuit has an enable pin.
 */
#define BAT_HAS_ENABLE_PIN 1 // PIN_BAT_ADC_EN à activer avant lecture
/**
 * @brief Number of ADC samples used for battery averaging.
 */
#define BAT_SAMPLE_COUNT 12
/**
 * @brief Number of warm-up ADC samples discarded before averaging.
 */
#define BAT_WARMUP_COUNT 4
/**
 * @brief Delay between warm-up ADC samples, in milliseconds.
 */
#define BAT_WARMUP_DELAY_MS 3
/**
 * @brief Delay between averaged ADC samples, in milliseconds.
 */
#define BAT_SAMPLE_DELAY_MS 3
/**
 * @brief Battery divider compensation factor for the T114 board.
 */
#define VBAT_DIVIDER_COMP 4.9f

// LED : T114 nécessite l'activation du rail VEXT 3.3V pour piloter la LED
// La LED est active-low (LOW = allumée, HIGH = éteinte)
/**
 * @brief Indicates that LED control requires VEXT control on this board.
 */
#define LED_HAS_VEXT_CTRL 1
/**
 * @brief GPIO level that turns the LED on.
 */
#define LED_ON_LEVEL LOW
/**
 * @brief GPIO level that turns the LED off.
 */
#define LED_OFF_LEVEL HIGH

// Broches arrosage (T114)
/**
 * @brief GPIO controlling the watering 9 V boost enable line.
 */
#define WATERING_PIN_BOOST_EN NRF_GPIO_PIN_MAP(0, 28)
/**
 * @brief GPIO connected to watering driver input 1.
 */
#define WATERING_PIN_DRV_IN1 NRF_GPIO_PIN_MAP(0, 10)
/**
 * @brief GPIO connected to watering driver input 2.
 */
#define WATERING_PIN_DRV_IN2 NRF_GPIO_PIN_MAP(0, 9)
/**
 * @brief GPIO connected to watering driver standby control.
 */
#define WATERING_PIN_STANDBY NRF_GPIO_PIN_MAP(0, 30)

// PIN_VEXT_CTRL
// #define FLOWPOWER NRF_GPIO_PIN_MAP(0, 21)
/**
 * @brief GPIO or mapped pin used to power the flow sensor circuit.
 */
#define FLOWPOWER g_ADigitalPinMap[PIN_VEXT_CTRL]

// #define WATERING_PIN_FLOW            NRF_GPIO_PIN_MAP(1, 10) // NOTOK because of the variant

// Button:
// #define WATERING_PIN_FLOW PIN_P1_10
// pin 1.01
/**
 * @brief Flow sensor input pin for the T114 board.
 */
#define WATERING_PIN_FLOW PIN_P1_01

/**
 * @brief Human-readable board name.
 */
#define BOARD_NAME_STR "Heltec T114"

#endif // BOARD_T114

// ============================================================================
// RAK4631 (WisBlock RAK4630 core)
// ============================================================================
#ifdef BOARD_RAK4631

// LoRa : RAK4631 utilise lora_rak4630_init() qui configure tout en interne
/**
 * @brief Disables custom SPI for the RAK4631 LoRa radio path.
 */
#define LORA_USE_CUSTOM_SPI 0
/**
 * @brief Disables explicit hwConfig initialization for RAK4631.
 */
#define LORA_USE_HW_CONFIG_INIT 0
/**
 * @brief Indicates no pre-channel Radio.Sleep() is required on RAK4631.
 */
#define LORA_SLEEP_BEFORE_CHANNEL 0
/**
 * @brief Selects polling of Radio.IrqProcess() for the RAK library.
 */
#define LORA_NEEDS_IRQ_PROCESS 1 // La lib RAK exige Radio.IrqProcess() en polling

// Batterie ADC : lecture directe sur PIN_VBAT (ou A0), pas de broche enable
/**
 * @brief Indicates that the battery ADC circuit does not have an enable pin.
 */
#define BAT_HAS_ENABLE_PIN 0
/**
 * @brief Number of ADC samples used for battery averaging.
 */
#define BAT_SAMPLE_COUNT 8
/**
 * @brief Number of warm-up ADC samples discarded before averaging.
 */
#define BAT_WARMUP_COUNT 0
/**
 * @brief Delay between warm-up ADC samples, in milliseconds.
 */
#define BAT_WARMUP_DELAY_MS 0
/**
 * @brief Delay between averaged ADC samples, in milliseconds.
 */
#define BAT_SAMPLE_DELAY_MS 2
/**
 * @brief Battery divider compensation factor for the RAK4631 board.
 */
#define VBAT_DIVIDER_COMP 1.725f

// LED : pilotage direct, active-high
/**
 * @brief Indicates that LED control does not require VEXT control on this board.
 */
#define LED_HAS_VEXT_CTRL 0
/**
 * @brief GPIO level that turns the LED on.
 */
#define LED_ON_LEVEL HIGH
/**
 * @brief GPIO level that turns the LED off.
 */
#define LED_OFF_LEVEL LOW

// Broches arrosage (RAK4631)
// #define WATERING_PIN_BOOST_EN_VAL NRF_GPIO_PIN_MAP(1, 4)
// #define WATERING_PIN_DRV_IN1_VAL NRF_GPIO_PIN_MAP(1, 3)
// #define WATERING_PIN_DRV_IN2_VAL NRF_GPIO_PIN_MAP(0, 17)
// #define WATERING_PIN_FLOW_VAL NRF_GPIO_PIN_MAP(0, 21)

/**
 * @brief GPIO controlling the watering 9 V boost enable line.
 */
#define WATERING_PIN_BOOST_EN NRF_GPIO_PIN_MAP(0, 4)
/**
 * @brief GPIO connected to watering driver input 1.
 */
#define WATERING_PIN_DRV_IN1 NRF_GPIO_PIN_MAP(0, 3)
/**
 * @brief GPIO connected to watering driver input 2.
 */
#define WATERING_PIN_DRV_IN2 NRF_GPIO_PIN_MAP(0, 17)
/**
 * @brief GPIO connected to watering driver standby control.
 */
#define WATERING_PIN_STANDBY NRF_GPIO_PIN_MAP(0, 21)

/**
 * @brief GPIO or mapped pin used to power the flow sensor circuit.
 */
#define FLOWPOWER WB_IO2
// P1.01 = SW1
/**
 * @brief Flow sensor input pin for the RAK4631 board.
 */
#define WATERING_PIN_FLOW NRF_GPIO_PIN_MAP(1, 1)

/**
 * @brief Human-readable board name.
 */
#define BOARD_NAME_STR "RAK4631"

#endif // BOARD_RAK4631

// ============================================================================
// Constantes ADC partagées (dépendent de VBAT_DIVIDER_COMP défini ci-dessus)
// ============================================================================
/**
 * @brief ADC reference voltage in millivolts.
 */
static constexpr float _ADC_REF_MV = 3000.0f;
/**
 * @brief ADC full-scale count value used for conversion.
 */
static constexpr float _ADC_MAX_COUNTS = 4096.0f;
/**
 * @brief Millivolts per raw ADC count before divider compensation.
 */
static constexpr float _ADC_MV_PER_LSB = (_ADC_REF_MV / _ADC_MAX_COUNTS);
/**
 * @brief Effective battery millivolts per ADC count after divider compensation.
 */
static constexpr float REAL_VBAT_MV_PER_LSB = (_ADC_MV_PER_LSB * VBAT_DIVIDER_COMP);
