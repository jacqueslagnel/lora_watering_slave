/**
 * @file lora_slave.cpp
 * @brief Implementation of the LoRa P2P watering slave protocol controller.
 *
 * This file implements radio initialization for supported boards, HELLO/ACK
 * exchange, command decoding, command acknowledgments, battery/status payloads,
 * watering command integration, and BLE OTA mode entry.
 */
#include "lora_slave.h"
#include "board_config.h"
#include "ble_ota.h"

// Inclusions spécifiques RAK4631
#ifdef BOARD_RAK4631
#include <Adafruit_TinyUSB.h>
#endif

#include <SX126x-Arduino.h>

// SPI dédié pour T114 (NRF_SPIM2)
#ifdef BOARD_T114
#include <SPI.h>
#endif

#include "watchdog_simple.h"

/**
 * @brief Debug serial stream used by this module.
 */
#define DBG Serial

// Sélection de la LED de statut selon la carte
#ifdef BOARD_RAK4631
#if defined(LED_BLUE)
/**
 * @brief Board-selected status LED pin.
 */
#define STATUS_LED LED_BLUE
#elif defined(LED_GREEN)
/**
 * @brief Board-selected status LED pin.
 */
#define STATUS_LED LED_GREEN
#elif defined(LED_BUILTIN)
/**
 * @brief Board-selected status LED pin.
 */
#define STATUS_LED LED_BUILTIN
#else
/**
 * @brief Sentinel value used when no status LED is available.
 */
#define STATUS_LED -1
#endif
#else // BOARD_T114
#if defined(LED_GREEN)
/**
 * @brief Board-selected status LED pin.
 */
#define STATUS_LED LED_GREEN
#elif defined(LED_BUILTIN)
/**
 * @brief Board-selected status LED pin.
 */
#define STATUS_LED LED_BUILTIN
#else
/**
 * @brief Sentinel value used when no status LED is available.
 */
#define STATUS_LED -1
#endif
#endif

// #else // BOARD_T114
// #ifdef BOARD_T114
// //#define STATUS_LED -1
// #elif defined(LED_BLUE)
// #define STATUS_LED LED_BLUE
// #elif defined(LED_GREEN)
// #define STATUS_LED LED_GREEN
// #elif defined(LED_BUILTIN)
// #define STATUS_LED LED_BUILTIN
// #else
// #define STATUS_LED -1
// #endif
// #endif

/**
 * @brief Radio callback table passed to the SX126x driver.
 */
static RadioEvents_t RadioEvents;

#ifdef BOARD_T114
/**
 * @brief SX126x hardware configuration for Heltec T114.
 */
static hw_config hwConfig;
// SX126x-Arduino.h déclare `extern SPIClass SPI_LORA;` — on DOIT le définir ici sans static
/**
 * @brief Dedicated SPI instance required by the T114 LoRa radio.
 */
SPIClass SPI_LORA(NRF_SPIM2, PIN_LORA_MISO, PIN_LORA_SCK, PIN_LORA_MOSI);
#endif

/**
 * @brief Active LoRaSlave instance used by static radio callback bridges.
 */
LoRaSlave *LoRaSlave::instance = nullptr;

/**
 * @brief Constructs a LoRaSlave with cleared buffers and default runtime state.
 */
LoRaSlave::LoRaSlave()
    : txDoneFlag(false),
      txTimeoutFlag(false),
      rxDoneFlag(false),
      rxTimeoutFlag(false),
      rxErrorFlag(false),
      rxSize(0),
      lastRssi(0),
      lastSnr(0),
      seqNum(0),
      ledState(false),
      sleepTimeMs(SLEEP_TIME_DEFAULT_MS),
      _lastAckHelloMs(0),
      _lastCmdSeq(0),
      _justBooted(true)
{
    memset(txBuffer, 0, sizeof(txBuffer));
    memset(rxBuffer, 0, sizeof(rxBuffer));
}

// ============================================================================
// begin()
// ============================================================================
void LoRaSlave::begin()
{
    instance = this;

    ledInit();

#ifdef BOARD_T114
    // Broche enable de l'ADC batterie (T114 uniquement)
    pinMode(PIN_BAT_ADC_EN, OUTPUT);
    digitalWrite(PIN_BAT_ADC_EN, LOW);
#endif

    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    delay(5);

#ifdef BOARD_RAK4631
    // Lecture de chauffe pour stabiliser l'ADC
    (void)readBatteryMilliVolts();
#endif

    DBG.println("Slave demarrant sur " BOARD_NAME_STR);
    DBG.println("Init Radio....");

    watering.begin();

#ifdef BOARD_T114
    SPI_LORA.begin();
    delay(5);
#endif

    if (!initRadio())
    {
        DBG.println("ERR radio init failed");
        while (1)
        {
#if STATUS_LED >= 0
            digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
#endif
            delay(200);
        }
    }

    // Anti-collision couche 1 : stagger au boot proportionnel au NODE_ID
    // Garantit la désynchronisation initiale même après un reboot général simultané.
    uint32_t staggerMs = (uint32_t)NODE_ID * 1000UL + (uint32_t)random(0, 1000);
    DBG.print("[AC] boot stagger ms=");
    DBG.println(staggerMs);
    {
        uint32_t t0 = millis();
        while ((millis() - t0) < staggerMs)
        {
            watchdogFeed();
            delay(50);
        }
    }

    _lastAckHelloMs = millis();
}

// ============================================================================
// loop()
// ============================================================================
void LoRaSlave::loop()
{
    watchdogFeed();
    watering.loop();
    checkWateringSafety();

    bool helloOk = sendHelloAndWaitAck();

    if (helloOk)
    {
        (void)processOptionalCommand();
    }

    watering.loop();
    checkWateringSafety();

    // Anti-collision couche 2 : jitter aléatoire sur le sleep pour éviter la resynchronisation
    nodeSleep(sleepTimeMs + (uint32_t)random(0, 2000));
}

void LoRaSlave::checkWateringSafety()
{
    if (watering.isWateringActive() &&
        (uint32_t)(millis() - _lastAckHelloMs) > WATERING_SUPERVISION_TIMEOUT_MS)
    {
        Serial.println("[SAFE] Watering stopped: ACK_HELLO supervision lost");
        watering.abortWatering();
    }
}

// ============================================================================
// LED — init et pilotage selon la carte
// ============================================================================
inline void LoRaSlave::ledInit()
{
#if STATUS_LED >= 0
    pinMode(STATUS_LED, OUTPUT);
    digitalWrite(STATUS_LED, LED_OFF_LEVEL);
#endif

    ledState = false;
}

inline void LoRaSlave::ledSet(bool on)
{
#if STATUS_LED >= 0
    digitalWrite(STATUS_LED, on ? LED_ON_LEVEL : LED_OFF_LEVEL);
#endif

    ledState = on;
}

// ============================================================================
// Divers utilitaires
// ============================================================================
void LoRaSlave::resetFlags()
{
    txDoneFlag = false;
    txTimeoutFlag = false;
    rxDoneFlag = false;
    rxTimeoutFlag = false;
    rxErrorFlag = false;
    rxSize = 0;
}

uint8_t LoRaSlave::nextSeq()
{
    seqNum++;
    if (seqNum == 0)
    {
        seqNum = 1;
    }
    return seqNum;
}

uint8_t LoRaSlave::getStatusFlag() const { return 0; }

uint32_t LoRaSlave::getUptimeSec() const { return millis() / 1000UL; }

// ============================================================================
// Batterie — lecture selon la carte
// ============================================================================
uint16_t LoRaSlave::readBatteryMilliVolts() const
{
    uint32_t acc = 0;

#ifdef BOARD_T114
    // T114 : activer la broche enable, attendre la stabilisation
    pinMode(PIN_BAT_ADC_EN, OUTPUT);
    digitalWrite(PIN_BAT_ADC_EN, HIGH);
    delay(20);
    pinMode(PIN_BAT_ADC, INPUT);

    // Echantillons de chauffe (rejetes)
    for (uint8_t i = 0; i < BAT_WARMUP_COUNT; i++)
    {
        watchdogFeed();
        (void)analogRead(PIN_BAT_ADC);
        delay(BAT_WARMUP_DELAY_MS);
    }

    for (uint8_t i = 0; i < BAT_SAMPLE_COUNT; i++)
    {
        watchdogFeed();
        acc += analogRead(PIN_BAT_ADC);
        delay(BAT_SAMPLE_DELAY_MS);
    }

    digitalWrite(PIN_BAT_ADC_EN, LOW);

#else // BOARD_RAK4631
    // RAK4631 : lecture directe sur PIN_VBAT (ou A0 en fallback)
#if defined(PIN_VBAT)
    const uint32_t vbat_pin = PIN_VBAT;
#else
    const uint32_t vbat_pin = A0;
#endif

    for (uint8_t i = 0; i < BAT_SAMPLE_COUNT; i++)
    {
        watchdogFeed();
        acc += analogRead(vbat_pin);
        delay(BAT_SAMPLE_DELAY_MS);
    }
#endif

    float raw = (float)acc / BAT_SAMPLE_COUNT;
    float mv = raw * REAL_VBAT_MV_PER_LSB;

    if (mv < 0.0f)
    {
        mv = 0.0f;
    }
    if (mv > 65535.0f)
    {
        mv = 65535.0f;
    }

    return (uint16_t)(mv + 0.5f);
}

// ============================================================================
// CRC et construction de trames
// ============================================================================
uint16_t LoRaSlave::crc16Ccitt(const uint8_t *data, uint16_t len) const
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;

        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc = (uint16_t)(crc << 1);
        }
    }

    return crc;
}

uint8_t LoRaSlave::getFrameTotalLength(uint8_t payloadLen) const
{
    return (uint8_t)(4 + payloadLen + 2);
}

uint8_t LoRaSlave::buildFrame(uint8_t type, uint8_t nodeId, uint8_t seq,
                              const uint8_t *payload, uint8_t payloadLen)
{
    txBuffer[0] = type;
    txBuffer[1] = nodeId;
    txBuffer[2] = seq;
    txBuffer[3] = payloadLen;

    for (uint8_t i = 0; i < payloadLen; i++)
    {
        txBuffer[4 + i] = payload[i];
    }

    uint16_t crc = crc16Ccitt(txBuffer, (uint16_t)(4 + payloadLen));
    txBuffer[4 + payloadLen] = (uint8_t)(crc & 0xFF);
    txBuffer[5 + payloadLen] = (uint8_t)((crc >> 8) & 0xFF);

    return getFrameTotalLength(payloadLen);
}

bool LoRaSlave::validateFrame(uint8_t expectedType, uint8_t expectedNode,
                              uint8_t expectedSeq) const
{
    if (rxSize < 6)
    {
        DBG.println("ERR frame too short");
        return false;
    }
    if (rxBuffer[0] != expectedType)
    {
        DBG.println("ERR frame type");
        return false;
    }
    if ((rxBuffer[1] != expectedNode) && (rxBuffer[1] != 0xFF))
    {
        DBG.println("ERR node id");
        return false;
    }
    if (rxBuffer[2] != expectedSeq)
    {
        DBG.println("ERR seq");
        return false;
    }

    uint8_t payloadLen = rxBuffer[3];
    uint16_t expectedTotal = (uint16_t)(4 + payloadLen + 2);

    if (expectedTotal > MAX_BUFFER_SIZE)
    {
        DBG.println("ERR frame oversize");
        return false;
    }
    if (rxSize != expectedTotal)
    {
        DBG.println("ERR frame length");
        return false;
    }

    uint16_t rxCrc = (uint16_t)rxBuffer[4 + payloadLen] |
                     ((uint16_t)rxBuffer[5 + payloadLen] << 8);
    uint16_t calcCrc = crc16Ccitt(rxBuffer, (uint16_t)(4 + payloadLen));

    if (rxCrc != calcCrc)
    {
        DBG.print("ERR CRC rx=");
        DBG.print(rxCrc, HEX);
        DBG.print(" calc=");
        DBG.println(calcCrc, HEX);
        return false;
    }

    return true;
}

void LoRaSlave::printFrameDebug(const char *prefix, const uint8_t *buf, uint8_t len) const
{
    DBG.print(prefix);
    DBG.print(" len=");
    DBG.print(len);
    DBG.print(" data=");

    for (uint8_t i = 0; i < len; i++)
    {
        if (buf[i] < 16)
        {
            DBG.print('0');
        }
        DBG.print(buf[i], HEX);
        DBG.print(' ');
    }

    DBG.println();
}

// ============================================================================
// initRadio() — initialisation radio selon la carte
// ============================================================================
bool LoRaSlave::initRadio()
{
    DBG.println("[LORA] initRadio()");

#ifdef BOARD_T114
    DBG.println("[LORA] lora_hardware_init()...");

    hwConfig.CHIP_TYPE = SX1262_CHIP;
    hwConfig.PIN_LORA_RESET = PIN_LORA_RESET;
    hwConfig.PIN_LORA_NSS = PIN_LORA_NSS;
    hwConfig.PIN_LORA_SCLK = PIN_LORA_SCK;
    hwConfig.PIN_LORA_MISO = PIN_LORA_MISO;
    hwConfig.PIN_LORA_MOSI = PIN_LORA_MOSI;
    hwConfig.PIN_LORA_DIO_1 = PIN_LORA_DIO1;
    hwConfig.PIN_LORA_BUSY = PIN_LORA_BUSY;
    hwConfig.RADIO_TXEN = -1;
    hwConfig.RADIO_RXEN = -1;
    hwConfig.USE_DIO2_ANT_SWITCH = LORA_USE_DIO2_ANT_SWITCH;
    hwConfig.USE_DIO3_TCXO = LORA_USE_DIO3_TCXO;
    hwConfig.USE_DIO3_ANT_SWITCH = LORA_USE_DIO3_ANT_SWITCH;
    hwConfig.USE_LDO = LORA_USE_LDO;
    hwConfig.USE_RXEN_ANT_PWR = LORA_USE_RXEN_ANT_PWR;
    hwConfig.TCXO_CTRL_VOLTAGE = LORA_TCXO_VOLTAGE;

    uint32_t initResult = lora_hardware_init(hwConfig);
    DBG.print("[LORA] init_result = ");
    DBG.println(initResult);

    if (initResult != 0)
    {
        return false;
    }

#else // BOARD_RAK4631
    lora_rak4630_init();
#endif

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
    RadioEvents.CadDone = OnCadDone;

    Radio.Init(&RadioEvents);

#if LORA_SLEEP_BEFORE_CHANNEL
    Radio.Sleep(); // Requis sur T114 avant SetChannel
#endif

    Radio.SetChannel(RF_FREQUENCY);

    Radio.SetTxConfig(
        MODEM_LORA, TX_OUTPUT_POWER, 0,
        LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD,
        true, 0, 0, LORA_IQ_INVERSION, TX_TIMEOUT_VALUE);

    Radio.SetRxConfig(
        MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE,
        0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
        LORA_FIX_LENGTH_PAYLOAD, 0, true, 0, 0, LORA_IQ_INVERSION, true);

    DBG.println("[LORA] init OK");
    return true;
}

// ============================================================================
// sendFrameAndWaitTx()
// T114  : interruptions matérielles => pas de polling IrqProcess
// RAK4631 : la lib exige Radio.IrqProcess() à chaque itération
// ============================================================================
bool LoRaSlave::sendFrameAndWaitTx(uint8_t totalLen)
{
    resetFlags();
    printFrameDebug("TX", txBuffer, totalLen);
    Radio.Send(txBuffer, totalLen);

    uint32_t startMs = millis();
    while ((millis() - startMs) < TX_TIMEOUT_VALUE)
    {
#if LORA_NEEDS_IRQ_PROCESS
        Radio.IrqProcess();
#endif
        watchdogFeed();

        if (txDoneFlag)
        {
            return true;
        }
        if (txTimeoutFlag)
        {
            DBG.println("ERR TX timeout");
            return false;
        }

        delay(1);
    }

    DBG.println("ERR TX wait timeout");
    return false;
}

// ============================================================================
// waitForFrame()
// ============================================================================
bool LoRaSlave::waitForFrame(uint32_t timeoutMs)
{
    resetFlags();
    Radio.Rx(timeoutMs);

    // Marge de securite : T114 +100 ms, RAK4631 +50 ms
#ifdef BOARD_T114
    const uint32_t overhead = 100U;
#else
    const uint32_t overhead = 50U;
#endif

    uint32_t startMs = millis();
    while ((millis() - startMs) < (timeoutMs + overhead))
    {
#if LORA_NEEDS_IRQ_PROCESS
        Radio.IrqProcess();
#endif
        watchdogFeed();

        if (rxDoneFlag)
        {
            if (rxSize > MAX_BUFFER_SIZE)
            {
                DBG.println("ERR RX oversize");
                return false;
            }
            printFrameDebug("RX", rxBuffer, (uint8_t)rxSize);
            return true;
        }

        if (rxTimeoutFlag)
        {
            return false;
        }

        if (rxErrorFlag)
        {
            DBG.println("ERR RX error");
            return false;
        }

        delay(1);
    }

    return false;
}

// ============================================================================
// Protocole HELLO / ACK
// ============================================================================
bool LoRaSlave::sendHelloAndWaitAckOnce()
{
    uint8_t seq = nextSeq();

    DBG.print("[PROTO] sendHelloAndWaitAck(), seq=");
    DBG.println(seq);

    uint8_t helloPayload[1];
    helloPayload[0] = getHelloFlags();

    uint8_t totalLen = buildFrame(FRAME_HELLO, NODE_ID, seq, helloPayload, sizeof(helloPayload));

    if (!sendFrameAndWaitTx(totalLen))
    {
        DBG.println("[PROTO] HELLO TX failed");
        return false;
    }

    uint32_t deadline = millis() + ACK_WAIT_MS;

    while ((int32_t)(millis() - deadline) < 0)
    {
        uint32_t remainingMs = (uint32_t)(deadline - millis());
        if (remainingMs > 100UL)
        {
            remainingMs = 100UL;
        }

        if (!waitForFrame(remainingMs))
        {
            continue;
        }

        if (rxSize < 6)
        {
            DBG.println("[PROTO] Ignore short frame while waiting ACK_HELLO");
            continue;
        }

        if (rxBuffer[0] != FRAME_ACK_HELLO)
        {
            DBG.print("[PROTO] Ignore non-ACK frame while waiting ACK_HELLO: type=0x");
            DBG.println(rxBuffer[0], HEX);
            continue;
        }

        if (rxBuffer[1] != NODE_ID)
        {
            DBG.print("[PROTO] Ignore ACK for other node: ");
            DBG.println(rxBuffer[1]);
            continue;
        }

        if (rxBuffer[2] != seq)
        {
            DBG.print("[PROTO] Ignore ACK with wrong seq: ");
            DBG.println(rxBuffer[2]);
            continue;
        }

        if (!validateFrame(FRAME_ACK_HELLO, NODE_ID, seq))
        {
            DBG.println("[PROTO] Ignore invalid ACK_HELLO (CRC/len)");
            continue;
        }

        _lastAckHelloMs = millis();
        _justBooted = false;
        DBG.println("ACK_HELLO OK");
        return true;
    }

    DBG.println("[PROTO] No ACK_HELLO");
    return false;
}

bool LoRaSlave::sendHelloAndWaitAck()
{
    for (uint8_t attempt = 0; attempt < HELLO_MAX_RETRIES; attempt++)
    {
        DBG.print("[PROTO] HELLO attempt ");
        DBG.print(attempt + 1);
        DBG.print(" / ");
        DBG.println(HELLO_MAX_RETRIES);

        if (sendHelloAndWaitAckOnce())
        {
            return true;
        }

        if (attempt + 1 < HELLO_MAX_RETRIES)
        {
            // Anti-collision couche 3 : gap aléatoire pour briser un verrou de collision
            watchdogFeed();
            delay((uint32_t)random(80, 350));
        }
    }

    return false;
}

// ============================================================================
// sendAckCmd()
// ============================================================================

bool LoRaSlave::sendAckCmd(uint8_t seq, uint8_t cmdCode, uint8_t ackStatus,
                           const uint8_t *extraPayload, uint8_t extraLen)
{
    uint8_t payload[48];

    if (extraLen > sizeof(payload) - 2)
    {
        extraLen = sizeof(payload) - 2;
    }

    payload[0] = cmdCode;
    payload[1] = ackStatus;

    for (uint8_t i = 0; i < extraLen; i++)
    {
        payload[2 + i] = extraPayload[i];
    }

    uint8_t payloadLen = (uint8_t)(2 + extraLen);
    uint8_t totalLen = buildFrame(FRAME_ACK_CMD, NODE_ID, seq, payload, payloadLen);

    DBG.print("ACK_CMD TX seq=");
    DBG.print(seq);
    DBG.print(" cmd=");
    DBG.print(cmdCode);
    DBG.print(" status=");
    DBG.println(ackStatus);

    return sendFrameAndWaitTx(totalLen);
}

// ============================================================================
// Helper : remplissage du statut arrosage dans le payload extra
// ============================================================================
static void fillWaterStatusExtra(uint8_t *extra, Watering &watering)
{
    uint8_t valveOpen = watering.isValveOpen() ? 1 : 0;
    uint8_t wateringActive = watering.isWateringActive() ? 1 : 0;
    uint16_t durationSec = watering.getProgrammedDurationSec();
    uint16_t remainingSec = watering.getRemainingDurationSec();
    uint32_t flowPulses = watering.getFlowPulses();
    uint16_t litres = watering.getLitres16();
    uint32_t postClosePulses = watering.getPostClosePulses();
    uint8_t postCloseLeakDetected = watering.isPostCloseLeakDetected();

    extra[0] = valveOpen;
    extra[1] = wateringActive;

    extra[2] = (uint8_t)(durationSec & 0xFF);
    extra[3] = (uint8_t)((durationSec >> 8) & 0xFF);

    extra[4] = (uint8_t)(remainingSec & 0xFF);
    extra[5] = (uint8_t)((remainingSec >> 8) & 0xFF);

    extra[6] = (uint8_t)(flowPulses & 0xFF);
    extra[7] = (uint8_t)((flowPulses >> 8) & 0xFF);
    extra[8] = (uint8_t)((flowPulses >> 16) & 0xFF);
    extra[9] = (uint8_t)((flowPulses >> 24) & 0xFF);

    extra[10] = (uint8_t)(litres & 0xFF);
    extra[11] = (uint8_t)((litres >> 8) & 0xFF);

    extra[12] = (uint8_t)(postClosePulses & 0xFF);
    extra[13] = (uint8_t)((postClosePulses >> 8) & 0xFF);
    extra[14] = (uint8_t)((postClosePulses >> 16) & 0xFF);
    extra[15] = (uint8_t)((postClosePulses >> 24) & 0xFF);

    extra[16] = postCloseLeakDetected;
}

// ============================================================================
// processOptionalCommand() — fenetre multi-trames (implémentation T114)
// ============================================================================
bool LoRaSlave::processOptionalCommand()
{
    DBG.println("[PROTO] Waiting optional CMD");

    uint32_t startWindowMs = millis();

    while ((millis() - startWindowMs) < CMD_WAIT_MS)
    {
        watchdogFeed();
        watering.loop();

        uint32_t elapsed = millis() - startWindowMs;
        uint32_t remain = (elapsed < CMD_WAIT_MS) ? (CMD_WAIT_MS - elapsed) : 0;

        if (remain == 0)
        {
            break;
        }
        if (remain < 50)
        {
            remain = 50;
        }

        if (!waitForFrame(remain))
        {
            continue;
        }

        if (rxSize < 6)
        {
            DBG.println("[PROTO] Ignore short frame");
            continue;
        }

        uint8_t frameType = rxBuffer[0];
        uint8_t targetNode = rxBuffer[1];
        uint8_t seq = rxBuffer[2];

        if (frameType != FRAME_CMD)
        {
            DBG.print("[PROTO] Ignore non-CMD frame type=0x");
            if (frameType < 16)
            {
                DBG.print('0');
            }
            DBG.println(frameType, HEX);
            continue;
        }

        if ((targetNode != NODE_ID) && (targetNode != 0xFF))
        {
            DBG.print("[PROTO] Ignore CMD for node ");
            DBG.println(targetNode);
            continue;
        }

        if (!validateFrame(FRAME_CMD, NODE_ID, seq))
        {
            DBG.println("[PROTO] Invalid CMD for me");
            continue;
        }

        uint8_t payloadLen = rxBuffer[3];
        if (payloadLen < 1)
        {
            DBG.println("[PROTO] Ignore empty CMD");
            continue;
        }

        uint8_t cmdCode = rxBuffer[4];

        if (seq == _lastCmdSeq)
        {
            DBG.print("[PROTO] Duplicate CMD seq=");
            DBG.println(seq);
            delay(ACK_TURNAROUND_DELAY_MS);
            sendAckCmd(seq, cmdCode, ACK_STATUS_OK, nullptr, 0);
            return true;
        }
        _lastCmdSeq = seq;

        DBG.print("[PROTO] CMD=0x");
        if (cmdCode < 16)
        {
            DBG.print('0');
        }
        DBG.println(cmdCode, HEX);

        switch (cmdCode)
        {

        case CMD_PING:
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_PING, ACK_STATUS_OK, nullptr, 0);

        case CMD_LED_ON:
            ledSet(true);
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_LED_ON, ACK_STATUS_OK, nullptr, 0);

        case CMD_LED_OFF:
            ledSet(false);
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_LED_OFF, ACK_STATUS_OK, nullptr, 0);

        case CMD_TEXT_MESSAGE:
        {
            DBG.print("[PROTO] TEXT: ");
            for (uint8_t i = 1; i < payloadLen; i++)
            {
                DBG.write((char)rxBuffer[4 + i]);
            }
            DBG.println();
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_TEXT_MESSAGE, ACK_STATUS_OK, nullptr, 0);
        }

        case CMD_BATTERY:
        {
            uint16_t batteryMv = readBatteryMilliVolts();
            uint8_t extra[2];
            extra[0] = (uint8_t)(batteryMv & 0xFF);
            extra[1] = (uint8_t)((batteryMv >> 8) & 0xFF);
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_BATTERY, ACK_STATUS_OK, extra, sizeof(extra));
        }

        case CMD_STATUS:
        {
            uint16_t batteryMv = readBatteryMilliVolts();
            uint8_t statusFlag = getStatusFlag();
            uint32_t uptimeSec = getUptimeSec();

            uint8_t extra[7];
            extra[0] = (uint8_t)(batteryMv & 0xFF);
            extra[1] = (uint8_t)((batteryMv >> 8) & 0xFF);
            extra[2] = statusFlag;
            extra[3] = (uint8_t)(uptimeSec & 0xFF);
            extra[4] = (uint8_t)((uptimeSec >> 8) & 0xFF);
            extra[5] = (uint8_t)((uptimeSec >> 16) & 0xFF);
            extra[6] = (uint8_t)((uptimeSec >> 24) & 0xFF);

            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_STATUS, ACK_STATUS_OK, extra, sizeof(extra));
        }

        case CMD_SET_PERIOD:
        {
            if (payloadLen < 3)
            {
                delay(ACK_TURNAROUND_DELAY_MS);
                return sendAckCmd(seq, CMD_SET_PERIOD, ACK_STATUS_ERROR, nullptr, 0);
            }

            uint16_t seconds = (uint16_t)rxBuffer[5] | ((uint16_t)rxBuffer[6] << 8);

            if (seconds == 0)
            {
                delay(ACK_TURNAROUND_DELAY_MS);
                return sendAckCmd(seq, CMD_SET_PERIOD, ACK_STATUS_ERROR, nullptr, 0);
            }

            sleepTimeMs = (uint32_t)seconds * 1000UL;
            DBG.print("[PROTO] New sleep period ms=");
            DBG.println(sleepTimeMs);

            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_SET_PERIOD, ACK_STATUS_OK, nullptr, 0);
        }

        case CMD_REBOOT:
        {
            delay(ACK_TURNAROUND_DELAY_MS);
            sendAckCmd(seq, CMD_REBOOT, ACK_STATUS_OK, nullptr, 0);
            delay(100);
            NVIC_SystemReset();
            return true;
        }

        case CMD_VALVE_OPEN:
        {
            watering.openValve();
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_VALVE_OPEN,
                              watering.isValveOpen() ? ACK_STATUS_OK : ACK_STATUS_ERROR, nullptr, 0);
        }

        case CMD_VALVE_CLOSE:
        {
            bool ok = watering.closeValve();
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_VALVE_CLOSE,
                              ok ? ACK_STATUS_OK : ACK_STATUS_ERROR, nullptr, 0);
        }

        case CMD_WATER_TIME:
        {
            if (payloadLen < 3)
            {
                delay(ACK_TURNAROUND_DELAY_MS);
                return sendAckCmd(seq, CMD_WATER_TIME, ACK_STATUS_ERROR, nullptr, 0);
            }

            uint16_t seconds = (uint16_t)rxBuffer[5] | ((uint16_t)rxBuffer[6] << 8);

            if (!watering.startWatering(seconds))
            {
                delay(ACK_TURNAROUND_DELAY_MS);
                return sendAckCmd(seq, CMD_WATER_TIME, ACK_STATUS_ERROR, nullptr, 0);
            }

            uint8_t extra[17];
            fillWaterStatusExtra(extra, watering);
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_WATER_TIME, ACK_STATUS_OK, extra, sizeof(extra));
        }

        case CMD_WATER_ABORT:
        {
            if (!watering.abortWatering())
            {
                delay(ACK_TURNAROUND_DELAY_MS);
                return sendAckCmd(seq, CMD_WATER_ABORT, ACK_STATUS_ERROR, nullptr, 0);
            }

            uint8_t extra[17];
            fillWaterStatusExtra(extra, watering);
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_WATER_ABORT, ACK_STATUS_OK, extra, sizeof(extra));
        }

        case CMD_WATER_STATUS:
        {
            uint8_t extra[17];
            fillWaterStatusExtra(extra, watering);
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, CMD_WATER_STATUS, ACK_STATUS_OK, extra, sizeof(extra));
        }

        case CMD_OTA_MODE:
        {
            delay(ACK_TURNAROUND_DELAY_MS);
            sendAckCmd(seq, CMD_OTA_MODE, ACK_STATUS_OK, nullptr, 0);
            delay(100);
            enterBleOtaMode();
            return true;
        }

        default:
            delay(ACK_TURNAROUND_DELAY_MS);
            return sendAckCmd(seq, cmdCode, ACK_STATUS_ERROR, nullptr, 0);
        }
    }

    DBG.println("[PROTO] No optional CMD in window");
    return false;
}

// ============================================================================
// nodeSleep()
// ============================================================================
void LoRaSlave::nodeSleep(uint32_t sleepMsValue)
{
    uint32_t startMs = millis();
    while ((millis() - startMs) < sleepMsValue)
    {
        watchdogFeed();
        watering.loop();
        delay(50);
        __WFI();
    }
}

// ============================================================================
// Callbacks radio (statiques)
// ============================================================================
void LoRaSlave::OnTxDone(void)
{
    if (instance != nullptr)
    {
        instance->txDoneFlag = true;
        DBG.println("[LORA] TX done");
    }
}

void LoRaSlave::OnTxTimeout(void)
{
    if (instance != nullptr)
    {
        instance->txTimeoutFlag = true;
        DBG.println("[LORA] TX timeout");
    }
}

void LoRaSlave::OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    if (instance == nullptr)
    {
        return;
    }

    if (size > sizeof(instance->rxBuffer))
    {
        size = sizeof(instance->rxBuffer);
    }

    memcpy(instance->rxBuffer, payload, size);
    instance->rxSize = size;
    instance->lastRssi = rssi;
    instance->lastSnr = snr;
    instance->rxDoneFlag = true;

    DBG.print("[LORA] RX done, size=");
    DBG.print(size);
    DBG.print(" RSSI=");
    DBG.print(rssi);
    DBG.print(" SNR=");
    DBG.println(snr);

    instance->printFrameDebug("RX", instance->rxBuffer, (uint8_t)instance->rxSize);
}

void LoRaSlave::OnRxTimeout(void)
{
    if (instance != nullptr)
    {
        instance->rxTimeoutFlag = true;
    }
}

void LoRaSlave::OnRxError(void)
{
    if (instance != nullptr)
    {
        instance->rxErrorFlag = true;
        DBG.println("[LORA] RX error");
    }
}

void LoRaSlave::OnCadDone(bool cadResult)
{
    (void)cadResult;
}

void LoRaSlave::enterBleOtaMode()
{
    BleOta::requestBoot(BleOta::DURATION_DEFAULT_SEC);
    delay(100);
    NVIC_SystemReset();
}

uint8_t LoRaSlave::getHelloFlags() const
{
    uint8_t flags = 0;

    if (getStatusFlag())
    {
        flags |= 0x01;
    }

    if (_justBooted)
    {
        flags |= 0x80;
    }

    return flags;
}
