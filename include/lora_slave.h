/**
 * @file lora_slave.h
 * @brief LoRa P2P slave node controller interface.
 *
 * This file declares the LoRaSlave class, which sends HELLO frames to the
 * master, waits for acknowledgments, receives optional commands, controls local
 * LED and watering functions, reports status, and can enter BLE OTA mode.
 */
#pragma once

#include <Arduino.h>
#include "board_config.h"
#include "watering.h"

/**
 * @class LoRaSlave
 * @brief Slave-side LoRa P2P protocol controller.
 *
 * LoRaSlave owns the radio callbacks, frame buffers, sequence numbers, command
 * handling, battery reporting, watering controller integration, and sleep
 * cadence for one node.
 */
class LoRaSlave
{
public:
    /**
     * @brief Static node identifier used in LoRa frames.
     */
    static constexpr uint8_t NODE_ID = 2;
    /**
     * @brief Constructs a slave controller with default runtime state.
     */
    LoRaSlave();

    /**
     * @brief Initializes LED, watering, and radio resources.
     */
    void begin();
    /**
     * @brief Runs one slave protocol cycle.
     */
    void loop();

private:
    /**
     * @enum FrameType
     * @brief Frame types exchanged with the LoRa master.
     */
    enum FrameType : uint8_t
    {
        FRAME_HELLO = 0x10,     ///< Node announcement frame.
        FRAME_ACK_HELLO = 0x11, ///< Master acknowledgment for HELLO.
        FRAME_CMD = 0x20,       ///< Command frame sent by the master.
        FRAME_ACK_CMD = 0x21    ///< Command acknowledgment sent by the node.
    };

    /**
     * @enum CommandType
     * @brief Command codes supported by the slave protocol.
     */
    enum CommandType : uint8_t
    {
        CMD_NONE = 0x00,         ///< No command.
        CMD_PING = 0x01,         ///< Connectivity test command.
        CMD_LED_ON = 0x02,       ///< Switch the local LED on.
        CMD_LED_OFF = 0x03,      ///< Switch the local LED off.
        CMD_TEXT_MESSAGE = 0x10, ///< Text message command acknowledged by the node.
        CMD_BATTERY = 0x11,      ///< Battery measurement command.
        CMD_REBOOT = 0x12,       ///< MCU reboot command.
        CMD_STATUS = 0x13,       ///< General node status query.
        CMD_SET_PERIOD = 0x14,   ///< Sleep/HELLO period configuration command.
        CMD_VALVE_OPEN = 0x20,   ///< Watering valve open command.
        CMD_VALVE_CLOSE = 0x21,  ///< Watering valve close command.
        CMD_WATER_TIME = 0x22,   ///< Timed watering start command.
        CMD_WATER_ABORT = 0x23,  ///< Timed watering abort command.
        CMD_WATER_STATUS = 0x24, ///< Watering status query command.
        CMD_OTA_MODE = 0x30      ///< BLE OTA mode command.
    };

    /**
     * @enum AckStatus
     * @brief Status values encoded in command acknowledgments.
     */
    enum AckStatus : uint8_t
    {
        ACK_STATUS_OK = 0x00,   ///< Command accepted or completed.
        ACK_STATUS_ERROR = 0x01 ///< Command rejected or failed.
    };

    static constexpr uint32_t RF_FREQUENCY = 865000000UL;  ///< LoRa RF frequency, in hertz.
    static constexpr int8_t TX_OUTPUT_POWER = 14;          ///< LoRa transmit power, in dBm.
    static constexpr uint8_t LORA_BANDWIDTH = 0;           ///< SX126x bandwidth code.
    static constexpr uint8_t LORA_SPREADING_FACTOR = 7;    ///< LoRa spreading factor.
    static constexpr uint8_t LORA_CODINGRATE = 1;          ///< SX126x coding rate code.
    static constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;    ///< LoRa preamble length, in symbols.
    static constexpr uint16_t LORA_SYMBOL_TIMEOUT = 0;     ///< LoRa symbol timeout for RX configuration.
    static constexpr bool LORA_FIX_LENGTH_PAYLOAD = false; ///< false to use variable-length packets.
    static constexpr bool LORA_IQ_INVERSION = false;       ///< false to keep standard IQ polarity.

    static constexpr uint32_t TX_TIMEOUT_VALUE = 3000UL;       ///< TX timeout in milliseconds.
    static constexpr uint32_t ACK_WAIT_MS = 1200UL;            ///< HELLO ACK wait duration in milliseconds.
    static constexpr uint32_t CMD_WAIT_MS = 1200UL;            ///< Optional command wait duration in milliseconds.
    static constexpr uint32_t SLEEP_TIME_DEFAULT_MS = 15000UL; ///< Default sleep delay between cycles.
    static constexpr uint32_t HELLO_RETRY_GAP_MS = 120UL;      ///< Delay between HELLO retries.
    static constexpr uint32_t ACK_TURNAROUND_DELAY_MS = 40UL;  ///< Delay before sending command ACKs.
    static constexpr uint8_t HELLO_MAX_RETRIES = 3;            ///< Maximum HELLO attempts per cycle.
    static constexpr uint8_t MAX_BUFFER_SIZE = 64;             ///< Radio frame buffer size in bytes.

    static constexpr uint32_t WATERING_SUPERVISION_TIMEOUT_MS = 90000UL; ///< Safety timeout for watering supervision.

    // Les constantes ADC (REAL_VBAT_MV_PER_LSB, VBAT_DIVIDER_COMP) sont
    // définies dans board_config.h en fonction de la carte sélectionnée.

    static LoRaSlave *instance; ///< Active instance used by static radio callbacks.

    /**
     * @brief Static radio TX done callback bridge.
     */
    static void OnTxDone(void);
    /**
     * @brief Static radio TX timeout callback bridge.
     */
    static void OnTxTimeout(void);
    /**
     * @brief Static radio RX done callback bridge.
     *
     * @param[in] payload Received bytes provided by the radio driver.
     * @param[in] size Number of received bytes.
     * @param[in] rssi Received RSSI.
     * @param[in] snr Received SNR.
     */
    static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
    /**
     * @brief Static radio RX timeout callback bridge.
     */
    static void OnRxTimeout(void);
    /**
     * @brief Static radio RX error callback bridge.
     */
    static void OnRxError(void);
    /**
     * @brief Static radio CAD done callback bridge.
     *
     * @param[in] cadResult CAD result provided by the radio driver.
     */
    static void OnCadDone(bool cadResult);

    /**
     * @brief Initializes the board LED GPIO.
     */
    inline void ledInit();
    /**
     * @brief Sets the board LED state.
     *
     * @param[in] on true to turn the LED on, false to turn it off.
     */
    inline void ledSet(bool on);

    /**
     * @brief Clears radio event flags and receive size.
     */
    void resetFlags();
    /**
     * @brief Returns and advances the protocol sequence number.
     *
     * @return Sequence number to use for the next outgoing frame.
     */
    uint8_t nextSeq();
    /**
     * @brief Builds the current node status flag byte.
     *
     * @return Status flag byte.
     */
    uint8_t getStatusFlag() const;
    /**
     * @brief Reads the battery voltage.
     *
     * @return Battery voltage in millivolts.
     */
    uint16_t readBatteryMilliVolts() const;
    /**
     * @brief Returns node uptime.
     *
     * @return Uptime in seconds.
     */
    uint32_t getUptimeSec() const;

    /**
     * @brief Computes CRC-16/CCITT over a byte buffer.
     *
     * @param[in] data Pointer to the bytes to checksum.
     * @param[in] len Number of bytes to process.
     * @return Computed CRC value.
     */
    uint16_t crc16Ccitt(const uint8_t *data, uint16_t len) const;
    /**
     * @brief Returns total frame length for a payload length.
     *
     * @param[in] payloadLen Payload length in bytes.
     * @return Total frame length in bytes.
     */
    uint8_t getFrameTotalLength(uint8_t payloadLen) const;
    /**
     * @brief Builds a protocol frame in txBuffer.
     *
     * @param[in] type Frame type.
     * @param[in] nodeId Node identifier.
     * @param[in] seq Sequence number.
     * @param[in] payload Payload bytes to copy.
     * @param[in] payloadLen Payload length in bytes.
     * @return Total frame length written to txBuffer.
     */
    uint8_t buildFrame(uint8_t type, uint8_t nodeId, uint8_t seq, const uint8_t *payload, uint8_t payloadLen);
    /**
     * @brief Validates the last received frame against expected header values.
     *
     * @param[in] expectedType Expected frame type.
     * @param[in] expectedNode Expected node identifier.
     * @param[in] expectedSeq Expected sequence number.
     * @return true if the frame is valid and matches the expected values, false otherwise.
     */
    bool validateFrame(uint8_t expectedType, uint8_t expectedNode, uint8_t expectedSeq) const;
    /**
     * @brief Prints a frame for diagnostics.
     *
     * @param[in] prefix Text printed before the frame bytes.
     * @param[in] buf Buffer containing bytes to print.
     * @param[in] len Number of bytes to print.
     */
    void printFrameDebug(const char *prefix, const uint8_t *buf, uint8_t len) const;

    /**
     * @brief Initializes the LoRa radio.
     *
     * @return true if radio initialization succeeds, false otherwise.
     */
    bool initRadio();
    /**
     * @brief Sends txBuffer and waits for TX completion.
     *
     * @param[in] totalLen Number of bytes to send from txBuffer.
     * @return true if transmission completed, false otherwise.
     */
    bool sendFrameAndWaitTx(uint8_t totalLen);
    /**
     * @brief Waits for one received frame.
     *
     * @param[in] timeoutMs Receive wait timeout in milliseconds.
     * @return true if a frame was received, false otherwise.
     */
    bool waitForFrame(uint32_t timeoutMs);

    /**
     * @brief Sends one HELLO frame and waits once for ACK.
     *
     * @return true if the HELLO is acknowledged, false otherwise.
     */
    bool sendHelloAndWaitAckOnce();
    /**
     * @brief Sends HELLO frames with retries until ACK or retry exhaustion.
     *
     * @return true if the HELLO is acknowledged, false otherwise.
     */
    bool sendHelloAndWaitAck();
    /**
     * @brief Waits for and processes one optional command from the master.
     *
     * @return true if a command was processed, false otherwise.
     */
    bool processOptionalCommand();
    /**
     * @brief Sends a command acknowledgment frame.
     *
     * @param[in] seq Sequence number to acknowledge.
     * @param[in] cmdCode Command code being acknowledged.
     * @param[in] ackStatus ACK status byte.
     * @param[in] extraPayload Optional extra payload bytes.
     * @param[in] extraLen Number of extra payload bytes.
     * @return true if the ACK frame was transmitted, false otherwise.
     */
    bool sendAckCmd(uint8_t seq, uint8_t cmdCode, uint8_t ackStatus, const uint8_t *extraPayload, uint8_t extraLen);

    /**
     * @brief Waits between protocol cycles.
     *
     * @param[in] sleepMs Wait duration in milliseconds.
     */
    void nodeSleep(uint32_t sleepMs);
    /**
     * @brief Runs watering safety supervision.
     */
    void checkWateringSafety();

    /**
     * @brief Requests BLE OTA mode entry.
     */
    void enterBleOtaMode();

private:
    uint8_t txBuffer[MAX_BUFFER_SIZE]; ///< Transmit frame buffer.
    uint8_t rxBuffer[MAX_BUFFER_SIZE]; ///< Receive frame buffer.

    volatile bool txDoneFlag;    ///< Set when radio TX completes.
    volatile bool txTimeoutFlag; ///< Set when radio TX times out.
    volatile bool rxDoneFlag;    ///< Set when radio RX completes.
    volatile bool rxTimeoutFlag; ///< Set when radio RX times out.
    volatile bool rxErrorFlag;   ///< Set when radio RX reports an error.

    volatile uint16_t rxSize;  ///< Size of the last received frame.
    volatile int16_t lastRssi; ///< RSSI of the last received frame.
    volatile int8_t lastSnr;   ///< SNR of the last received frame.

    uint8_t seqNum;           ///< Protocol sequence counter.
    bool ledState;            ///< Cached LED state.
    uint32_t sleepTimeMs;     ///< Current delay between protocol cycles.
    uint32_t _lastAckHelloMs; ///< millis() timestamp of the last acknowledged HELLO.
    uint8_t _lastCmdSeq;      ///< Last command sequence processed.
    Watering watering;        ///< Watering actuator and flow controller.

    bool _justBooted; ///< true until the first HELLO/status report has been sent.
    /**
     * @brief Builds HELLO flag bits.
     *
     * @return HELLO flags byte.
     */
    uint8_t getHelloFlags() const;
};
