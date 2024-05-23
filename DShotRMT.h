//
// Name:        DShotRMT.h
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#ifndef _DSHOTRMT_h
#define _DSHOTRMT_h

#include <Arduino.h>

// The RMT (Remote Control) module library is used for generating the DShot signal.
#include <driver/rmt.h>

// Stuff taken from betaflight:
static constexpr int DSHOT_MAX_COMMAND{47};
static constexpr int DSHOT_PROTOCOL_DETECTION_DELAY_MS{3000};
static constexpr int DSHOT_INITIAL_DELAY_US{10000};
static constexpr int DSHOT_COMMAND_DELAY_US{1000};
static constexpr int DSHOT_ESCINFO_DELAY_US{12000};
static constexpr int DSHOT_BEEP_DELAY_US{100000};




// 3D-Mode: positive direction
static constexpr int DSHOT3D_THROTTLE_N_MIN{48};
static constexpr int DSHOT3D_THROTTLE_N_MAX{1047};

// 3D-Mode: negative direction
static constexpr int DSHOT3D_THROTTLE_R_MIN{1048};
static constexpr int DSHOT3D_THROTTLE_R_MAX{2047};

static constexpr int DSHOT3D_RANGE = DSHOT3D_THROTTLE_N_MAX - DSHOT3D_THROTTLE_N_MIN;
static constexpr float DSHOT3D_SLOPE = DSHOT3D_RANGE / 100.0F;

// Normal-Mode
static constexpr int DSHOT_THROTTLE_MIN{48};
static constexpr int DSHOT_THROTTLE_MAX{2047};

static constexpr int DSHOT_RANGE = DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN;
static constexpr float DSHOT_SLOPE = DSHOT_RANGE / 100.0F;

// Defines the library version
constexpr auto DSHOT_LIB_VERSION = "0.2.4";

// Constants related to the DShot protocol
constexpr auto DSHOT_CLK_DIVIDER = 8;    // Slow down RMT clock to 0.1 microseconds / 100 nanoseconds per cycle -> 80 MHz/8: 10 MHz
constexpr auto DSHOT_PACKET_LENGTH = 17; // 16 bits + Last pack is the pause
constexpr auto DSHOT_NULL_PACKET = 0b0000000000000000;
constexpr auto DSHOT_PAUSE = 21; // 21-bit is recommended
constexpr auto DSHOT_PAUSE_BIT = 16;
constexpr auto F_CPU_RMT = APB_CLK_FREQ;
constexpr auto RMT_CYCLES_PER_SEC = (F_CPU_RMT / DSHOT_CLK_DIVIDER);
constexpr auto RMT_CYCLES_PER_ESP_CYCLE = (F_CPU / RMT_CYCLES_PER_SEC);

// DShot frame time, taken from https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#frame-structure, rounded to integers
// ToDo: Calculate from or synchronize with DShot timings in this library
static constexpr int dshot_frame_us[] = {
    0,
    107,
    53,
    27,
    13
};

static constexpr int telemetry_frame_us = 1.5 * 868;

// Enumeration for the DShot mode
typedef enum dshot_mode_e
{
    DSHOT_OFF,
    DSHOT150,
    DSHOT300,
    DSHOT600,
    DSHOT1200
} dshot_mode_t;

// Array of human-readable DShot mode names
static const char *const dshot_mode_name[] = {
    "DSHOT_OFF",
    "DSHOT150",
    "DSHOT300",
    "DSHOT600",
    "DSHOT1200"};

// Enumeration for telemetric request
typedef enum telemetric_request_e
{
    NO_TELEMETRIC,
    ENABLE_TELEMETRIC,
} telemetric_request_t;

// Structure for DShot packets
typedef struct dshot_packet_s
{
    uint16_t throttle_value : 11;
    telemetric_request_t telemetric_request : 1;
    uint16_t checksum : 4;
} dshot_packet_t;

// Structure for eRPM packets
typedef struct eRPM_packet_s
{
    uint16_t eRPM_data : 12;
    uint8_t checksum : 4;
} eRPM_packet_t;

// Structure for all settings for the DShot mode
typedef struct dshot_config_s
{
    dshot_mode_t mode;
    String name_str;
    bool is_bidirectional;
    gpio_num_t gpio_num;
    uint8_t pin_num;
    rmt_channel_t rmt_channel;
    uint8_t mem_block_num;
    uint16_t ticks_per_bit;
    uint8_t clk_div;
    uint16_t ticks_zero_high;
    uint16_t ticks_zero_low;
    uint16_t ticks_one_high;
    uint16_t ticks_one_low;
} dshot_config_t;

// The official DShot Commands
typedef enum dshot_cmd_e
{
    DSHOT_CMD_MOTOR_STOP = 0,          // Currently not implemented - STOP Motors
    DSHOT_CMD_BEEP1,                   // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP2,                   // Wait at least length of beep (380ms) before next command
    DSHOT_CMD_BEEP3,                   // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP4,                   // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_BEEP5,                   // Wait at least length of beep (400ms) before next command
    DSHOT_CMD_ESC_INFO,                // Currently not implemented
    DSHOT_CMD_SPIN_DIRECTION_1,        // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_2,        // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_OFF,             // Need 6x, no wait required
    DSHOT_CMD_3D_MODE_ON,              // Need 6x, no wait required
    DSHOT_CMD_SETTINGS_REQUEST,        // Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS,           // Need 6x, wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_NORMAL,   // Need 6x, no wait required
    DSHOT_CMD_SPIN_DIRECTION_REVERSED, // Need 6x, no wait required
    DSHOT_CMD_LED0_ON,                 // Currently not implemented
    DSHOT_CMD_LED1_ON,                 // Currently not implemented
    DSHOT_CMD_LED2_ON,                 // Currently not implemented
    DSHOT_CMD_LED3_ON,                 // Currently not implemented
    DSHOT_CMD_LED0_OFF,                // Currently not implemented
    DSHOT_CMD_LED1_OFF,                // Currently not implemented
    DSHOT_CMD_LED2_OFF,                // Currently not implemented
    DSHOT_CMD_LED3_OFF,                // Currently not implemented
    DSHOT_CMD_MAX = 47
} dshot_cmd_t;

// Telemetry data packages
typedef struct ESCTelemetry
{
    float temperature;
    float voltage;
    float current;
    float consumption;
    float RPM;
} ESCTelemetry_t;

typedef struct ESCTelemetryPackage
{
    unsigned long timestamp_us;
    ESCTelemetry_t data;
} ESCTelemetryPackage_t;

// The main DShotRMT class
class DShotRMT
{
public:
    // Constructor for the DShotRMT class
    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel);
    DShotRMT(uint8_t pin, uint8_t channel);

    DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, HardwareSerial *bus, const int8_t rxpin, const int8_t txpin);

    // Destructor for the DShotRMT class
    ~DShotRMT();

    // Copy constructor for the DShotRMT class
    DShotRMT(DShotRMT const &);

    void setup(bool is_3D, uint8_t motor_poles);

    // The begin() function initializes the DShotRMT class with
    // a given DShot mode (DSHOT_OFF, DSHOT150, DSHOT300, DSHOT600, DSHOT1200)
    // and a bidirectional flag. It returns a boolean value
    // indicating whether or not the initialization was successful.
    bool begin(dshot_mode_t dshot_mode = DSHOT_OFF, bool is_bidirectional = false);

    // The sendThrottleValue() function sends a DShot packet with a given
    // throttle value (between 49 and 2047) and an optional telemetry
    // request flag.
    // void sendThrottleValue(uint16_t throttle_value, telemetric_request_t telemetric_request = NO_TELEMETRIC);
    void send_throttle_raw(uint16_t);

    void send_throttle(float);

    float read(void) {return m_throttle_des;};

    void send_cmd(uint16_t cmd, bool telem);

    bool set_3D_mode(bool);

    bool is_enabled(void) {return m_is_enabled;};
    void enable(void);
    void disable(void);

    ESCTelemetryPackage_t get_telemetry(void);

    // void enable();
    // void disable();

    int8_t is_tx_finished(void);

    float m_throttle_des{0.0F};
    uint16_t m_throttle_cmd{48};

private:
    rmt_item32_t dshot_tx_rmt_item[DSHOT_PACKET_LENGTH]; // An array of RMT items used to send a DShot packet.
    rmt_config_t dshot_tx_rmt_config;                    // The RMT configuration used for sending DShot packets.
    dshot_config_t dshot_config;                         // The configuration for the DShot mode.

    bool m_is_3D {false};
    bool m_is_enabled {false};

    // Telemetry
    HardwareSerial *m_telem_uart {nullptr};
    int8_t m_telem_rxpin {-1};
    int8_t m_telem_txpin {-1};
    bool m_telem_enabled {false};
    uint8_t m_serial_buffer[10];
    uint8_t m_received_bytes {0U};
    ESCTelemetryPackage_t m_telem_data { };
    unsigned long m_telem_timestamp_us {0UL};
    bool m_telem_requested {false};
    uint8_t m_motor_poles {14U};
    uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
    uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
    void receive_telemetry(void);

    dshot_packet_t generate_dshot_packet(uint16_t, telemetric_request_t);
    dshot_packet_t generate_dshot_packet(uint16_t);
    void send_dshot(void);

    rmt_item32_t *buildTxRmtItem(uint16_t parsed_packet);       // Constructs an RMT item from a parsed DShot packet.
    uint16_t calculateCRC(const dshot_packet_t &dshot_packet);  // Calculates the CRC checksum for a DShot packet.
    uint16_t parseRmtPaket(const dshot_packet_t &dshot_packet); // Parses an RMT packet to obtain a DShot packet.

    void sendRmtPaket(const dshot_packet_t &dshot_packet); // Sends a DShot packet via RMT.
};

#endif
