//
// Name:        DShotRMT.cpp
// Created: 	20.03.2021 00:49:15
// Author:  	derdoktor667
//

#include <DShotRMT.h>

// Constructor that takes gpio and rmtChannel as arguments
DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel)
{
    // Initialize the dshot_config structure with the arguments passed to the constructor
    dshot_config.gpio_num = gpio;
    dshot_config.pin_num = static_cast<uint8_t>(gpio);
    dshot_config.rmt_channel = rmtChannel;
    //dshot_config.mem_block_num = static_cast<uint8_t>(RMT_CHANNEL_MAX - static_cast<uint8_t>(rmtChannel));  // This uses all RMT shared memory (8 blocks @ 256 Byte) if RMT channel = 0 and 1 RMT shared memory if RMT channel = 7
    dshot_config.mem_block_num = 1;

    // Create an empty packet using the DSHOT_NULL_PACKET and the buildTxRmtItem function
    buildTxRmtItem(DSHOT_NULL_PACKET);
}

// Constructor that takes pin and channel as arguments
DShotRMT::DShotRMT(uint8_t pin, uint8_t channel)
{
    // Initialize the dshot_config structure with the arguments passed to the constructor
    dshot_config.gpio_num = static_cast<gpio_num_t>(pin);
    dshot_config.pin_num = pin;
    dshot_config.rmt_channel = static_cast<rmt_channel_t>(channel);
    //dshot_config.mem_block_num = RMT_CHANNEL_MAX - channel;
    dshot_config.mem_block_num = 1;

    // Create an empty packet using the DSHOT_NULL_PACKET and the buildTxRmtItem function
    buildTxRmtItem(DSHOT_NULL_PACKET);
}

DShotRMT::DShotRMT(gpio_num_t gpio, rmt_channel_t rmtChannel, HardwareSerial *uart, const int8_t rxpin, const int8_t txpin)
    : DShotRMT(gpio, rmtChannel)
{
    m_telem_uart = uart;
    m_telem_rxpin = rxpin;
    m_telem_txpin = txpin;
    m_telem_enabled = true;
}

DShotRMT::~DShotRMT()
{
    // Uninstall the RMT driver
    rmt_driver_uninstall(dshot_config.rmt_channel);
}

DShotRMT::DShotRMT(DShotRMT const &)
{
    // ...write me
}

bool DShotRMT::begin(dshot_mode_t dshot_mode, bool is_bidirectional)
{
    // Set DShot configuration parameters based on input parameters
    dshot_config.mode = dshot_mode;
    dshot_config.clk_div = DSHOT_CLK_DIVIDER;
    dshot_config.name_str = dshot_mode_name[dshot_mode];
    dshot_config.is_bidirectional = is_bidirectional;

    // Set timing parameters based on selected DShot mode
    // According to https://brushlesswhoop.com/dshot-and-bidirectional-dshot/ the below timings are not exactly correct...
    switch (dshot_config.mode)
    {
    case DSHOT150:
        dshot_config.ticks_per_bit = 64;
        dshot_config.ticks_zero_high = 24;
        dshot_config.ticks_one_high = 48;
        break;

    case DSHOT300:
        dshot_config.ticks_per_bit = 32;
        //dshot_config.ticks_per_bit = 33;
        dshot_config.ticks_zero_high = 12;
        dshot_config.ticks_one_high = 24;
        //dshot_config.ticks_one_high = 25;
        break;

    case DSHOT600:
        dshot_config.ticks_per_bit = 16;
        dshot_config.ticks_zero_high = 6;
        dshot_config.ticks_one_high = 12;
        break;

    case DSHOT1200:
        dshot_config.ticks_per_bit = 8;
        dshot_config.ticks_zero_high = 3;
        dshot_config.ticks_one_high = 6;
        break;

    // Default case to handle invalid input
    default:
        dshot_config.ticks_per_bit = 0;
        dshot_config.ticks_zero_high = 0;
        dshot_config.ticks_one_high = 0;
        break;
    }

    // Calculate low signal timing
    dshot_config.ticks_zero_low = (dshot_config.ticks_per_bit - dshot_config.ticks_zero_high);
    dshot_config.ticks_one_low = (dshot_config.ticks_per_bit - dshot_config.ticks_one_high);

    // Set up RMT configuration for DShot transmission
    dshot_tx_rmt_config.rmt_mode = RMT_MODE_TX;
    dshot_tx_rmt_config.channel = dshot_config.rmt_channel;
    dshot_tx_rmt_config.gpio_num = dshot_config.gpio_num;
    dshot_tx_rmt_config.mem_block_num = dshot_config.mem_block_num;
    dshot_tx_rmt_config.clk_div = dshot_config.clk_div;
    dshot_tx_rmt_config.tx_config.loop_en = false;
    dshot_tx_rmt_config.tx_config.carrier_en = false;
    dshot_tx_rmt_config.tx_config.idle_output_en = true;

    // Set idle level for RMT transmission based on input parameter
    if (dshot_config.is_bidirectional)
    {
        dshot_tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_HIGH;
    }
    else
    {
        dshot_tx_rmt_config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    }

    // Set up selected DShot mode
    rmt_config(&dshot_tx_rmt_config);

    // Install RMT driver
    esp_err_t install_state = rmt_driver_install(dshot_tx_rmt_config.channel, 0, 0);

    // Start serial ESC telemetry
    if (m_telem_enabled) {
        m_telem_uart->begin(115200, SERIAL_8N1, m_telem_rxpin, m_telem_txpin);
        
        // clear serial buffer
        while(m_telem_uart->available())
            m_telem_uart->read();
    }

    // Return result
    if (install_state == ESP_OK) {
        return true;
    }
    else {
        return false;
    }
}


void DShotRMT::send_throttle_raw(uint16_t throttle)
{
    if (!m_is_enabled)
        return;

    // 3D-Mode
    if (m_is_3D) {
        if (throttle == DSHOT_CMD_MOTOR_STOP)
            m_throttle_cmd = DSHOT_CMD_MOTOR_STOP;
        else
            m_throttle_cmd = constrain(throttle, DSHOT3D_THROTTLE_N_MIN, DSHOT3D_THROTTLE_R_MAX);
    }

    // Normal-Mode
    else {
        m_throttle_cmd = constrain(throttle, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
    }

    send_dshot();
}


void DShotRMT::send_throttle(float throttle)
{
    if (!m_is_enabled)
        return;

    m_throttle_des = throttle;

    float throttle_cmd;

    // 3D-Mode
    if (m_is_3D) {

        // positive direction
        if (throttle > 1e-6) {
            throttle_cmd = DSHOT3D_SLOPE * throttle + DSHOT3D_THROTTLE_N_MIN;
            throttle_cmd = constrain(throttle_cmd, DSHOT3D_THROTTLE_N_MIN, DSHOT3D_THROTTLE_N_MAX);
            m_throttle_cmd = static_cast<uint16_t>(std::round(throttle_cmd));
        }
        // negative direction
        else if (throttle < -1e-6) {
            throttle_cmd = DSHOT3D_SLOPE * abs(throttle) + DSHOT3D_THROTTLE_R_MIN;
            throttle_cmd = constrain(throttle_cmd, DSHOT3D_THROTTLE_R_MIN, DSHOT3D_THROTTLE_R_MAX);
            m_throttle_cmd = static_cast<uint16_t>(std::round(throttle_cmd));
        }
        else {
            m_throttle_cmd = 0;
        }
    }

    // Normal-Mode
    else {
        throttle_cmd = DSHOT_SLOPE * throttle + DSHOT_THROTTLE_MIN;
        throttle_cmd = constrain(throttle_cmd, DSHOT_THROTTLE_MIN, DSHOT_THROTTLE_MAX);
        m_throttle_cmd = static_cast<uint16_t>(std::round(throttle_cmd));
    }

    send_dshot();
}


// This method builds the RMT data transmission sequence for the DShot protocol
rmt_item32_t *DShotRMT::buildTxRmtItem(uint16_t parsed_packet)
{
    // Check if DShot is set to bidirectional mode
    if (dshot_config.is_bidirectional)
    {
        // If bidirectional, invert the high/low bits
        for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1)
        {
            if (parsed_packet & 0b1000000000000000)
            {
                // Set RMT item for a logic high signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_low;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_high;
            }
            else
            {
                // Set RMT item for a logic low signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_low;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_high;
            }

            // Set level of RMT item
            dshot_tx_rmt_item[i].level0 = 0;
            dshot_tx_rmt_item[i].level1 = 1;
        }
    }
    else
    {
        // If not bidirectional, set the RMT items as usual
        for (int i = 0; i < DSHOT_PAUSE_BIT; i++, parsed_packet <<= 1)
        {
            if (parsed_packet & 0b1000000000000000)
            {
                // Set RMT item for a logic high signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_one_high;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_one_low;
            }
            else
            {
                // Set RMT item for a logic low signal
                dshot_tx_rmt_item[i].duration0 = dshot_config.ticks_zero_high;
                dshot_tx_rmt_item[i].duration1 = dshot_config.ticks_zero_low;
            }

            // Set level of RMT item
            dshot_tx_rmt_item[i].level0 = 1;
            dshot_tx_rmt_item[i].level1 = 0;
        }
    }

    // Set end marker for each frame
    if (dshot_config.is_bidirectional)
    {
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 1;
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 0;
    }
    else
    {
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level0 = 0;
        dshot_tx_rmt_item[DSHOT_PAUSE_BIT].level1 = 1;
    }

    // Add packet seperator aka DShot Pause.
    dshot_tx_rmt_item[DSHOT_PAUSE_BIT].duration1 = DSHOT_PAUSE;

    // Return the rmt_item
    return dshot_tx_rmt_item;
}

// Calculates a CRC value for a DShot digital control signal packet
uint16_t DShotRMT::calculateCRC(const dshot_packet_t &dshot_packet)
{
    uint16_t crc;

    // Combine the throttle value and telemetric request flag into a 16-bit packet value
    const uint16_t packet = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;

    // Calculate the CRC value using different bitwise operations depending on the DShot configuration
    if (dshot_config.is_bidirectional)
    {
        // Bidirectional configuration: perform a bitwise negation of the result of XORing the packet with its right-shifted values by 4 and 8 bits,
        // and then bitwise AND the result with 0x0F
        const uint16_t intermediate_result = packet ^ (packet >> 4) ^ (packet >> 8);
        crc = (~intermediate_result) & 0x0F;
    }
    else
    {
        // Unidirectional configuration: XOR the packet with its right-shifted values by 4 and 8 bits,
        // and then bitwise AND the result with 0x0F
        crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    }

    // Return the calculated CRC value as a 16-bit unsigned integer
    return crc;
}

uint16_t DShotRMT::parseRmtPaket(const dshot_packet_t &dshot_packet)
{
    uint16_t parsedRmtPaket = DSHOT_NULL_PACKET;
    uint16_t crc = calculateCRC(dshot_packet);

    // Complete the paket
    parsedRmtPaket = (dshot_packet.throttle_value << 1) | dshot_packet.telemetric_request;
    parsedRmtPaket = (parsedRmtPaket << 4) | crc;

    return parsedRmtPaket;
}

// Output using ESP32 RMT
void DShotRMT::sendRmtPaket(const dshot_packet_t &dshot_packet)
{
    buildTxRmtItem(parseRmtPaket(dshot_packet));

    rmt_write_items(dshot_tx_rmt_config.channel, dshot_tx_rmt_item, DSHOT_PACKET_LENGTH, false);
}


int8_t DShotRMT::is_tx_finished()
{
    esp_err_t tx_state = rmt_wait_tx_done(dshot_tx_rmt_config.channel, 0);
    
    // Return result
    switch (tx_state) {
        case ESP_OK:
            return 1;
        case ESP_ERR_TIMEOUT:
            return 0;
        default:
            return -1;
    }
}


bool DShotRMT::set_3D_mode(bool activate)
{
    // This does not fully work. I don't know why, but we need to send cmd 0 (motor stop) so long that the ESC is armed and after that we can configure. I don't know how the BLHeliSuite does it, because the ESC does not beep while using the Suite, but if we arm it before, it beeps a lot!

/*
    // mode is already in desired mode
    if (activate == m_is_3D_mode)
        return true;

    // only allow 3D-Mode change when motors have zero throttle
    if (abs(m_throttle_des) > 1e-6)
        return false;
*/

    // enable / disable 3D-Mode
    dshot_packet_t dshot_rmt_packet;
    if (activate) {
        //dshot_rmt_packet = generate_dshot_packet(DSHOT_CMD_3D_MODE_ON, ENABLE_TELEMETRIC);
        dshot_rmt_packet = generate_dshot_packet(DSHOT_CMD_SPIN_DIRECTION_2, ENABLE_TELEMETRIC);
        m_is_3D = true;
    }
    else if (!activate) {
        dshot_rmt_packet = generate_dshot_packet(DSHOT_CMD_3D_MODE_OFF, ENABLE_TELEMETRIC);
        m_is_3D = false;
    };

    // send dshot command 10x, sendRmtPaket (in detail: rmt_write_items()) will block if the previous package is not fully send, so this loop blocks!
    // I put the delay blocks here based on the source code of betaflight: https://github.com/betaflight/betaflight/blob/master/src/main/drivers/dshot_command.c
    // Before that, the configuration did not succeed everytime
    delayMicroseconds(DSHOT_INITIAL_DELAY_US - DSHOT_COMMAND_DELAY_US);
    for (int idx = 0; idx < 10; ++idx) {
        delayMicroseconds(DSHOT_COMMAND_DELAY_US);
        sendRmtPaket(dshot_rmt_packet);
    }
    delayMicroseconds(DSHOT_COMMAND_DELAY_US);

    
    // save settings
    dshot_rmt_packet = generate_dshot_packet(DSHOT_CMD_SAVE_SETTINGS, ENABLE_TELEMETRIC);

    // send dshot command 10x, sendRmtPaket (in detail: rmt_write_items()) will block if the previous package is not fully send, so this loop blocks!
    // millis() will definitely block, not such a nice solution here!
    delayMicroseconds(DSHOT_INITIAL_DELAY_US - DSHOT_COMMAND_DELAY_US);
    for (int idx = 0; idx < 10; ++idx) {
        sendRmtPaket(dshot_rmt_packet);
        delay(40);
    }
    delayMicroseconds(DSHOT_COMMAND_DELAY_US);

    return true;

}


void DShotRMT::send_cmd(uint16_t cmd, bool telem)
{
    telemetric_request_t tel;
    if (telem)
        auto tel = ENABLE_TELEMETRIC;
    else
        auto tel = NO_TELEMETRIC;

    auto dshot_rmt_packet = generate_dshot_packet(cmd, tel);
    sendRmtPaket(dshot_rmt_packet);
}


dshot_packet_t DShotRMT::generate_dshot_packet(uint16_t cmd, telemetric_request_t telem)
{
    dshot_packet_t dshot_rmt_packet = {};

    dshot_rmt_packet.throttle_value = cmd;
    dshot_rmt_packet.telemetric_request = telem;

    dshot_rmt_packet.checksum = calculateCRC(dshot_rmt_packet);

    return dshot_rmt_packet;
}


dshot_packet_t DShotRMT::generate_dshot_packet(uint16_t cmd)
{
    return generate_dshot_packet(cmd, NO_TELEMETRIC);
}


void DShotRMT::send_dshot(void){

    auto telem_request = NO_TELEMETRIC;

    if (m_telem_enabled) {
        receive_telemetry();

        // Only send new request, if the request before is fully received...
        // Problem: If we have data loss, this does not work
        // Reset routine:
        /*
        if (m_telem_requested && ((micros() + dshot_frame_us[dshot_config.mode]) - m_telem_timestamp_us) > 1200)
        {
            Serial.println("reset");
            m_telem_requested = false;
            // also empty serial buffer...
            m_received_bytes = 0;
            while(m_telem_uart->available())
                m_telem_uart->read();
        }
        */

        if (!m_telem_requested) {
            m_telem_requested = true;
            m_telem_timestamp_us = micros() + dshot_frame_us[dshot_config.mode]; // This is only a guess, I don't know if the telemetry package the ESC sends back is from the time of successfull dshot frame receive.
            //m_telem_timestamp_us = micros();
            telem_request = ENABLE_TELEMETRIC;
        }
    }
    auto dshot_rmt_packet = generate_dshot_packet(m_throttle_cmd, telem_request);

    sendRmtPaket(dshot_rmt_packet);
}


// Taken from: https://github.com/JyeSmith/dshot-esc-tester/blob/master/dshot-esc-tester.ino
void DShotRMT::receive_telemetry(void)
{
    if (!m_telem_requested)
        return;


    //Serial.println("trying");



    // Read buffered data
    while (m_telem_uart->available() && m_received_bytes <= 9){
        m_serial_buffer[m_received_bytes++] = m_telem_uart->read();
        //Serial.println("rding");
        /*
        if (m_received_bytes > 9)
        {
            break;
        }
        */
    }
    

    if(m_received_bytes > 9){ // transmission complete

        //Serial.println("Data read!");

        m_telem_requested = false;

        m_received_bytes = 0;
        
        uint8_t crc8 = get_crc8(m_serial_buffer, 9); // get the 8 bit CRC
        
        if(crc8 != m_serial_buffer[9]) {
            // Transmission failure
            //Serial.println("fail!");
            
            // Empty serial buffer
            while(m_telem_uart->available())
                m_telem_uart->read();
        
            return;
        }
        
        // compute the received values
        m_telem_data.timestamp_us = m_telem_timestamp_us;
        m_telem_data.data.temperature = static_cast<float>(m_serial_buffer[0]);
        m_telem_data.data.voltage = ((m_serial_buffer[1]<<8)|m_serial_buffer[2]) / 100.0F;
        m_telem_data.data.current = ((m_serial_buffer[3]<<8)|m_serial_buffer[4]) / 10.0F;
        m_telem_data.data.consumption = ((m_serial_buffer[5]<<8)|m_serial_buffer[6]) / 100.0F;
        m_telem_data.data.RPM = ((m_serial_buffer[7]<<8)|m_serial_buffer[8]) * 100.0F / (m_motor_poles / 2);

        return;
    }


    // ESC telemetry timeout handling
    if (m_telem_requested && ((micros() + dshot_frame_us[dshot_config.mode]) - m_telem_timestamp_us) > telemetry_frame_us)
    {
        //Serial.println("reset");

        m_telem_requested = false;
        
        // Reset telemetry readout
        m_received_bytes = 0;
        while(m_telem_uart->available())
            m_telem_uart->read();
    }
}


// Taken from: https://github.com/JyeSmith/dshot-esc-tester/blob/master/dshot-esc-tester.ino
// ...but Function is from here: https://www.rcgroups.com/forums/showatt.php?attachmentid=8524039&d=1450424877
uint8_t DShotRMT::get_crc8(uint8_t *Buf, uint8_t BufLen) {
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}


// Taken from: https://github.com/JyeSmith/dshot-esc-tester/blob/master/dshot-esc-tester.ino
// ...but function is from here: https://www.rcgroups.com/forums/showatt.php?attachmentid=8524039&d=1450424877
uint8_t DShotRMT::update_crc8(uint8_t crc, uint8_t crc_seed) {
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}



void DShotRMT::setup(bool is_3D, uint8_t motor_poles) {
    m_is_3D = is_3D;
    m_motor_poles = motor_poles;
}


ESCTelemetryPackage_t DShotRMT::get_telemetry(void) {

    if (!m_is_enabled)
        return ESCTelemetryPackage_t{ };

    receive_telemetry();

    // If motor is stopping or stopped, we could get garbage RPM, so we override
    // This problem seems only to occur, if damped mode (breaking) is on. If damped mode is of we don't need this
    // ToDo: Check ESC-Settings at ESC-Begin and set internals accordingly
    /*
    if (abs(m_throttle_des) < 1e-6)
    {
        auto telem_data = m_telem_data;
        telem_data.data.RPM = 0.0F;

        return telem_data;
    }
    */
    
    return m_telem_data;
}

void DShotRMT::enable(void) {
    if (!m_is_enabled)
        m_is_enabled = true;
}

void DShotRMT::disable(void) {
    if (m_is_enabled) {
        send_throttle(0.0F);
        m_is_enabled = false;
    }
}