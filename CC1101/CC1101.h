
/**
 * @file CC1101.h
 * @brief Header file for the CC1101 module.
 * @author Jakub Mašek (https://github.com/masekjakub)
 *
 * This file contains the declaration of the CC1101 class, which provides
 * functions to interact with the CC1101 module.
 */

#ifndef CC1101_H
#define CC1101_H

#include <stdint.h>
#include <stdio.h>
#include <cstring>
#include <iostream>
#include <bitset>
#include "pico/stdlib.h"
#include "hardware/spi.h"

/*__________________________________________________________________________________
|                                                                                   |
|                              PACKET STRUCTURE                                     |
| __________________________________________________________________________________|
|                                                 |          |                      |
|                     HEADER                      |   DATA   |        STATUS        |
|_________________________________________________|__________|______________________|
|   1B   |     1B       |     1B      |    1B     |    xB    |   1B   |     1B      |
| LENGTH | DEST ADDRESS | SRC ADDRESS |    ACK    |    ...   |  RSSI  | LQI+CRC(1b) |
|________|______________|_____________|___________|__________|________|____________*/

#define curMillis() to_ms_since_boot(get_absolute_time())

/*----------------------[CC1101 - constants]----------------------------*/
#define HEADER_LEN 4
#define MAX_PACKET_LEN 58
#define FIFO_SIZE 64
/*-------------------------[END constants]-------------------------------*/

/*----------------------[CC1101 - status byte]----------------------------*/
#define FIFO_BYTES_AVAILABLE 0x0F
#define STATE_IDLE 0x00
#define STATE_RX 0x01
#define STATE_TX 0x02
#define STATE_FSTXON 0x03
#define STATE_CALIBRATE 0x04
#define STATE_SETTLING 0x05
#define STATE_RXFIFO_OVERFLOW 0x06
#define STATE_TXFIFO_UNDERFLOW 0x07
/*-------------------------[END status byte]-------------------------------*/

/*--------------------------[CC1101 - ACK]--------------------------------*/
#define ACK_DISABLE 0x00
#define ACK_ENABLE 0x01
#define ACK_OK 0x02
#define ACK_FAIL 0x03
/*-------------------------[END ACK]-----------------------------------*/

/*------------------------[CC1101 - R/W addresses]----------------------------*/
#define WRITE_BYTE 0x00
#define WRITE_BURST 0x40
#define READ_BYTE 0x80
#define READ_BURST 0xC0

#define RX_FIFO 0x3F
#define TX_FIFO 0x3F
#define TX_FIFO_BURST 0x7F
#define RX_FIFO_BURST 0xFF

#define PATABLE_BYTE 0xFE
#define PATABLE_BURST 0x7E
/*-------------------------[END R/W addresses]-------------------------------*/

/*------------------------[CC1101 - presets]----------------------------*/
enum CC1101_Preset
{
    ASK_OOK_4_8_kb,
    GFSK_38_4_kb,
    GFSK_100_kb,
    MSK_500_kb
};
/*-------------------------[END modulation]-------------------------------*/

/*------------------------[CC1101 - command strobes]----------------------------*/
#define SRES 0x30    // Reset chip
#define SFSTXON 0x31 // Enable/calibrate frequency synthesizer
#define SXOFF 0x32   // Turn off crystal oscillator
#define SCAL 0x33    // Calibrate frequency synthesizer and turn it off
#define SRX 0x34     // Enable RX
#define STX 0x35     // Enable TX
#define SIDLE 0x36   // Exit RX / TX,  turn off frequency synthesizer
#define SWOR 0x38    // Start automatic RX polling sequence
#define SPWD 0x39    // Enter power down mode when CSn goes high
#define SFRX 0x3A    // Flush the RX FIFO buffer
#define SFTX 0x3B    // Flush the TX FIFO buffer
#define SWORRST 0x3C // Reset real time clock to Event1 value
#define SNOP 0x3D    // No operation (May be used to get access to the chip status byte)
/*-------------------------[END command strobes]------------------------------*/

/*----------------------[CC1101 - configuiration registers]----------------------------*/
#define IOCFG2 0x00   // GDO2 output pin configuration
#define IOCFG1 0x01   // GDO1 output pin configuration
#define IOCFG0 0x02   // GDO0 output pin configuration
#define FIFOTHR 0x03  // RX FIFO and TX FIFO thresholds
#define SYNC1 0x04    // Sync word, high byte
#define SYNC0 0x05    // Sync word, low byte
#define PKTLEN 0x06   // Packet length
#define PKTCTRL1 0x07 // Packet automation control
#define PKTCTRL0 0x08 // Packet automation control
#define ADDR 0x09     // Device address
#define CHANNR 0x0A   // Channel number
#define FSCTRL1 0x0B  // Frequency synthesizer control
#define FSCTRL0 0x0C  // Frequency synthesizer control
#define FREQ2 0x0D    // Frequency control word, high byte
#define FREQ1 0x0E    // Frequency control word, middle byte
#define FREQ0 0x0F    // Frequency control word, low byte
#define MDMCFG4 0x10  // Modem configuration
#define MDMCFG3 0x11  // Modem configuration
#define MDMCFG2 0x12  // Modem configuration
#define MDMCFG1 0x13  // Modem configuration
#define MDMCFG0 0x14  // Modem configuration
#define DEVIATN 0x15  // Modem deviation setting
#define MCSM2 0x16    // Main Radio Cntrl State Machine config
#define MCSM1 0x17    // Main Radio Cntrl State Machine config
#define MCSM0 0x18    // Main Radio Cntrl State Machine config
#define FOCCFG 0x19   // Frequency Offset Compensation config
#define BSCFG 0x1A    // Bit Synchronization configuration
#define AGCCTRL2 0x1B // AGC control
#define AGCCTRL1 0x1C // AGC control
#define AGCCTRL0 0x1D // AGC control
#define WOREVT1 0x1E  // High byte Event 0 timeout
#define WOREVT0 0x1F  // Low byte Event 0 timeout
#define WORCTRL 0x20  // Wake On Radio control
#define FREND1 0x21   // Front end RX configuration
#define FREND0 0x22   // Front end TX configuration
#define FSCAL3 0x23   // Frequency synthesizer calibration
#define FSCAL2 0x24   // Frequency synthesizer calibration
#define FSCAL1 0x25   // Frequency synthesizer calibration
#define FSCAL0 0x26   // Frequency synthesizer calibration
#define RCCTRL1 0x27  // RC oscillator configuration
#define RCCTRL0 0x28  // RC oscillator configuration
#define FSTEST 0x29   // Frequency synthesizer cal control
#define PTEST 0x2A    // Production test
#define AGCTEST 0x2B  // AGC test
#define TEST2 0x2C    // Various test settings
#define TEST1 0x2D    // Various test settings
#define TEST0 0x2E    // Various test settings
/*-------------------------[END configuiration registers]------------------------------*/

/*----------------------[CC1101 - status register]----------------------------*/
#define PARTNUM 0xF0    // Part number
#define VERSION 0xF1    // Current version number
#define FREQEST 0xF2    // Frequency offset estimate
#define LQI 0xF3        // Demodulator estimate for link quality
#define RSSI 0xF4       // Received signal strength indication
#define MARCSTATE 0xF5  // Control state machine state
#define WORTIME1 0xF6   // High byte of WOR timer
#define WORTIME0 0xF7   // Low byte of WOR timer
#define PKTSTATUS 0xF8  // Current GDOx status and packet status
#define VCO_VC_DAC 0xF9 // Current setting from PLL cal module
#define TXBYTES 0xFA    // Underflow and # of bytes in TXFIFO
#define RXBYTES 0xFB    // Overflow and # of bytes in RXFIFO
#define BYTES_IN_FIFO_MASK 0x7F
#define OVERFLOW_FIFO_MASK 0x80
#define RCCTRL1_STATUS 0xFC // Last RC Oscillator Calibration Result
#define RCCTRL0_STATUS 0xFD // Last RC Oscillator Calibration Result
/*--------------------------[END status register]-------------------------------*/

/*----------------------[CC1101 - PA table]----------------------------*/
#define PA_TABLE_SIZE 8
static uint8_t PA_table_315[PA_TABLE_SIZE] = {0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2};
static uint8_t PA_table_433[PA_TABLE_SIZE] = {0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0};
static uint8_t PA_table_868[PA_TABLE_SIZE] = {0x03, 0x0F, 0x1E, 0x27, 0x50, 0x81, 0xCB, 0xC2};
static uint8_t PA_table_915[PA_TABLE_SIZE] = {0x03, 0x0E, 0x1E, 0x27, 0x8E, 0xCD, 0xC7, 0xC0};
/*-------------------------[END PA table]-------------------------------*/

/*----------------------[CC1101 - ISM band]----------------------------*/
#define ISM_315 0
#define ISM_433 1
#define ISM_868 2
#define ISM_915 3
/*-------------------------[END ISM band]-------------------------------*/

class CC1101
{

private:
    uint8_t ss_pin;
    uint8_t gdo0_pin;
    uint8_t gdo2_pin;
    spi_inst_t *spi;
    uint8_t my_addr;
    uint16_t ack_timeout_ms = 250;
    uint8_t ack_retries = 4;
    uint8_t pa_table[8];

    void spi_write_reg(uint8_t, uint8_t);
    void spi_write_burst(uint8_t, uint8_t *, size_t);
    uint8_t spi_read_reg(uint8_t);
    void spi_read_burst(uint8_t, uint8_t *, size_t);
    uint8_t spi_write_strobe(uint8_t);
    void flush_rx_fifo();

    double get_rssi_dbm(uint8_t rssi_dec);

public:
    // NOTE PATABLE lost in sleep mode
    // TODO set_datarate, wor, get rssi and lqi + crc,
    struct Packet
    {
        uint8_t src_address = 0;
        uint8_t data_length = 0;
        uint8_t data[FIFO_SIZE - HEADER_LEN] = {0};
        uint8_t ack_flag = 0;
        double rssi = 0.0;
        uint8_t lqi = 0;
        bool crc_ok = false;
    };

    CC1101(spi_inst_t *spi, uint8_t ss_pin, uint8_t gdo0_pin, uint8_t gdo2_pin = -1)
    {
        this->spi = spi;
        this->ss_pin = ss_pin;
        this->gdo0_pin = gdo0_pin;
        this->gdo2_pin = gdo2_pin;
    }

    /**
     * Initializes the CC1101 module with the specified address.
     *
     * @param my_addr The address to be assigned to the CC1101 module.
     * @return True if initialization is successful, false otherwise.
     * @note Default settings are set (freq: 868MHz, power: 2, address_filtering: 3)
     */
    bool begin(uint8_t my_addr);

    /**
     * @brief Resets the CC1101 module.
     *
     * This function resets the CC1101 module to its default state.
     */
    void reset();

    /**
     * @brief Sets the preset value for the CC1101 module.
     *
     * This function sets the preset value for the CC1101 module. The preset value determines the configuration settings for the module.
     *
     * @param preset The preset value to set (ASK_OOK_4_8_kb, GFSK_38_4_kb, GFSK_100_kb, MSK_500_kb).
     */
    void set_preset(CC1101_Preset);

    /**
     * @brief Sets the frequency of the CC1101 module.
     *
     * This function sets the frequency of the CC1101 module to the specified value.
     *
     * @param freq The frequency value to set (ISM_315, ISM_433, ISM_868, ISM_915).
     */
    void set_freq(uint8_t);

    /**
     * @brief Sets the channel of the CC1101 transceiver.
     *
     * This function sets the channel of the CC1101 transceiver to the specified value.
     *
     * @param channel The channel number to set.
     */
    void set_channel(uint8_t);

    /**
     * @brief Sets the address of the CC1101 module.
     *
     * This function sets the address of the CC1101 module. The address is used for communication
     * between multiple CC1101 modules in a network.
     *
     * @param address The address to be set for the CC1101 module.
     */
    void set_address(uint8_t);

    /**
     * @brief Sets the power table for the CC1101 module.
     *
     * This function allows you to set the power table for the CC1101 module.
     * The power table is an 8 bytes long array that represent the power levels
     * for different output power settings of the module. Check the datasheet for more information.
     *
     * @param power_table A pointer to the power array.
     */
    void set_power_table(uint8_t *);

    /**
     * @brief Sets the power level of the CC1101 module.
     *
     * This function allows you to set the power level of the CC1101 module.
     * The power level is index in the power table array.
     *
     * @param power The power level to set (0-7).
     */
    void set_power(uint8_t);

    /**
     * @brief Sets the address filtering mode for the CC1101 module.
     *
     * Determines how the CC1101 module should filter incoming packets based on the address.
     *
     * @param addressFiltering The address filtering mode to be set.
     * @note Modes: 0 - Disabled, 1 - Address check, 2 - Address check and 0x00 as broadcast, 3 - Address check and 0x00 and 0xFF as broadcast.
     */
    void set_address_filtering(uint8_t);

    /**
     * @brief Puts the CC1101 into idle mode.
     *
     * In idle mode, the CC1101 is still powered on, but not actively transmitting or receiving data.
     */
    void idle();

    /**
     * @brief Sets CC1101 module to receive mode.
     *
     */
    void receive();

    /**
     * @brief Sets the CC1101 module to transmit mode.
     *
     */
    void transmit();

    /**
     * Checks if a packet is available to be read.
     *
     * @return true if a packet is available, false otherwise.
     */
    bool packet_available();

    /**
     * Sends a packet over the CC1101 module.
     *
     * @param address The address to send the packet to.
     * @param data Pointer to the data buffer containing the packet payload.
     * @param len The length of the packet payload.
     * @param use_ack Flag indicating whether to use acknowledgment for the packet (default: true).
     * @return True if the packet was sent (and received when ACK is enabled) successfully, false otherwise.
     * @note The maximum length of the packet payload is 58 bytes.
     * @note The function will block until the packet is sucessfully sent or the acknowledgment timeout is reached.
     */
    bool send_packet(uint8_t address, uint8_t *data, uint8_t len, bool use_ack = true);

    /**
     * Reads an incoming packet from the CC1101 module.
     *
     * @return The received packet. When invalid or no packet is received, the packet's variables are set to 0.
     */
    Packet read_packet();

    /**
     * @brief Get the state of the CC1101 module.
     *
     * This function returns the current state of the CC1101 module.
     *
     * @return The state of the CC1101 module as a uint8_t value. The possible states are defined with prefix STATE_.
     *
     */
    uint8_t get_state();

    /**
     * @brief Get the address of the device.
     *
     * This function returns the address of the device.
     *
     * @return The address of the device.
     */
    uint8_t get_my_addr() { return my_addr; }
};

#endif