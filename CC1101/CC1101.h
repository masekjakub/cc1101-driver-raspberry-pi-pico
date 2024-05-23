
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
#include <string.h>
#include <vector>
#include <iostream>
#include "pico/stdlib.h"
#include "hardware/spi.h"

/*__________________________________________________________________________________
|                                                                                   |
|                              PACKET STRUCTURE                                     |
| __________________________________________________________________________________|
|        |                                        |          |                      |
|        |                 HEADER                 |   DATA   |        STATUS        |
|________|________________________________________|__________|______________________|
|   1B   |     1B       |     1B      |    1B     |    xB    |   1B   |     1B      |
| LENGTH | DEST ADDRESS | SRC ADDRESS |    ACK    |    ...   |  RSSI  | LQI+CRC(1b) |
|________|______________|_____________|___________|__________|________|____________*/

const int MAX_PACKET_LEN = 58;
const int PA_TABLE_SIZE = 8;

/*-----------------------------[CC1101 - presets]----------------------------*/
enum CC1101_Preset
{
    GFSK_4_8_kb,
    ASK_OOK_4_8_kb,
    GFSK_38_4_kb,
    GFSK_100_kb,
    MSK_500_kb
};
/*------------------------------[END modulation]-----------------------------*/

/*----------------------------[CC1101 - ISM band]----------------------------*/
enum CC1101_ISM
{
    ISM_315,
    ISM_433,
    ISM_868,
    ISM_915
};
/*------------------------------[END ISM band]-------------------------------*/

struct Packet
{
    uint8_t src_address = 0;
    uint8_t data_length = 0;
    uint8_t data[MAX_PACKET_LEN] = {0};
    uint8_t ack_flag = 0;
    double rssi = 0.0;
    uint8_t lqi = 0;
    bool valid = false;
};

class CC1101
{
private:
    uint8_t ss_pin;
    uint8_t gdo0_pin;
    uint8_t gdo2_pin;
    spi_inst_t *spi;
    uint8_t my_addr;
    uint8_t ack_retries;
    uint8_t pa_table[PA_TABLE_SIZE];

    std::vector<Packet> rx_packet_buffer; // buffer for incoming packets not received in read_packet
    bool sending_now = false;             // flag indicating that the module is currently sending a packet

    uint8_t get_state();
    void flush_rx_fifo();
    double get_rssi_dbm(uint8_t rssi_dec);

    void spi_write_reg(const uint8_t, const uint8_t);
    void spi_write_burst(const uint8_t, const uint8_t *, const size_t);
    uint8_t spi_read_reg(const uint8_t);
    void spi_read_burst(const uint8_t, uint8_t *, const size_t);
    uint8_t spi_write_strobe(const uint8_t);

public:
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
     * @brief Sets the power level of the CC1101 module. Default is 5.
     *
     * This function allows you to set the power level of the CC1101 module.
     * The power level is index in the power table array.
     *
     * @param power The power level to set (0-7).
     */
    void set_power(uint8_t);

    /**
     * @brief Sets the address filtering mode for the CC1101 module. Default is 3.
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
     * @brief Get the address of the device.
     *
     * This function returns the address of the device.
     *
     * @return The address of the device.
     */
    uint8_t get_my_addr() { return my_addr; }

    /**
     * @brief Set the number of times the module should try to resend the packet if no acknowledgment is received. Default is 3.
     *
     * @param ack_retries The number of retries to set.
     */
    void set_ack_retries(uint8_t ack_retries) { this->ack_retries = ack_retries; }
};
#endif