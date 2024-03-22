/**
 * @file CC1101.cpp
 * @brief Implementation of the CC1101 driver.
 * @author Jakub Mašek (https://github.com/masekjakub)
 *
 * This file contains the implementation of the CC1101 class, which provides
 * functions for controlling the CC1101 RF transceiver module.
 */
#include "CC1101.h"

// #define DEBUG

void CC1101::reset()
{
    gpio_put(this->ss_pin, 1);
    sleep_us(50);
    gpio_put(this->ss_pin, 0);
    sleep_us(50);
    gpio_put(this->ss_pin, 1);
    sleep_us(40);

    spi_write_strobe(SRES);
    gpio_put(this->ss_pin, 1);
    sleep_us(10);
}

bool CC1101::begin(uint8_t my_addr)
{

    if (my_addr >= 255 || my_addr < 0)
    {
        printf("Invalid address. Can be between 1 and 254.\n");
        return 0;
    }

    gpio_init(this->ss_pin);
    gpio_init(this->gdo0_pin);
    if (this->gdo2_pin != -1)
        gpio_init(this->gdo2_pin);

    gpio_set_dir(this->ss_pin, GPIO_OUT);
    gpio_set_dir(this->gdo0_pin, GPIO_IN);
    if (this->gdo2_pin != -1)
        gpio_set_dir(this->gdo2_pin, GPIO_IN);

    sleep_ms(10);

    this->my_addr = my_addr;

    int speed;
    if ((speed = spi_init(this->spi, 8000000)) <= 0)
    {
        printf("SPI failed to initialize\n");
        return 0;
    }

#ifdef DEBUG
    printf("SPI initialized at %dHz\n", speed);
#endif

    reset();

    idle();

    spi_write_reg(IOCFG0, 0x01); // assert GDO0 when RXFIFO is filled above treshold
    spi_write_reg(MCSM1, 0x3C);

    set_freq(ISM_868);
    set_channel(0);
    set_power(0);
    set_address(my_addr);
    set_address_filtering(3);

    return 1;
}

uint8_t CC1101::get_state()
{
    uint8_t last_status = spi_write_strobe(SNOP);

    while (spi_write_strobe(SNOP) != last_status)
    {
        sleep_us(1);
        last_status = spi_write_strobe(SNOP);
    }
    return (spi_write_strobe(SNOP) >> 4) & 0b0111;
}

void CC1101::idle()
{
    spi_write_strobe(SIDLE);
    while (get_state() != STATE_IDLE)
    {
        sleep_us(1);
    }
}

void CC1101::receive()
{
    idle();
    spi_write_strobe(SRX);
    while (get_state() != STATE_RX)
    {
        sleep_us(1);
    }
}

void CC1101::transmit()
{
    idle();
    spi_write_strobe(STX);
    while (get_state() != STATE_TX)
    {
        sleep_us(1);
    }
}

bool CC1101::packet_available()
{
    if (gpio_get(gdo0_pin) == 1)
    {
        return true;
    }
    return false;
}

double CC1101::get_rssi_dbm(uint8_t rssi_dec)
{
    if (rssi_dec >= 128)
        return (rssi_dec - 256) / 2.0 - 74.0;
    else
        return rssi_dec / 2.0 - 74.0;
}

CC1101::Packet CC1101::read_packet()
{
    Packet packet;
    uint8_t rx_bytes = spi_read_reg(RXBYTES);
#ifdef DEBUG
    printf("RXBYTES: %d\n", rx_bytes);
#endif
    if (rx_bytes & OVERFLOW_FIFO_MASK || rx_bytes & BYTES_IN_FIFO_MASK < HEADER_LEN)
    {
        idle();
        spi_write_strobe(SFRX);
        receive();
#ifdef DEBUG
        printf("Overflow or invalid packet\n");
#endif
        return packet;
    }

    uint8_t packet_length = spi_read_reg(RX_FIFO);
    if (packet_length < 3 || packet_length > MAX_PACKET_LEN + HEADER_LEN - 1)
    {
        idle();
        spi_write_strobe(SFRX);
        receive();
#ifdef DEBUG
        printf("Invalid packet length\n");
#endif
        return packet;
    }

    uint8_t status[2];
    uint8_t data[packet_length];
    spi_read_burst(RX_FIFO_BURST, data, packet_length);
    spi_read_burst(RX_FIFO_BURST, status, 2);

    packet.src_address = data[1];
    packet.data_length = packet_length - HEADER_LEN + 1;
    memcpy(packet.data, &(data[HEADER_LEN - 1]), packet.data_length);

    packet.ack_flag = data[2];
    packet.rssi = get_rssi_dbm(status[0]);
    packet.lqi = status[1] & 0x7F;
    packet.crc_ok = status[1] & 0x80;

    if (data[2] == ACK_ENABLE)
    {
        uint8_t ack_packet[4] = {3, packet.src_address, this->my_addr};
        ack_packet[3] = (packet.crc_ok == 1 ? ACK_OK : ACK_FAIL);
        spi_write_burst(TX_FIFO_BURST, ack_packet, 4);
        transmit();

        while (get_state() != STATE_IDLE)
        {
            sleep_us(1);
        }
        receive();
    }
    return packet;
}

bool CC1101::send_packet(uint8_t address, uint8_t *data, uint8_t len, bool use_ack)
{
    if (len > MAX_PACKET_LEN)
    {
        printf("Packet length must be less than 59 bytes\n");
        return 0;
    }

    auto tx_bytes = spi_read_reg(TXBYTES);
    if (tx_bytes)
    {
        idle();
        spi_write_strobe(SFTX);
    }

    len += HEADER_LEN;

    uint8_t tx_buffer[len];
    tx_buffer[0] = len - 1; // length of payload (excluding length byte)
    tx_buffer[1] = address;
    tx_buffer[2] = this->my_addr;
    tx_buffer[3] = use_ack;

    for (int i = HEADER_LEN; i < len; i++)
    {
        tx_buffer[i] = data[i - HEADER_LEN];
    }

#ifdef DEBUG
    printf("Sending packet: ");
    for (int i = 0; i < len; i++)
    {
        printf("%02X ", tx_buffer[i]);
    }
    printf("\n");
#endif

    bool ack_ok = 0;
    uint8_t retries = ack_retries;
    do
    {
#ifdef DEBUG
        printf("RETRIES: %d\n", retries);
#endif
        spi_write_burst(TX_FIFO_BURST, tx_buffer, len);
        transmit();

        while (get_state() != STATE_IDLE)
        {
            sleep_us(1);
        }

        receive();

        if (use_ack)
        {
            uint32_t time_sent = curMillis();
            while (curMillis() - time_sent < ack_timeout_ms)
            {
                if (packet_available())
                {
                    Packet packet = read_packet();
                    if (packet.ack_flag == ACK_OK)
                    {
#ifdef DEBUG
                        printf("ACK OK\n");
#endif
                        ack_ok = 1;
                        break;
                    }
                    if (packet.ack_flag == ACK_FAIL)
                    {
#ifdef DEBUG
                        printf("ACK FAIL\n");
#endif
                        ack_ok = 0;
                        break;
                    }
                }
                sleep_us(500);
            }
            retries--;
        }
        else
        {
            ack_ok = 1;
        }
    } while (!ack_ok && retries);
    return ack_ok;
}

void CC1101::set_preset(uint8_t mode)
{
    switch (mode)
    {
    case ASK_OOK_4_8_kb:
        spi_write_reg(FSCTRL1, 0x06);
        spi_write_reg(FSCTRL0, 0x00);
        spi_write_reg(MDMCFG4, 0xC7);
        spi_write_reg(MDMCFG3, 0x83);
        spi_write_reg(MDMCFG2, 0x12);
        spi_write_reg(MDMCFG1, 0x22);
        spi_write_reg(MDMCFG0, 0xF8);
        spi_write_reg(DEVIATN, 0x34);
        spi_write_reg(FREND1, 0x56);
        spi_write_reg(FREND0, 0x10);
        spi_write_reg(MCSM0, 0x18);
        spi_write_reg(FOCCFG, 0x16);
        spi_write_reg(BSCFG, 0x6C);
        spi_write_reg(AGCCTRL2, 0x03);
        spi_write_reg(AGCCTRL1, 0x00);
        spi_write_reg(AGCCTRL0, 0x92);
        spi_write_reg(FSCAL3, 0xE9);
        spi_write_reg(FSCAL2, 0x2A);
        spi_write_reg(FSCAL1, 0x00);
        spi_write_reg(FSCAL0, 0x1F);
        spi_write_reg(FSTEST, 0x59);
        spi_write_reg(TEST2, 0x81);
        spi_write_reg(TEST1, 0x35);
        spi_write_reg(TEST0, 0x09);
        spi_write_reg(FIFOTHR, 0x47);

        break;
    case GFSK_38_4_kb:
        spi_write_reg(FSCTRL1, 0x06);
        spi_write_reg(FSCTRL0, 0x00);
        spi_write_reg(MDMCFG4, 0xCA);
        spi_write_reg(MDMCFG3, 0x83);
        spi_write_reg(MDMCFG2, 0x12);
        spi_write_reg(MDMCFG1, 0x22);
        spi_write_reg(MDMCFG0, 0xF8);
        spi_write_reg(DEVIATN, 0x34);
        spi_write_reg(FREND1, 0x56);
        spi_write_reg(FREND0, 0x10);
        spi_write_reg(MCSM0, 0x18);
        spi_write_reg(FOCCFG, 0x16);
        spi_write_reg(BSCFG, 0x6C);
        spi_write_reg(AGCCTRL2, 0x43);
        spi_write_reg(AGCCTRL1, 0x40);
        spi_write_reg(AGCCTRL0, 0x91);
        spi_write_reg(FSCAL3, 0xE9);
        spi_write_reg(FSCAL2, 0x2A);
        spi_write_reg(FSCAL1, 0x00);
        spi_write_reg(FSCAL0, 0x1F);
        spi_write_reg(FSTEST, 0x59);
        spi_write_reg(TEST2, 0x81);
        spi_write_reg(TEST1, 0x35);
        spi_write_reg(TEST0, 0x09);
        spi_write_reg(FIFOTHR, 0x47);

        break;
    case GFSK_100_kb:
        spi_write_reg(FSCTRL1, 0x08);
        spi_write_reg(FSCTRL0, 0x00);
        spi_write_reg(MDMCFG4, 0x5B);
        spi_write_reg(MDMCFG3, 0xF8);
        spi_write_reg(MDMCFG2, 0x13);
        spi_write_reg(MDMCFG1, 0x22);
        spi_write_reg(MDMCFG0, 0xF8);
        spi_write_reg(DEVIATN, 0x47);
        spi_write_reg(FREND1, 0xB6);
        spi_write_reg(FREND0, 0x10);
        spi_write_reg(MCSM0, 0x18);
        spi_write_reg(FOCCFG, 0x1D);
        spi_write_reg(BSCFG, 0x1C);
        spi_write_reg(AGCCTRL2, 0xC7);
        spi_write_reg(AGCCTRL1, 0x00);
        spi_write_reg(AGCCTRL0, 0xB2);
        spi_write_reg(FSCAL3, 0xEA);
        spi_write_reg(FSCAL2, 0x2A);
        spi_write_reg(FSCAL1, 0x00);
        spi_write_reg(FSCAL0, 0x1F);
        spi_write_reg(FSTEST, 0x59);
        spi_write_reg(TEST2, 0x81);
        spi_write_reg(TEST1, 0x35);
        spi_write_reg(TEST0, 0x09);
        spi_write_reg(FIFOTHR, 0x47);

        break;

    case MSK_500_kb:
        spi_write_reg(FSCTRL1, 0x0E);
        spi_write_reg(FSCTRL0, 0x00);
        spi_write_reg(MDMCFG4, 0x0E);
        spi_write_reg(MDMCFG3, 0x3B);
        spi_write_reg(MDMCFG2, 0x73);
        spi_write_reg(MDMCFG1, 0xA0);
        spi_write_reg(MDMCFG0, 0xF8);
        spi_write_reg(DEVIATN, 0x00);
        spi_write_reg(FREND1, 0xB6);
        spi_write_reg(FREND0, 0x10);
        spi_write_reg(MCSM0, 0x18);
        spi_write_reg(FOCCFG, 0x1D);
        spi_write_reg(BSCFG, 0x1C);
        spi_write_reg(AGCCTRL2, 0xC7);
        spi_write_reg(AGCCTRL1, 0x00); // 0x00
        spi_write_reg(AGCCTRL0, 0xB0); // 0xB0
        spi_write_reg(FSCAL3, 0xEA);
        spi_write_reg(FSCAL2, 0x2A); // 0x2A
        spi_write_reg(FSCAL1, 0x00);
        spi_write_reg(FSCAL0, 0x1F); // 0x1F
        spi_write_reg(FSTEST, 0x59);
        spi_write_reg(TEST2, 0x88); // 0x88
        spi_write_reg(TEST1, 0x31); // 0x31
        spi_write_reg(TEST0, 0x09); // 0x09
        spi_write_reg(FIFOTHR, 0x07);

        break;
    default:
        printf("Invalid preset\n");
    }
}

void CC1101::set_freq(uint8_t band)
{
    uint8_t freq[3] = {0};
    switch (band)
    {
    case ISM_315:
        freq[2] = 0x0C;
        freq[1] = 0x1D;
        freq[0] = 0x89;
        spi_write_burst(PATABLE_BURST, PA_table_315, PA_TABLE_SIZE);
        break;
    case ISM_433:
        freq[2] = 0x10;
        freq[1] = 0xB0;
        freq[0] = 0x71;
        spi_write_burst(PATABLE_BURST, PA_table_433, PA_TABLE_SIZE);
        break;
    case ISM_868:
        freq[2] = 0x21;
        freq[1] = 0x62;
        freq[0] = 0x76;
        spi_write_burst(PATABLE_BURST, PA_table_868, PA_TABLE_SIZE);
        break;
    case ISM_915:
        freq[2] = 0x23;
        freq[1] = 0x31;
        freq[0] = 0x3B;
        spi_write_burst(PATABLE_BURST, PA_table_915, PA_TABLE_SIZE);
        break;
    default:
        printf("Invalid freq band\n");
    }

    spi_write_reg(FREQ2, freq[2]);
    spi_write_reg(FREQ1, freq[1]);
    spi_write_reg(FREQ0, freq[0]);
}

void CC1101::set_channel(uint8_t channel)
{
    spi_write_reg(CHANNR, channel);
}

void CC1101::set_address(uint8_t my_addr)
{
    spi_write_reg(ADDR, my_addr);
}

void CC1101::set_power_table(uint8_t *pa_table)
{
    spi_write_burst(PATABLE_BURST, pa_table, PA_TABLE_SIZE);
}

void CC1101::set_power(uint8_t power)
{
    if (power < 0 || power > 7)
    {
        printf("Invalid power level. Must be 0-7.\n");
        return;
    }
    spi_write_reg(FREND0, power);
}

void CC1101::set_address_filtering(uint8_t mode)
{
    if (mode < 0 || mode > 3)
    {
        printf("Invalid address filtering mode. Must be 0-3.\n");
        return;
    }
    spi_write_reg(PKTCTRL1, mode | 0x04);
}

void CC1101::spi_write_reg(uint8_t instruction, uint8_t data)
{
    uint8_t src[2] = {instruction | WRITE_BYTE, data};

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, src, 2);
    gpio_put(this->ss_pin, 1);
}

void CC1101::spi_write_burst(uint8_t instruction, uint8_t *data, size_t len)
{
    uint8_t src[len + 2] = {0};
    src[0] = instruction | WRITE_BURST;

    memcpy(&(src[1]), data, len);

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, src, len + 1);
    gpio_put(this->ss_pin, 1);
}

uint8_t CC1101::spi_read_reg(uint8_t instruction)
{
    uint8_t src = instruction | READ_BYTE;
    uint8_t dst;

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, &src, 1);
    spi_read_blocking(this->spi, src, &dst, 1);
    gpio_put(this->ss_pin, 1);
    return dst;
}

void CC1101::spi_read_burst(uint8_t instruction, uint8_t *data, size_t len)
{
    uint8_t src = instruction | READ_BURST;

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, &src, 1);
    spi_read_blocking(this->spi, src, data, len);
    gpio_put(this->ss_pin, 1);
}

uint8_t CC1101::spi_write_strobe(uint8_t strobe)
{
    gpio_put(this->ss_pin, 0);
    uint8_t status;
    spi_write_blocking(this->spi, &strobe, 1);
    spi_read_blocking(this->spi, strobe, &status, 1);
    gpio_put(this->ss_pin, 1);
    return status;
}