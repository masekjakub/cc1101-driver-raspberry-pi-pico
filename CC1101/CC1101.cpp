/**
 * @file CC1101.cpp
 * @brief Implementation of the CC1101 driver.
 * @author Jakub Mašek (https://github.com/masekjakub)
 *
 * This file contains the implementation of the CC1101 class, which provides
 * functions for controlling the CC1101 RF transceiver module.
 */
#include "CC1101.h"

//#define DEBUG

/*----------------------------[CC1101 - constants]---------------------------*/
#define HEADER_LEN_WITH_LENBYTE 4
#define HEADER_LEN 3
#define FIFO_SIZE 64
/*------------------------------[END constants]------------------------------*/

/*--------------------------[CC1101 - status byte]---------------------------*/
#define FIFO_BYTES_AVAILABLE 0x0F
#define STATE_IDLE 0x00
#define STATE_RX 0x01
#define STATE_TX 0x02
#define STATE_FSTXON 0x03
#define STATE_CALIBRATE 0x04
#define STATE_SETTLING 0x05
#define STATE_RXFIFO_OVERFLOW 0x06
#define STATE_TXFIFO_UNDERFLOW 0x07
/*-----------------------------[END status byte]-----------------------------*/

/*------------------------------[CC1101 - ACK]-------------------------------*/
#define ACK_DISABLE 0x00
#define ACK_ENABLE 0x01
#define ACK_OK 0x02
#define ACK_FAIL 0x03
/*---------------------------------[END ACK]---------------------------------*/

/*------------------------[CC1101 - R/W addresses]---------------------------*/
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
/*----------------------------[END R/W addresses]----------------------------*/

/*------------------------[CC1101 - command strobes]-------------------------*/
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
/*---------------------------[END command strobes]---------------------------*/

/*--------------------[CC1101 - configuiration registers]--------------------*/
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
/*----------------------[END configuiration registers]-----------------------*/

/*------------------------[CC1101 - status register]-------------------------*/
#define PARTNUM 0xF0            // Part number
#define VERSION 0xF1            // Current version number
#define FREQEST 0xF2            // Frequency offset estimate
#define LQI 0xF3                // Demodulator estimate for link quality
#define RSSI 0xF4               // Received signal strength indication
#define MARCSTATE 0xF5          // Control state machine state
#define WORTIME1 0xF6           // High byte of WOR timer
#define WORTIME0 0xF7           // Low byte of WOR timer
#define PKTSTATUS 0xF8          // Current GDOx status and packet status
#define VCO_VC_DAC 0xF9         // Current setting from PLL cal module
#define TXBYTES 0xFA            // Underflow and # of bytes in TXFIFO
#define RXBYTES 0xFB            // Overflow and # of bytes in RXFIFO
#define BYTES_IN_FIFO_MASK 0x7F // Mask for FIFO bytes
#define OVERFLOW_FIFO_MASK 0x80 // Mask for FIFO overflow
#define RCCTRL1_STATUS 0xFC     // Last RC Oscillator Calibration Result
#define RCCTRL0_STATUS 0xFD     // Last RC Oscillator Calibration Result
/*---------------------------[END status register]---------------------------*/

/*----------------------------[CC1101 - PA table]----------------------------*/
const uint8_t PA_table_315[] = {0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2};
const uint8_t PA_table_433[] = {0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0};
const uint8_t PA_table_868[] = {0x03, 0x0F, 0x1E, 0x27, 0x50, 0x81, 0xCB, 0xC2};
const uint8_t PA_table_915[] = {0x03, 0x0E, 0x1E, 0x27, 0x8E, 0xCD, 0xC7, 0xC0};
/*------------------------------[END PA table]-------------------------------*/

inline uint32_t curMillis()
{
    return to_ms_since_boot(get_absolute_time());
}

/*-----------------------------[public methods]------------------------------*/
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

    if (my_addr >= 255 || my_addr <= 0)
    {
        printf("Invalid address. Can be between 1 and 254.\n");
        return 0;
    }
    this->my_addr = my_addr;

    gpio_init(this->ss_pin);
    gpio_init(this->gdo0_pin);
    if (this->gdo2_pin != -1)
        gpio_init(this->gdo2_pin);

    gpio_set_dir(this->ss_pin, GPIO_OUT);
    gpio_set_dir(this->gdo0_pin, GPIO_IN);
    if (this->gdo2_pin != -1)
        gpio_set_dir(this->gdo2_pin, GPIO_IN);

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

    spi_write_reg(IOCFG0, 0x01); // assert GDO0 when RXFIFO is filled above treshold or end of packet received
    spi_write_reg(MCSM1, 0x3C);  // stay in RX mode after packet received

    set_freq(ISM_868);
    memcpy(this->pa_table, PA_table_868, PA_TABLE_SIZE);

    set_power(5);
    set_address(my_addr);
    set_address_filtering(3);
    set_ack_retries(3);

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
    if (gpio_get(gdo0_pin) == 1 || (this->rx_packet_buffer.size() > 0 && !this->sending_now))
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

Packet CC1101::read_packet()
{
    Packet packet;

    if (this->rx_packet_buffer.size() > 0 && !this->sending_now)
    {
        packet = this->rx_packet_buffer.back();
        this->rx_packet_buffer.pop_back();
        return packet;
    }

    uint8_t rx_bytes = spi_read_reg(RXBYTES);

    if (rx_bytes & OVERFLOW_FIFO_MASK || rx_bytes & BYTES_IN_FIFO_MASK < HEADER_LEN_WITH_LENBYTE)
    {
        flush_rx_fifo();
        return packet;
    }

    packet.data_length = spi_read_reg(RX_FIFO) - HEADER_LEN; // read LENGTH byte
    if (packet.data_length < 0 || packet.data_length > MAX_PACKET_LEN)
    {
        packet.data_length = 0;
        flush_rx_fifo();
        return packet;
    }

    uint8_t data[packet.data_length + HEADER_LEN];
    uint8_t status[2];
    spi_read_burst(RX_FIFO_BURST, data, packet.data_length + HEADER_LEN); // dest, src, ack flag, data
    spi_read_burst(RX_FIFO_BURST, status, 2);                             // rssi, lqi+crc

    packet.src_address = data[1];
    memcpy(packet.data, &(data[HEADER_LEN]), packet.data_length);

    packet.ack_flag = data[2];
    packet.rssi = get_rssi_dbm(status[0]);
    packet.lqi = status[1] & 0x7F;
    packet.valid = status[1] & 0x80;

    if (packet.ack_flag == ACK_ENABLE && packet.valid == 1)
    {
        uint8_t ack_packet[HEADER_LEN_WITH_LENBYTE] = {HEADER_LEN, packet.src_address, this->my_addr, packet.valid == 1 ? ACK_OK : ACK_FAIL};

        spi_write_burst(TX_FIFO_BURST, ack_packet, HEADER_LEN_WITH_LENBYTE);
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

    len += HEADER_LEN_WITH_LENBYTE;

    uint8_t tx_buffer[len];
    tx_buffer[0] = len - 1; // length of payload (excluding length byte)
    tx_buffer[1] = address;
    tx_buffer[2] = this->my_addr;
    tx_buffer[3] = use_ack;

    for (int i = HEADER_LEN_WITH_LENBYTE; i < len; i++)
    {
        tx_buffer[i] = data[i - HEADER_LEN_WITH_LENBYTE];
    }

#ifdef DEBUG
    printf("Sending data: ");
    for (int i = 0; i < len - HEADER_LEN_WITH_LENBYTE; i++)
    {
        printf("%02X ", tx_buffer[i] + HEADER_LEN_WITH_LENBYTE);
    }
    printf("\n");
#endif

    this->sending_now = true;
    bool ack_ok = 0;
    uint8_t retries = ack_retries;
    srand(curMillis());
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

            // TODO change timeout based on packet length / data rate
            auto timeout = rand() % 300 + 300;
            while (curMillis() - time_sent < timeout)
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
                    else if (packet.ack_flag == ACK_FAIL)
                    {
                        ack_ok = 0;
                        break;
                    }
                    else
                    {
                        this->rx_packet_buffer.push_back(packet);
                    }
                }
                sleep_us(50);
            }
            retries--;
        }
        else
        {
            ack_ok = 1;
        }
    } while (!ack_ok && retries);
    this->sending_now = false;
    return ack_ok;
}

void CC1101::set_preset(CC1101_Preset preset)
{
    switch (preset)
    {

    case GFSK_4_8_kb:
        spi_write_reg(FSCTRL1, 0x06);
        spi_write_reg(FSCTRL0, 0x00);
        spi_write_reg(MDMCFG4, 0xC7);
        spi_write_reg(MDMCFG3, 0x83);
        spi_write_reg(MDMCFG2, 0x17); // sync mode 30/32 sync word bits + carrier sense
        spi_write_reg(MDMCFG1, 0x22);
        spi_write_reg(MDMCFG0, 0xF8);
        spi_write_reg(DEVIATN, 0x40);
        spi_write_reg(FREND1, 0x56);
        spi_write_reg(MCSM0, 0x14);
        spi_write_reg(FOCCFG, 0x16);
        spi_write_reg(BSCFG, 0x6C);
        spi_write_reg(AGCCTRL2, 0x43);
        spi_write_reg(AGCCTRL1, 0x40);
        spi_write_reg(AGCCTRL0, 0x91);
        spi_write_reg(WOREVT1, 0x87);
        spi_write_reg(WOREVT0, 0x6B);
        spi_write_reg(WORCTRL, 0xF8);
        spi_write_reg(FREND1, 0x56);
        spi_write_reg(FSCAL3, 0xE9);
        spi_write_reg(FSCAL2, 0x2A);
        spi_write_reg(FSCAL1, 0x00);
        spi_write_reg(FSCAL0, 0x1F);
        spi_write_reg(RCCTRL1, 0x41);
        spi_write_reg(RCCTRL0, 0x00);
        spi_write_reg(FSTEST, 0x59);
        spi_write_reg(PTEST, 0x7F);
        spi_write_reg(AGCTEST, 0x3F);
        spi_write_reg(TEST2, 0x81);
        spi_write_reg(TEST1, 0x35);
        spi_write_reg(TEST0, 0x09);
        spi_write_reg(FIFOTHR, 0x07);
        break;
        
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
        spi_write_reg(MDMCFG4, 0xC7);
        spi_write_reg(MDMCFG3, 0x83);
        spi_write_reg(MDMCFG2, 0x12);
        spi_write_reg(MDMCFG1, 0x22);
        spi_write_reg(MDMCFG0, 0xF8);
        spi_write_reg(DEVIATN, 0x34);
        spi_write_reg(FREND1, 0x56);
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
        spi_write_reg(MCSM0, 0x18);
        spi_write_reg(FOCCFG, 0x1D);
        spi_write_reg(BSCFG, 0x1C);
        spi_write_reg(AGCCTRL2, 0xC7);
        spi_write_reg(AGCCTRL1, 0x00);
        spi_write_reg(AGCCTRL0, 0xB0);
        spi_write_reg(FSCAL3, 0xEA);
        spi_write_reg(FSCAL2, 0x2A);
        spi_write_reg(FSCAL1, 0x00);
        spi_write_reg(FSCAL0, 0x1F);
        spi_write_reg(FSTEST, 0x59);
        spi_write_reg(TEST2, 0x88);
        spi_write_reg(TEST1, 0x31);
        spi_write_reg(TEST0, 0x09);
        spi_write_reg(FIFOTHR, 0x07);
        break;

    default:
        set_preset(ASK_OOK_4_8_kb);
        printf("Invalid preset, used ASK_OOK_4_8_kb.\n");
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
        printf("Invalid freq band.\n");
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
    memcpy(this->pa_table, pa_table, PA_TABLE_SIZE);
    spi_write_burst(PATABLE_BURST, pa_table, PA_TABLE_SIZE);
}

void CC1101::set_power(uint8_t power)
{
    if (power < 0 || power > 7)
    {
        printf("Invalid power level. Must be 0-7.\n");
        return;
    }
    spi_write_reg(FREND0, power | 0x01);
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
/*----------------------------[END public methods]---------------------------*/

/*-----------------------------[private methods]-----------------------------*/

void CC1101::spi_write_reg(const uint8_t instruction, const uint8_t data)
{
    uint8_t src[2] = {instruction, data};

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, src, 2);
    gpio_put(this->ss_pin, 1);
}

void CC1101::spi_write_burst(const uint8_t instruction, const uint8_t *data, const size_t len)
{
    uint8_t src[len + 2] = {0};
    src[0] = instruction | WRITE_BURST;

    memcpy(&(src[1]), data, len);

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, src, len + 1);
    gpio_put(this->ss_pin, 1);
}

uint8_t CC1101::spi_read_reg(const uint8_t instruction)
{
    uint8_t src = instruction | READ_BYTE;
    uint8_t dst;

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, &src, 1);
    spi_read_blocking(this->spi, src, &dst, 1);
    gpio_put(this->ss_pin, 1);
    return dst;
}

void CC1101::spi_read_burst(const uint8_t instruction, uint8_t *data, const size_t len)
{
    uint8_t src = instruction | READ_BURST;

    gpio_put(this->ss_pin, 0);
    spi_write_blocking(this->spi, &src, 1);
    spi_read_blocking(this->spi, src, data, len);
    gpio_put(this->ss_pin, 1);
}

uint8_t CC1101::spi_write_strobe(const uint8_t strobe)
{
    gpio_put(this->ss_pin, 0);
    uint8_t status;
    spi_write_blocking(this->spi, &strobe, 1);
    spi_read_blocking(this->spi, strobe, &status, 1);
    gpio_put(this->ss_pin, 1);
    return status;
}

void CC1101::flush_rx_fifo()
{
    idle();
    spi_write_strobe(SFRX);
    receive();
}
/*---------------------------[END private methods]---------------------------*/