#include <iostream>
#include "pico/stdlib.h"
#include "CC1101.h"
#include "pico/binary_info.h"

int main()
{
    stdio_init_all(); // usb

    // CC1101
    gpio_set_function(19, GPIO_FUNC_SPI);
    gpio_set_function(18, GPIO_FUNC_SPI);
    gpio_set_function(16, GPIO_FUNC_SPI);

    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    sleep_ms(2000);

    auto cc1101 = CC1101(spi0, 17, 20);
    if (!cc1101.begin(3))
    {
        printf("CC1101 initialization failed\n");
        return 1;
    }

    cc1101.set_preset(GFSK_100_kb);
    cc1101.receive();

    uint8_t data[64] = {0};
    uint8_t len = 0;
    size_t count = 0;
    while (1)
    {
        if (cc1101.packet_available())
        {
            CC1101::Packet packet = cc1101.read_packet();
            if (!packet.crc_ok)
            {
                printf("CRC error\n");
                continue;
            }

            printf("Packet received: ");
            for (int i = 0; i < packet.data_length; i++)
            {
                printf("%02X ", packet.data[i]);
            }
            printf("\nSRC: %d\n", packet.src_address);
            printf("RSSI: %f\n", packet.rssi);
            printf("LQI: %d\n", packet.lqi);
            printf("CRC: %d\n", packet.crc_ok);
            printf("ACK_FLAG: %d\n", packet.ack_flag);
            printf("Count: %d\n\n\n", ++count);
        }
        sleep_ms(10);
    }
}