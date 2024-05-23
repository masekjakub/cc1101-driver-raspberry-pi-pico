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

    auto cc1101 = CC1101(spi0, 17, 20);
    if (!cc1101.begin(3))
    {
        printf("CC1101 initialization failed\n");
        return 1;
    }

    cc1101.set_preset(ASK_OOK_4_8_kb);
    cc1101.receive();

    size_t count = 0;
    while (1)
    {
        if (cc1101.packet_available())
        {
            Packet packet = cc1101.read_packet();
            if (!packet.valid || packet.src_address != 1)
            {
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
            printf("CRC: %d\n", packet.valid);
            printf("ACK_FLAG: %d\n", packet.ack_flag);
            printf("# Count received: %d\n\n\n", ++count);

            // Send same packet back
            cc1101.send_packet(packet.src_address, packet.data, packet.data_length);
        }
        sleep_ms(10);
    }
}