#include <iostream>
#include "pico/stdlib.h"
#include "CC1101.h"
#include "pico/binary_info.h"

auto cc1101 = CC1101(spi0, 17, 20, 21);
bool send_now = false;

bool callback(struct repeating_timer *t)
{
    send_now = true;
    return true;
}

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

    printf("CC1101 test send\n");
    uint8_t addr = 1;
    if (!cc1101.begin(addr))
    {
        printf("CC1101 initialization failed\n");
        return 1;
    }

    cc1101.set_preset(ASK_OOK_4_8_kb);
    cc1101.receive();

    struct repeating_timer timer;
    add_repeating_timer_ms(1500, &callback, nullptr, &timer);

    size_t count = 0;
    uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                      0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
                      0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
                      0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
                      0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22};

    while (1)
    {
        if (cc1101.packet_available())
        {
            Packet packet = cc1101.read_packet();
            if (!packet.valid || packet.src_address != 3)
            {
                continue;
            }
            /*printf("Packet received: ");
            for (int i = 0; i < packet.data_length; i++)
            {
                printf("%02X ", packet.data[i]);
            }
            printf("\nSRC: %d\n", packet.src_address);
            printf("RSSI: %f\n", packet.rssi);
            printf("LQI: %d\n", packet.lqi);
            printf("CRC: %d\n", packet.valid);
            printf("ACK_FLAG: %d\n", packet.ack_flag);*/
        }

        if (send_now)
        {
            send_now = false;
            count += cc1101.send_packet(3, data, 2, true);
            count += cc1101.send_packet(3, data, 10);
            count += cc1101.send_packet(3, data, 20, true);
            data[0]++;

            printf("# Count sent: %d\n\n", count);
        }
    }
}