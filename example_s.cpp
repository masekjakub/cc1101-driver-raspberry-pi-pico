#include <iostream>
#include "pico/stdlib.h"
#include "CC1101.h"
#include "pico/binary_info.h"

#define PacketHeaderSize 4

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

    printf("CC1101 test send\n");
    uint8_t addr = 1;
    auto cc1101 = CC1101(spi0, 17, 20, 21);
    if(!cc1101.begin(addr))
    {
        printf("CC1101 initialization failed\n");
        return 1;
    }

    cc1101.set_preset(GFSK_100_kb);
    cc1101.receive();

    uint8_t data[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
                      0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D,
                      0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14,
                      0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B,
                      0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22,
                      0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29,
                      0x2A, 0x2B, 0x2C, 0x2D, 0x2E, 0x2F, 0x30,
                      0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
                      0x38, 0x39, 0x3A, 0x3B, 0x3C};
    size_t count = 0;
    while (1)
    {
        sleep_ms(400);
        printf("Sending\n");
        count += cc1101.send_packet(3, data, 2, true);
        count += cc1101.send_packet(3, data, 20);
        data[0]++;

        printf("Count: %d\n", count);

        if (cc1101.packet_available())
        {
            CC1101::Packet packet = cc1101.read_packet();
            printf("Packet received");
        }
    }
}