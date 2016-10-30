#include <avr/io.h>
#include <avr/cpufunc.h>
#include <stdint.h>

#define F_CPU 16000000UL
#include <util/delay.h>

#define CS  PB3 // Chip select
#define RST PB5 // Reset
#define DC  PB4 // Data/Command
#define SCL PB2 // Clock line
#define SDA PB0 // Data line

inline void send_byte(uint8_t byte) {
    PORTB |= 1 << SCL;
    #pragma unroll
    for (int8_t i = 7; i >= 0; i--) {
        PORTB &= ~(1 << SCL);
        if (byte & 0x80)
            PORTB |=  (1 << SDA);
        else
            PORTB &= ~(1 << SDA);
        _delay_us(5);
        PORTB |= 1 << SCL;
        byte <<= 1;
        _delay_us(5);
    }
}

void send_command(uint8_t cmd) {
    PORTB &= ~(1 << DC);
    PORTB &= ~(1 << CS);
    send_byte(cmd);
    PORTB |=  (1 << CS);
}

void send_data(uint8_t cmd) {
    PORTB |=  (1 << DC);
    PORTB &= ~(1 << CS);
    send_byte(cmd);
    PORTB |=  (1 << CS);
}

void fill(uint16_t col) {
    send_command(0x26); // Enable fill
    send_command(0x01);

    send_command(0x22); // Draw rectangle
    send_command(0x00);
    send_command(0x00);
    send_command(0x5F);
    send_command(0x3F);
    send_command((col & 0x3F) << 1);
    send_command((col >> 5) & 0x1F);
    send_command((col >> 10) & 0x3F);
    send_command((col & 0x3F) << 1);
    send_command((col >> 5) & 0x1F);
    send_command((col >> 10) & 0x3F);
}

void cursor(uint8_t x, uint8_t y) {
    send_command(0x15);
    send_command(x);
    send_command(0x5F);

    send_command(0x75);
    send_command(y);
    send_command(0x3F);
}

void init() {
    PORTB &= ~(1 << CS);

    PORTB |=  (1 << RST);
    _delay_ms(500);
    PORTB &= ~(1 << RST);
    _delay_ms(500);
    PORTB |=  (1 << RST);
    _delay_ms(500);

    send_command(0xAF); // Display OFF
    send_command(0xA0); // Set remap
    send_command(0x72); // RGB
    send_command(0xA1); // Start line
    send_command(0x00);
    send_command(0xA2); // Display offset
    send_command(0x00);

    send_command(0xA4); // Normal display
    send_command(0xA8); // Set multiplex
    send_command(0x3F); // 1/64 duty
    send_command(0xAD); // Set master
    send_command(0x8E); 
    send_command(0xB0); // Power mode
    send_command(0x0B);
    send_command(0xB1); // Precharge
    send_command(0x31);

    send_command(0xB3); // Clock div
    send_command(0xF0); // 7:4 -> freq., 3:0 -> div ratio

    send_command(0x8A); // Precharge A
    send_command(0x64);
    send_command(0x8B); // Precharge B
    send_command(0x78);
    send_command(0x8C); // Precharge C
    send_command(0x64);
    send_command(0xBB); // Precharge level
    send_command(0x3A);

    send_command(0xBE); // V_COMH
    send_command(0x3E);

    send_command(0x87); // Master current
    send_command(0x06);
    send_command(0x81); // Contrast A
    send_command(0x91);
    send_command(0x82); // Contrast B
    send_command(0x50);
    send_command(0x83); // Contrast C
    send_command(0x7D);

    send_command(0xAF); // Display ON
}

int main(void) {
    DDRB = 0xFF;

    _delay_ms(10);
    init();

    fill(0xFF);

    cursor(0, 0);

    while (1) {
        send_data(0xFF);
        send_data(0xFF);
    }
}
