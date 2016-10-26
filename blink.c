#include <avr/io.h>
#include <stdint.h>
#include <stdfix.h>

#define F_CPU 16000000UL
#include <util/delay.h>

int main(void) {
    short _Accum c = 0.1hk;

    DDRB = 0xFF;
    while (1) {
        PORTB |= 1 << PB3;
        for (uint8_t i = 0; i < 100; i++) _delay_ms(1);
        PORTB &= ~(1 << PB3);
        for (uint8_t i = 0; i < 90; i++) _delay_ms(10);
    }
}
