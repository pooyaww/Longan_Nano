#include <msp430f2013.h>

#define NOTE_C4 3822
#define NOTE_D4 3405
#define NOTE_E4 3033
#define NOTE_F4 2863
#define NOTE_G4 2551
#define NOTE_A4 2272
#define NOTE_B4 2024

#define NOTE_C5  1911
#define NOTE_C5S 1803
#define NOTE_D5  1702
#define NOTE_D5S 1607
#define NOTE_E5  1516
#define NOTE_F5  1431
#define NOTE_F5S 1351
#define NOTE_G5  1275
#define NOTE_G5S 1203
#define NOTE_A5  1136
#define NOTE_A5S 1083
#define NOTE_B5  1012

#define INTRO
#define HIS_THEME

const int notes[] = {
#ifdef INTRO
#define BTICK 700
#define ETICK 150
    NOTE_G4, BTICK, 0, ETICK,
    NOTE_G5S, BTICK, 0, ETICK,
    NOTE_D5S, BTICK * 2, 0, ETICK * 2,

    NOTE_C5S, BTICK, 0, ETICK,
    NOTE_G5S, BTICK, 0, ETICK,
    NOTE_G4, BTICK * 2, 0, ETICK * 2,

    NOTE_G4, BTICK, 0, ETICK,
    NOTE_C5S, BTICK, 0, ETICK,
    NOTE_G5S, BTICK, 0, ETICK,
    NOTE_A5S, BTICK, 0, ETICK,

    NOTE_G5S, BTICK, 0, ETICK,
    NOTE_D5S, BTICK, 0, ETICK,
    NOTE_C5S, BTICK * 2, 0, ETICK * 2,

    0, (BTICK + ETICK) * 4,
#undef BTICK
#undef ETICK
#endif // INTRO

#ifdef HIS_THEME
#define BTICK 300
#define ETICK 70
    NOTE_B4, BTICK, 0, ETICK,
    NOTE_F5, BTICK, 0, ETICK,
    NOTE_E5, BTICK, 0, ETICK,
    NOTE_B4, BTICK, 0, ETICK,
    NOTE_D5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_D5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_E5, BTICK, 0, ETICK,

    0, BTICK + ETICK,
    NOTE_B4, BTICK, 0, ETICK,
    NOTE_E5, BTICK, 0, ETICK,
    NOTE_B4, BTICK, 0, ETICK,
    NOTE_D5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_D5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_E5, BTICK, 0, ETICK,

    NOTE_B4, BTICK, 0, ETICK,
    NOTE_F5, BTICK, 0, ETICK,
    NOTE_E5, BTICK, 0, ETICK,
    NOTE_B4, BTICK, 0, ETICK,
    NOTE_D5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_D5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_E5, BTICK, 0, ETICK,

    0, BTICK + ETICK,
    NOTE_B4, BTICK, 0, ETICK,
    NOTE_E5, BTICK, 0, ETICK,
    NOTE_G5, BTICK, 0, ETICK,
    NOTE_F5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_E5, 3 * BTICK / 2, 0, 3 * ETICK / 2,
    NOTE_F5, BTICK, 0, ETICK,

    0, (BTICK + ETICK) * 4,
#undef BTICK
#undef ETICK
#endif // HIS_THEME
};

const int length = sizeof(notes) / (sizeof(notes[0]) * 2);

volatile int cursor;
volatile int timer;

void main(void) {
    // Disable watchdog
    WDTCTL  = WDTPW | WDTHOLD;

    // CPU clock at 1Mhz
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL  = CALDCO_1MHZ;

    // Enable timerA0.0 interrupts in counting mode, timerA0.1 in output toggle mode, using SMCLK as source, up mode
    TACTL   = TASSEL_2 | MC_1;
    TACCTL0 = OUTMOD_4 | CCIE;
    TACCR0  = 1000;

    // P1.0 and P1.5 as output
    P1DIR  = 0x21;
    P1SEL  = 0x20;
    P1OUT = 0;

    // Start at the beginning of the song
    cursor = 0;
    timer = 0;

    // Go in Low power mode 0 with interrupts enabled
    __bis_SR_register(CPUOFF + GIE);
    while (1) ;
} 

void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A(void) {
    if (timer <= 0) {
        P1OUT ^= 0x01;
        int period = notes[2 * cursor + 0] / 2;
        int duration = notes[2 * cursor + 1];
        int silent = period == 0;
        TACCTL0 = CCIE | (silent ? OUTMOD_5 : OUTMOD_4);
        TACCR0  = silent ? 1000 : period;
        timer = silent ? duration : (duration * 16) / (period / 64);
        cursor = (cursor + 1) % length;
    } else timer--;
}