#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TIMER1_CLKFUDGE 3
#define TIMER1_GETVALUE(x) ((x) >> 1)

#define TIMER2_PRESCALE_DIVIDER 1024

#define PULSEWIDTH_MARGIN 10

// set by TIMER1_CAPT_vect interrupt routine
volatile uint16_t pulsewidth = 0;

void setup_usart(void)
{
#ifndef BAUD
#define BAUD 57600
#endif
#include <util/setbaud.h>
    UBRR0H = UBRRH_VALUE; // from setbaud.h
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~_BV(U2X0);
#endif
    // enable tx and rx
    UCSR0B = _BV(TXEN0) | _BV(RXEN0);
    // 8 data bits
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);
}

void sendbyte(uint8_t data)
{
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}

void sendstring(const char *s)
{
    if (s) {
        while (*s != '\0')
            sendbyte(*s++);
    }
}

char tohex(uint8_t nibble) 
{
    return nibble < 10
        ? '0' + nibble
        : 'A' + nibble - 10;
}

void sendhexbyte(uint8_t byte)
{
    sendbyte(tohex((byte & 0xf0) >> 4));
    sendbyte(tohex(byte & 0x0f));
}

void sendhexword(uint16_t word)
{
    sendhexbyte((word & 0xff00) >> 8);
    sendhexbyte(word & 0xff);
    sendbyte('\r');
}

ISR(TIMER1_CAPT_vect)
{
    uint16_t icr1 = ICR1;
    TCNT1 = 0;

    if (bit_is_set(TCCR1B, ICES1)) {
        // was rising edge -> set to detect falling edge
        TCCR1B &= ~_BV(ICES1);
        PORTB = _BV(PB2);
    } else {
        pulsewidth = icr1 + TIMER1_CLKFUDGE;
        // was falling -> now set to detect rising edge
        TCCR1B |= _BV(ICES1);
        PORTB &= ~_BV(PB2);
    }
}

void setup(void)
{
    DDRB &= ~_BV(PB0); // ICP1 input pin = PB0
    DDRB |= _BV(PB2); // debug led
}

void setup_timer1(void)
{
    TCCR1A = 0;

    TCCR1B |= _BV(ICES1); // trigger input capture on rising edge
    TCCR1B |= _BV(CS11);  // set prescaler /8

    TIMSK1 = _BV(ICIE1);  // input capture interrupt enable
}

void setup_timer2(void)
{
    DDRB |= _BV(PB3); // OC2A pin

    TCCR2A = 0;
    TCCR2B = 0;

#if F_CPU == 16000000
#if TIMER2_PRESCALE_DIVIDER == 64
    // prescale /64
    TCCR2B |= _BV(CS22);
    TCCR2B &= ~_BV(CS21);
    TCCR2B &= ~_BV(CS20);
#elif TIMER2_PRESCALE_DIVIDER == 256
    // prescale /256
    TCCR2B |= _BV(CS22);
    TCCR2B |= _BV(CS21);
    TCCR2B &= ~_BV(CS20);
#elif TIMER2_PRESCALE_DIVIDER == 1024
    // prescale /1024
    TCCR2B |= _BV(CS22);
    TCCR2B |= _BV(CS21);
    TCCR2B |= _BV(CS20);
#else
#error TIMER2_PRESCALE_DIVIDER not set correctly
#endif
#else
#error F_CPU not recognized
#endif

    // fast pwm mode 3
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);
}

void timer2_set_oc2a(uint8_t ocr2a)
{
    if (ocr2a) {
        // clear OC2A on compare match (non-inverting mode)
        TCCR2A |= _BV(COM2A1);
        TCCR2A &= ~_BV(COM2A0);
        OCR2A = ocr2a;
    } else {
        // OC2A disconnected
        TCCR2A &= ~(_BV(COM2A1) | _BV(COM2A0));
    }
}

static long map(int x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int main(void)
{
    setup();
    setup_usart();
    setup_timer1();
    setup_timer2();
    sei();

    for (;;) {
        uint16_t reading = TIMER1_GETVALUE(pulsewidth);
        sendhexword(reading);

        if (reading > 2000 - PULSEWIDTH_MARGIN) {
            timer2_set_oc2a(0xff);
        } else if (reading < (1000 + PULSEWIDTH_MARGIN)) {
            timer2_set_oc2a(0);
        } else {
            timer2_set_oc2a(map(reading, 1000, 2000, 0, 256));
        }

        _delay_ms(10);
    }

    return 0;
}
