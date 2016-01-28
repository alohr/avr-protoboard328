#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define TIMER2_PRESCALE_DIVIDER 8


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

void setup_timer2(void)
{
    DDRB |= _BV(PB3); // OC2A pin

    TCCR2A = 0;
    TCCR2B = 0;

#if F_CPU == 16000000
#if TIMER2_PRESCALE_DIVIDER == 1
    // no prescale 
    TCCR2B &= ~_BV(CS22);
    TCCR2B &= ~_BV(CS21);
    TCCR2B |= _BV(CS20);
#elif TIMER2_PRESCALE_DIVIDER == 8
    // prescale /8
    TCCR2B &= ~_BV(CS22);
    TCCR2B |= _BV(CS21);
    TCCR2B &= ~_BV(CS20);
#elif TIMER2_PRESCALE_DIVIDER == 32
    // prescale /32
    TCCR2B &= ~_BV(CS22);
    TCCR2B |= _BV(CS21);
    TCCR2B |= _BV(CS20);
#elif TIMER2_PRESCALE_DIVIDER == 64
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

void setup(void)
{
    setup_timer2();

    DDRD = _BV(PD6) | _BV(PD7);
    DDRB = _BV(PB1) | _BV(PB2) | _BV(PB3);
}

int main(void)
{
    setup();
xo    // setup_usart();
    PORTB |= _BV(PB2);

    for (;;) {
        timer2_set_oc2a(230);
        _delay_ms(2000);
        timer2_set_oc2a(255);
        _delay_ms(2000);

    }

    return 0;
}
