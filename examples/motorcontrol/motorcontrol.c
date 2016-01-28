#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


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

uint8_t receivebyte(void)
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void setup(void)
{
    setup_usart();

    DDRB = _BV(PB1) | _BV(PB2) | _BV(PB3);
}

int main(void)
{
    setup();
    sendstring("serial port motor control\r\n");

    for (;;) {
        uint8_t c = receivebyte();
        switch (c) {
        case 'o':
            break;
        }
        sendbyte(c);
    }

    return 0;
}
