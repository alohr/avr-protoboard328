#include <avr/io.h>
#include <string.h>

void setupusart(void)
{
#ifndef BAUD
#define BAUD 9600
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

uint8_t receivebyte(void)
{
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void sendstring(const char *s)
{
    if (s != NULL) {
        while (*s != '\0')
            sendbyte(*s++);
    }
}

void setup(void)
{
    DDRB |= _BV(PB0);
    PORTB = 0;

    setupusart();
}

int main(void)
{
    setup();
    sendstring("Hello, Serial Port!\r\n");

    for (;;) {
        uint8_t c = receivebyte();
        if (c == '1')
            PORTB |= _BV(PB0);
        else if (c == '0')
            PORTB &= ~_BV(PB0);
        sendbyte(c);
    }

    return 0;
}
