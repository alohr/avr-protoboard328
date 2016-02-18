#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "timer0.h"

//                           +-\/-+
//               reset PC6  1|    |28  PC5 display cathode 3
// display anode seg A PD0  2|    |27  PC4 display cathode 2
// display anode seg B PD1  3|    |26  PC3 display cathode 1
// display anode seg C PD2  4|    |25  PC2 display cathode 0
// display anode seg D PD3  5|    |24  PC1
// display anode seg E PD4  6|    |23  PC0 ADC0 potentiometer
//                     VCC  7|    |22  GND
//                     GND  8|    |21  AREF
//               XTAL1 PB6  9|    |20  AVCC
//               XTAL2 PB7 10|    |19  PB5
// display anode seg F PD5 11|    |18  PB4
// display anode seg G PD6 12|    |17  PB3
//       push button 2 PD7 13|    |16  PB2 led cathode
//       push button 1 PB0 14|    |15  PB1 OC1A pwm output
//                           +----+

#if F_CPU == 1000000
  #define TIMER2_PRESCALE 8
#elif F_CPU == 8000000 || F_CPU == 16000000
  #define TIMER2_PRESCALE 64
#else
  #error F_CPU not recognized
#endif

enum {
    TIMER2_RESET_TO_400_MICROS = 256 - (F_CPU / TIMER2_PRESCALE / 2500),

    PIN_LED = PB2,
    PIN_BUTTON1 = PB0,
    PIN_BUTTON2 = PD7,

    PIN_CATHODE_DIGIT_0 = PC2,
    PIN_CATHODE_DIGIT_1 = PC3,
    PIN_CATHODE_DIGIT_2 = PC4,
    PIN_CATHODE_DIGIT_3 = PC5,
    PIN_CATHODES_MASK = (_BV(PIN_CATHODE_DIGIT_0) | _BV(PIN_CATHODE_DIGIT_1) |
                         _BV(PIN_CATHODE_DIGIT_2) | _BV(PIN_CATHODE_DIGIT_3)),

    PIN_ANODE_SEG_A = PD0,
    PIN_ANODE_SEG_B = PD1,
    PIN_ANODE_SEG_C = PD2,
    PIN_ANODE_SEG_D = PD3,
    PIN_ANODE_SEG_E = PD4,
    PIN_ANODE_SEG_F = PD5,
    PIN_ANODE_SEG_G = PD6,
    PIN_ANODES_MASK = (_BV(PIN_ANODE_SEG_A) | _BV(PIN_ANODE_SEG_B) | _BV(PIN_ANODE_SEG_C) |
                       _BV(PIN_ANODE_SEG_D) | _BV(PIN_ANODE_SEG_E) | _BV(PIN_ANODE_SEG_F) |
                       _BV(PIN_ANODE_SEG_G))
};

static int segment[] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
};

typedef struct {
    uint8_t digits[4];
    uint8_t digit;
    uint8_t segment;
    int value;
    int on;
} display_t;

typedef struct {
    int raw, prevraw;
    int v;
    long t;
} analogvalue_t;

static volatile display_t display;

static void display_update(volatile display_t *d)
{
    if (d->on) {
        // digit
        uint8_t port_c = PORTC;
        port_c |= PIN_CATHODES_MASK;

        switch (d->digit) {
        case 0:
            port_c &= ~_BV(PIN_CATHODE_DIGIT_0);
            break;
        case 1:
            if (d->digits[1] != 0)
                port_c &= ~_BV(PIN_CATHODE_DIGIT_1);
            break;
        case 2:
            if (d->digits[2] != 0)
                port_c &= ~_BV(PIN_CATHODE_DIGIT_2);
            break;
        case 3:
            if (d->digits[3] != 0)
                port_c &= ~_BV(PIN_CATHODE_DIGIT_3);
            break;
        }

        PORTC = port_c;

        // segment
        int value = d->digits[d->digit];
        uint8_t port_d = (PORTD & _BV(PD7));
        PORTD = (segment[value] & _BV(d->segment)) | port_d;

        if (++d->segment == 7) {
            d->segment = 0;
            if (++d->digit == 4)
                d->digit = 0;
        }
    }
}

static void display_on(volatile display_t *d, int on)
{
    cli();
    d->on = on;
    sei();
}

static void display_off(volatile display_t *d)
{
    display_on(d, 0);
}

void display_toggle(volatile display_t *d)
{
    cli();
    d->on = !d->on;
    if (!d->on)
        display_off(d);
    sei();
}

static void display_set(volatile display_t *d, int value)
{
    d->digits[0] = value % 10;
    d->digits[1] = (value / 10) % 10;
    d->digits[2] = (value / 100) % 10;
    d->digits[3] = (value / 1000) % 10;
    d->value = value;
}

ISR(TIMER2_OVF_vect)
{
    // timer interrupt overflows every 400 microseconds
    TCNT2 = TIMER2_RESET_TO_400_MICROS;
    display_update(&display);
}

void setup_timer2(void)
{
#if F_CPU == 8000000
    // prescale /32
    TCCR2A = 0;
    TCCR2B = _BV(CS21) | _BV(CS20);
#elif F_CPU == 16000000
    // prescale /64
    TCCR2A = 0;
    TCCR2B = _BV(CS22);
#else
#error F_CPU not recognized
#endif

    // overflow interrupt enable
    TIMSK2 |= _BV(TOIE2);
    TCNT2 = TIMER2_RESET_TO_400_MICROS;

    sei();
}

void setup(void)
{
    // turn rx/tx on PD0 and PD1 off
    UCSR0B = 0;

    // display anodes PD0 .. PD6
    DDRD = PIN_ANODES_MASK;

    // display cathodes
    DDRC = PIN_CATHODES_MASK;
    PORTC |= PIN_CATHODES_MASK;

    // Set ADC prescaler /128, 16 Mhz / 128 = 125 KHz which is inside
    // the desired 50-200 KHz range
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    // LED pin
    DDRB |= _BV(PIN_LED);
    PORTB |= _BV(PIN_LED);
}

void analog_init(analogvalue_t *value)
{
    value->raw = 0;
    value->prevraw = 0;
    value->v = 0;
    value->t = millis();
}

int analog_read(int chan, analogvalue_t *value)
{
    uint8_t low, high;

    // AVCC with external capacitor at AREF pin, select channel
    ADMUX = (_BV(REFS0) | (chan & 0x0f));

    // start single conversion
    ADCSRA |= _BV(ADSC);

    // wait for conversion to complete
    loop_until_bit_is_clear(ADCSRA, ADSC);

    low  = ADCL;
    high = ADCH;

    value->raw = (high << 8) | low;
    value->t = millis();

    return value->raw;
}

int potentiometer_read(analogvalue_t *value)
{
    analogvalue_t newvalue;
    enum { CHANNEL = 0, N_READINGS = 3 };

    int raw = 0;
    for (int i = 0; i < N_READINGS; i++) {
        analog_read(CHANNEL, &newvalue);
        raw += newvalue.raw;
    }

    newvalue.raw = raw / N_READINGS;
    if (newvalue.raw > 1020)
        newvalue.raw = 1020;

    if ((newvalue.raw > value->prevraw && newvalue.raw - value->prevraw > 3) ||
        (newvalue.raw < value->prevraw && value->prevraw - newvalue.raw > 3)) {

        if (newvalue.t - value->t > 50) {
            value->prevraw = value->raw;
            value->raw = newvalue.raw;
            value->v = newvalue.raw / 4;
            value->t = newvalue.t;
        }
    }

    return value->v;
}

void settle_on_low(volatile uint8_t *port, uint8_t mask)
{
    long t = 0, t0 = 0;
    enum { SETTLE_ON_LOW_TIMEOUT_MS = 20 };

    loop_until_bit_is_clear(*port, mask);
    t0 = millis();

    // now wait for pin to stay low for at least x milliseconds
    while ((t = millis()) - t0 < SETTLE_ON_LOW_TIMEOUT_MS) {
	if (bit_is_set(*port, mask))
	    t0 = millis();
    }
}

/*
void setup_int0(void)
{
    // set INT0 to trigger on falling edge
    EICRA |= _BV(ISC01);
    EICRA &= ~_BV(ISC00);
}

void enable_int0()
{
    EIMSK |= _BV(INT0);
}

void disable_int0()
{
    EIMSK &= ~_BV(INT0);
}
*/

/* volatile long t0 = 0; */

/* ISR(INT0_vect) */
/* { */
/*     disable_int0(); */
/*     t0 = micros(); */
/* } */

int main(void)
{
    analogvalue_t delay;

    setup();
    setup_timer0();

    // setup_int0();

    // display_off(&display);
    setup_timer2();

    analog_init(&delay);
    potentiometer_read(&delay);

    display_set(&display, delay.v);
    display_on(&display, 1);

    /* if ((PIND & _BV(PIN_SW1)) == 0) */
    /*     pin = PIN_LED_RED; */

    for (;;) {
        if ((PIND & _BV(PIN_BUTTON2)) == 0) {
            PORTB &= _BV(PIN_LED);
        } else {
            PORTB |= _BV(PIN_LED);
        }



/*
        if ((PIND & _BV(PIN_SW1)) == 0) {
            long tmax = 1000L * delay.v;
            while (micros() - t0 < tmax)
                ;
            PORTD |= _BV(pin);
        } else {
            PORTD &= ~_BV(pin);
            enable_int0();
        }

        if ((PIND & _BV(PIN_SW2)) == 0) {
            settle_on_low(&PIND, PIN_SW2);
            loop_until_bit_is_set(PIND, PIN_SW2);
            display_toggle(&display);
        }
*/

        potentiometer_read(&delay);
        if (delay.v != display.value) {
            display_set(&display, delay.v);
        }
    }

    return 0;
}
