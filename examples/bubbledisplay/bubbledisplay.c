#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#if F_CPU == 1000000
  #define TIMER0_PRESCALE 8
#elif F_CPU == 8000000 || F_CPU == 16000000
  #define TIMER0_PRESCALE 64
#else
  #error F_CPU not recognized
#endif

#define TIMER1_PRESCALE_ADJUST(x) ((x) << 1)

enum {
    TIMER0_RESET_TO_400_MICROS = 256 - (F_CPU / TIMER0_PRESCALE / 2500),
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
} analogvalue_t;

static volatile display_t display;

static void display_update(volatile display_t *d)
{
    if (d->on) {
        // digit
        uint8_t port_c = PORTC;
        port_c |= PIN_CATHODES_MASK;

        switch (d->digit) {
        case 0: port_c &= ~_BV(PIN_CATHODE_DIGIT_0); break;
        case 1: port_c &= ~_BV(PIN_CATHODE_DIGIT_1); break;
        case 2: port_c &= ~_BV(PIN_CATHODE_DIGIT_2); break;
        case 3: port_c &= ~_BV(PIN_CATHODE_DIGIT_3); break;
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

static void display_on(volatile display_t *d)
{
    cli();
    d->on = 1;
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

ISR(TIMER0_OVF_vect)
{
    // timer interrupt overflows every 400 microseconds
    TCNT0 = TIMER0_RESET_TO_400_MICROS;
    display_update(&display);
}

static void setup(void)
{
    // turn rx/tx on PD0 and PD1 off
    UCSR0B = 0;

    // display anodes PD0 .. PD6
    DDRD = PIN_ANODES_MASK;

    // display cathodes
    DDRC = PIN_CATHODES_MASK;
    PORTC |= PIN_CATHODES_MASK;
    
    // Set ADC prescaler /128, 16 Mhz / 128 = 125 KHz which is inside
    // the desired 50-200 KHz range.
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
}

static void setup_timer0(void)
{
#if F_CPU == 1000000
    // prescale /8
    TCCR0A = 0;
    TCCR0B = _BV(CS01);
#elif F_CPU == 8000000 || F_CPU == 16000000
    // prescale /64
    TCCR0A = 0;
    TCCR0B = _BV(CS01) | _BV(CS00);
#else
#error F_CPU not recognized
#endif
    // overflow interrupt enable
    TIMSK0 |= _BV(TOIE0);
    TCNT0 = TIMER0_RESET_TO_400_MICROS;
    sei();
}

static void setup_timer1(void)
{
    DDRB |= _BV(PB1); // OC1A pin

    // fast PWM, mode 14
    TCCR1A = _BV(WGM11);
    TCCR1B = _BV(WGM13) | _BV(WGM12);

    // clear OC1A on compare match (set output to low)
    TCCR1A |= _BV(COM1A1);

    // clear OC1B on compare match (set output to low)
    TCCR1A |= _BV(COM1B1);

    // set prescaler /8
    TCCR1B |= _BV(CS11);

    ICR1 = TIMER1_PRESCALE_ADJUST(20000);
    OCR1A = TIMER1_PRESCALE_ADJUST(1500);
    OCR1B = 0;
}

static void set_timer1_compare_match(int micros)
{
    OCR1A = TIMER1_PRESCALE_ADJUST(micros);
}

static void analog_init(analogvalue_t *value)
{
    value->raw = 0;
    value->prevraw = 0;
    value->v = 0;
}

static int analog_read(int chan, analogvalue_t *value)
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

    return value->raw;
}

static long map(int x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static int potentiometer_read(analogvalue_t *value)
{
    analogvalue_t newvalue;
    enum { CHANNEL = 0, N_READINGS = 3 };

    int raw = 0;
    for (int i = 0; i < N_READINGS; i++) {
        analog_read(CHANNEL, &newvalue);
        raw += newvalue.raw;
    }

    newvalue.raw = raw / N_READINGS;

    if ((newvalue.raw > value->prevraw && newvalue.raw - value->prevraw > 3) ||
        (newvalue.raw < value->prevraw && value->prevraw - newvalue.raw > 3)) {

        value->prevraw = value->raw;
        value->raw = newvalue.raw;
        value->v = map(newvalue.raw, 0, 1023, 1000, 2000);
    }

    return value->v;
}

int main(void)
{
    analogvalue_t potvalue;

    setup();
    setup_timer0();
    setup_timer1();

    analog_init(&potvalue);
    potentiometer_read(&potvalue);
    
    display_set(&display, potvalue.v);
    display_on(&display);

    for (;;) {
        potentiometer_read(&potvalue);
        if (potvalue.v != display.value) {
            set_timer1_compare_match(potvalue.v);
            display_set(&display, potvalue.v);
        }
    }

    return 0;
}
