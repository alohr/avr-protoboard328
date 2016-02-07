#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//#define ONE_DIRECTION 
#define TWO_DIRECTIONS

#if !defined(ONE_DIRECTION) && !defined(TWO_DIRECTIONS)
#error Must define either ONE_DIRECTION or TWO_DIRECTIONS
#elif defined(ONE_DIRECTION) && defined(TWO_DIRECTIONS)
#error Must not define ONE_DIRECTION and TWO_DIRECTIONS
#endif

#define TIMER1_CLKFUDGE 3
#define TIMER1_GETVALUE(x) ((x) >> 1)

#define TIMER2_PRESCALE_DIVIDER 256
#define PWM_MIN 0x40
#define PWM_MAX 0xff

#define PULSEWIDTH_MARGIN 10
#define PULSEWIDTH_MIN 1000
#define PULSEWIDTH_MID 1500
#define PULSEWIDTH_MAX 2000

#define BACKWARD 0
#define FORWARD 1

// --------------------------
// TIMER1 - input capture
// --------------------------

volatile uint16_t pulsewidth = 0; // set by TIMER1_CAPT_vect interrupt routine

ISR(TIMER1_CAPT_vect)
{
    uint16_t icr1 = ICR1;
    TCNT1 = 0;

    if (bit_is_set(TCCR1B, ICES1)) {
        // was rising edge -> set to detect falling edge
        TCCR1B &= ~_BV(ICES1);
    } else {
        pulsewidth = icr1 + TIMER1_CLKFUDGE;
        // was falling -> now set to detect rising edge
        TCCR1B |= _BV(ICES1);
    }
}

void setup_timer1(void)
{
    TCCR1A = 0;

    TCCR1B |= _BV(ICES1); // trigger input capture on rising edge
    TCCR1B |= _BV(CS11);  // set prescaler /8

    TIMSK1 = _BV(ICIE1);  // input capture interrupt enable
}

// --------------------------
// TIMER2 - motor pwm control
// --------------------------

void setup_timer2(void)
{
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
    setup_timer1();
    setup_timer2();

    DDRB &= ~_BV(PB0); // ICP1 input pin = PB0
    
    // LD293 control pins
    DDRB |= _BV(PB1) | _BV(PB2) | _BV(PB3);
}

long map(int x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void set_motor_pins(int direction, uint8_t pwm)
{
    if (pwm != 0 && direction == BACKWARD) {
        PORTB &= ~_BV(PB2);
        PORTB |= _BV(PB1);
    } else if (pwm != 0 && direction == FORWARD) {
        PORTB &= ~_BV(PB1);
        PORTB |= _BV(PB2);
    } else {
        PORTB &= ~_BV(PB1);
        PORTB &= ~_BV(PB2);
    }
    timer2_set_oc2a(pwm);
}

void one_direction(uint16_t reading)
{
    if (reading > PULSEWIDTH_MAX - PULSEWIDTH_MARGIN) {
        // full speed
        set_motor_pins(FORWARD, 0xff);
    } else if (reading < (PULSEWIDTH_MIN + PULSEWIDTH_MARGIN)) {
        // off
        set_motor_pins(FORWARD, 0);
    } else {
        set_motor_pins(FORWARD, map(reading, 1000, 2000, PWM_MIN, PWM_MAX));
    }
}

void two_directions(uint16_t reading)
{
    if (reading < PULSEWIDTH_MID - PULSEWIDTH_MARGIN) {
        // backward
        if (reading < PULSEWIDTH_MIN + PULSEWIDTH_MARGIN) {
            // full speed
            set_motor_pins(BACKWARD, 0xff);
        } else {
            uint8_t pwm = PWM_MAX - map(reading, 1000, 1500, PWM_MIN, PWM_MAX);
            set_motor_pins(BACKWARD, pwm);
        }
    } else if (reading > PULSEWIDTH_MID + PULSEWIDTH_MARGIN) {
        // forward
        if (reading > PULSEWIDTH_MAX - PULSEWIDTH_MARGIN) {
            // full speed
            set_motor_pins(FORWARD, 0xff);
        } else {
            set_motor_pins(FORWARD, map(reading, PULSEWIDTH_MID, PULSEWIDTH_MAX, 0x40, 0xff));
        }
    } else {
        set_motor_pins(FORWARD, 0);
    }
}

int main(void)
{
    setup();
    sei();

    PORTB |= _BV(PB2);

    for (;;) {
        uint16_t reading = TIMER1_GETVALUE(pulsewidth);

#if defined(ONE_DIRECTION)
        one_direction(reading);
#elif defined(TWO_DIRECTIONS)
        two_directions(reading);
#endif
        _delay_ms(10);

    }

    return 0;
}
