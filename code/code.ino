/*
* reciprocal_freq_counter_ATTINY2313.c
*
* Created: 27-8-2019 15:41:08
* Author : wilko
* Modified to Arduino: pilotak
*/

#define SPI_CLK (PB0)
#define SPI_DO  (PB1)
#define SPI_CE  (PB2)
#define SPI_PORT (PORTB)

#define D_RESET (PD3)
#define D_CLK (PD4)
#define D_IN (PD5)
#define D_FF_PORT (PORTD)

#define ICP (PD6)
#define LED (PB3)


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>

void soft_spi_16(uint16_t data);
void MAX7219_shownumber(uint32_t number, uint8_t decimal_point);

volatile uint16_t input_counter_high;
volatile uint16_t val_input_counter_high;
volatile uint8_t val_input_counter_low;

volatile uint16_t reference_counter_high;
volatile uint16_t val_reference_counter_high;
volatile uint16_t val_reference_counter_low;

volatile uint8_t wait_a_second;
volatile uint16_t timeout_counter;
volatile uint8_t message;

uint32_t temp;

uint32_t input_freq;
uint32_t ref_freq;

uint32_t previous_input_freq;
uint32_t previous_reference_freq;

uint64_t real_frequency;
uint64_t tussen_freq;
uint64_t multiplier;
uint8_t dig_point;

void setup() {
    delay(500);                                             //give MAX7219 time to start

    DDRD |= (1 << D_RESET) | (1 << D_IN);                   //PD3, PD5 output (output to D-ff /RESET and D-in)
    D_FF_PORT |= (1 << D_CLK) | (1 << ICP);                 //PD4, PD6 pull up (inputs for resp. T0 en ICP)

    DDRB |= (1 << SPI_CLK) | (1 << SPI_DO) | (1 << SPI_CE); //PB0,1,2 output (to MAX7219)
    DDRB |= (1 << LED);                                     //PB3 output
    SPI_PORT |= (1 << SPI_CE);                              //set CE high


    TCCR0A = 0;                                             //normal mode
    TCCR0B |= (1 << CS02) | (1 << CS01) | (1 << CS00);      //clock input extern, RISING edge
    TIMSK |= (1 << TOIE0);                                  //interrupt bij overflow

    TCCR1A = 0;                                             //normal mode
    TCCR1B |= (1 << CS10);                                  //prescaler = 1 (10MHz)
    TCCR1B |= (1 << ICES1);                                 //input capture on RISING edge
    TCCR1C = 0;
    TIMSK |= (1 << TOIE1) | (1 << ICIE1);                   //enable interrupt on overflow & input capture

    soft_spi_16(0x0000);                                    //nop
    soft_spi_16(0x0C00);                                    //shutdown
    soft_spi_16(0x0000);                                    //nop

    soft_spi_16(0x0F00);                                    //testmode off
    soft_spi_16(0x0C01);                                    //normal operation
    soft_spi_16(0x09FF);                                    //bcd code B, all digits
    soft_spi_16(0x0A02);                                    //brightness
    soft_spi_16(0x0B07);                                    //all digits

    previous_input_freq = 0;
    previous_reference_freq = 0;
    real_frequency = 0;
    dig_point = 8;
    multiplier = 0;
    wait_a_second = 0;
    message = 0;
    timeout_counter = 765;


    sei();

    D_FF_PORT |= (1 << D_IN);                               //make D_FF high
    D_FF_PORT &= ~(1 << D_RESET);                           //activate /RESET
    delay(1);
    D_FF_PORT |= (1 << D_RESET);                            //release /RESET
}

void loop() {
    while (1) {
        if (timeout_counter == 0) {                         //no input signal
            real_frequency = 0;
            dig_point = 8;
            delay(50);
        }

        if (message == 1) {
            message = 0;

            timeout_counter = 765;                          //input signal detected, reset timeout_counter (5 second wait)

            PORTB |= (1 << LED);                            //input signal present: LED on

            D_FF_PORT &= ~(1 << D_RESET);                   //activate /RESET
            _delay_us(100);
            D_FF_PORT &= ~(1 << D_IN);                      //make D-FF low
            _delay_us(100);
            D_FF_PORT |= (1 << D_RESET);                    //release /RESET

            wait_a_second = 152;                            //start countdown to 1 second

            temp = ((uint32_t) val_input_counter_high << 8) | val_input_counter_low;
            input_freq = temp - previous_input_freq;
            previous_input_freq = temp;

            temp = ((uint32_t) val_reference_counter_high << 16) | val_reference_counter_low;
            ref_freq = temp - previous_reference_freq;
            previous_reference_freq = temp;

            if (ref_freq == 0) { ref_freq = 1; }                //never divide by 0

            if (input_freq < 10) {                              //number of received pulses in ~1 second
                multiplier = 10000000;
                dig_point = 8;

            } else if (input_freq < 100) {                      //number of received pulses in ~1 second
                multiplier = 1000000;
                dig_point = 7;

            } else if (input_freq < 1000) {                     //etc.
                multiplier = 100000;
                dig_point = 6;

            } else if (input_freq < 10000) {
                multiplier = 10000;
                dig_point = 5;

            } else if (input_freq < 100000) {
                multiplier = 1000;
                dig_point = 4;

            } else if (input_freq < 1000000) {
                multiplier = 100;
                dig_point = 3;

            } else {
                multiplier = 10;
                dig_point = 2;
            }

            tussen_freq = (uint64_t) input_freq * F_CPU * multiplier;
            real_frequency = tussen_freq / (uint64_t) ref_freq;
        }

        if (real_frequency < 99999999) {
            MAX7219_shownumber(real_frequency, dig_point);
            //MAX7219_shownumber(input_freq, dig_point);
            //MAX7219_shownumber(ref_freq, dig_point);
        }
    }
}


ISR (TIMER0_OVF_vect) {                                 //TIMER0 overflows after 256 counts
    input_counter_high++;
}

ISR (TIMER1_OVF_vect) {                                 //TIMER1 overflow after 65536 counts
    //occurs 153 times per second
    reference_counter_high++;

    if (wait_a_second > 0) { wait_a_second--; }

    else { D_FF_PORT |= (1 << D_IN); }                          //make D_FF high

    if (timeout_counter > 0) { timeout_counter--; }

    if (wait_a_second == 76) { PORTB &= ~(1 << LED); }          //LED off after half a second
}


ISR (TIMER1_CAPT_vect) {
    val_input_counter_low = TCNT0;
    val_input_counter_high = input_counter_high;


    val_reference_counter_low = ICR1;
    val_reference_counter_high = reference_counter_high;

    message = 1;                                            //signal to main program
}


void soft_spi_16(uint16_t data) {
    SPI_PORT &= ~(1 << SPI_CE);                             //set CE low

    for (uint8_t i = 0; i < 16; i++) {
        if (data & 0x8000) { SPI_PORT |= (1 << SPI_DO); }       //if MSBit high, output high

        else { SPI_PORT &= ~(1 << SPI_DO); }                    //else output low

        _delay_us(1);

        SPI_PORT |= (1 << SPI_CLK);                         //generate clock pulse
        _delay_us(1);
        SPI_PORT &= ~(1 << SPI_CLK);
        _delay_us(1);

        data <<= 1;                                         //shift to next bit
    }

    SPI_PORT |= (1 << SPI_CE);                              //all done, set CE high
}

void MAX7219_shownumber(uint32_t number, uint8_t decimal_point) {
    if (number > 999999999) { number = 999999999; }             //max. value the MAX7219 can display

    for (uint8_t i = 1; i < 9; i++) {
        if (i == decimal_point) { soft_spi_16((i << 8) | ((number % 10) | 0x80)); }

        else { soft_spi_16((i << 8) | (number % 10)); }         //output least significant digit

        number /= 10;                                       //next digit
    }
}
