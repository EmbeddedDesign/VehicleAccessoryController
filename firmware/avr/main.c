/*
 * Name: Vehicle Accessory Controller
 * Author: Noctivore
 * Description: AVR-C application code for Vehicle Accessory Controller
 * Version 1.0
 * Date: 9 July 2020
 */

 // Include headers
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>
#include "main.h"

// Serial definitions
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef BAUD
#define BAUD 115200
#endif
#include <util/setbaud.h>

#define SERIAL_DELAY 1

// Pin definitions
#define PARK_FRONT   14 // A0
#define PARK_RIGHT   15 // A1
#define PARK_LEFT    16 // A2
#define PARK_REAR    17 // A3
#define PARK_STATUS  18
#define POWER_STATUS 13
#define ACC          2

// String constants stored in flash
const __flash char status[] = "System Status: OK";
const __flash char enablePark[] = "Park Mode: Enabled";
const __flash char disablePark[] = "Park Mode: Disabled";

// URART initialization function
void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
    
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif

    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // 8-bit data
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX
}

// UART putchar function
static int uart_putchar(char c, FILE *stream) {
    if (c == '\n') {
        uart_putchar('\r', stream);
    }
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}

// Setup UART stream
static FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

// Enable Park Mode
void enableParkMode(void) {
  PORTC &= ~_BV(PORTC0); // digitalWrite(PARK_FRONT, LOW);
  PORTC &= ~_BV(PORTC1); // digitalWrite(PARK_RIGHT, LOW);
  PORTC &= ~_BV(PORTC2); // digitalWrite(PARK_LEFT, LOW);
  PORTC &= ~_BV(PORTC3); // digitalWrite(PARK_REAR, LOW);
  // Turn PARK_STATUS LED ON
  PORTC |= _BV(PORTC4); // digitalWrite(PARK_STATUS, HIGH);
  puts_P(enablePark); // Print park mode status message
  _delay_ms(SERIAL_DELAY); // Wait for serial output to finish
}

// Disable Park Mode
void disableParkMode(void) {
  PORTC |= _BV(PORTC0); // digitalWrite(PARK_FRONT, HIGH);
  PORTC |= _BV(PORTC1); // digitalWrite(PARK_RIGHT, HIGH);
  PORTC |= _BV(PORTC2); // digitalWrite(PARK_LEFT, HIGH);
  PORTC |= _BV(PORTC3); // digitalWrite(PARK_REAR, HIGH);
  // Turn PARK_STATUS LED OFF
  PORTC &= ~_BV(PORTC4); // digitalWrite(PARK_STATUS, LOW);
  puts_P(disablePark); // Print park mode status message
  _delay_ms(SERIAL_DELAY); // Wait for serial output to finish
}

// Main loop
int main(void) {
  // Configure IO
  DDRC |= _BV(DDC0);    // pinMode(PARK_FRONT, OUTPUT);
  DDRC |= _BV(DDC1);    // pinMode(PARK_RIGHT, OUTPUT);
  DDRC |= _BV(DDC2);    // pinMode(PARK_LEFT, OUTPUT);
  DDRC |= _BV(DDC3);    // pinMode(PARK_REAR, OUTPUT);
  DDRC |= _BV(DDC4);    // pinMode(PARK_STATUS, OUTPUT);
  DDRB |= _BV(DDB5);    // pinMode(POWER_STATUS, OUTPUT);
  DDRD &= ~_BV(DDD2);   // Clear the PD2 pin
  PORTD |= _BV(PORTD2); // Enable PD2 internal pull-up
  
  // UART setup
  uart_init();
  stdout = &uart_output;

  // Turn POWER_STATUS LED ON
  PORTB |= _BV(PORTB5); // digitalWrite(POWER_STATUS, HIGH);
  puts_P(status); // Print system status message
  _delay_ms(SERIAL_DELAY); // Wait for serial output to finish
 
 // Loop
  while(1) {
    // Power saving
    power_adc_disable ();
    power_spi_disable();
    power_twi_disable();
    power_timer1_disable();
    power_timer2_disable();

    // Configure sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Do not interrupt before to sleep or ISR will detach interrupts won't wake.
    cli();

    // Interrupt init
    // Enable Pin Change Interrupt on the PCINT18 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= _BV(PCINT18);
    // Enable Pin Change Interrupt 2 using the Pin Change Interrupt Control Register (PCICR)
    PCICR |= _BV(PCIE2);

    // Turn off brown-out enable in software
    MCUCR = _BV(BODS) | _BV(BODSE);
    MCUCR = _BV(BODS);

    // Enable interrupts
    sei();
    // Guaranteed sleep_cpu call will be done as CPU executes next instruction after interrupts enabled
    sleep_cpu();
  }

  return 0;
}

ISR(PCINT2_vect)
{
  // Cancel sleep as a precaution
  sleep_disable();
  // Disable Pin Change Interrupt 2 while we do stuff
  PCICR &= ~_BV(PCIE2);
  // Read PD2 using the Port D Pin Input Register (PIND)
  if (PIND & _BV(PIND2)) { // PD2 is HIGH
    enableParkMode();
  } else { // PD2 is LOW
    disableParkMode();
  }
}
