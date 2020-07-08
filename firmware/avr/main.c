/*
 * Vehicle Accessory Controller
 *
 */

 // Include headers
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "main.h"
#include "uart.h"

// Pin definitions
#define PARK_FRONT   14 // A0
#define PARK_RIGHT   15 // A1
#define PARK_LEFT    16 // A2
#define PARK_REAR    17 // A3
#define PARK_STATUS  18
#define POWER_STATUS 13
#define ACC          2

// Enable Park Mode
void enableParkMode(void) {
  PORTC &= ~_BV(PORTC0); // digitalWrite(PARK_FRONT, LOW);
  PORTC &= ~_BV(PORTC1); // digitalWrite(PARK_RIGHT, LOW);
  PORTC &= ~_BV(PORTC2); // digitalWrite(PARK_LEFT, LOW);
  PORTC &= ~_BV(PORTC3); // digitalWrite(PARK_REAR, LOW);
  // Turn PARK_STATUS LED ON
  PORTC |= _BV(PORTC4); // digitalWrite(PARK_STATUS, HIGH);
  puts("Park Mode: Enabled"); // Print park mode status message
}

// Disable Park Mode
void disableParkMode(void) {
  PORTC |= _BV(PORTC0); // digitalWrite(PARK_FRONT, HIGH);
  PORTC |= _BV(PORTC1); // digitalWrite(PARK_RIGHT, HIGH);
  PORTC |= _BV(PORTC2); // digitalWrite(PARK_LEFT, HIGH);
  PORTC |= _BV(PORTC3); // digitalWrite(PARK_REAR, HIGH);
  // Turn PARK_STATUS LED OFF
  PORTC &= ~_BV(PORTC4); // digitalWrite(PARK_STATUS, LOW);
  puts("Park Mode: Disabled"); // Print park mode status message
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
  stdin  = &uart_input;

  // Turn POWER_STATUS LED ON
  PORTB |= _BV(PORTB5); // digitalWrite(POWER_STATUS, HIGH);
  // Print system status message
  puts("System Status: OK");
 
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
  // Disable Pin Change Interrupt 2 while we do other stuff
  PCICR &= ~_BV(PCIE2);
  // Read PD2 using the Port D Pin Input Register (PIND)
  if (PIND & _BV(PIND2)) { // PD2 is HIGH
    enableParkMode();
  } else { // PD2 is LOW
    disableParkMode();
  }
}
