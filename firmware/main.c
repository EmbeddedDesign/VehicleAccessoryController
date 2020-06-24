/*
 * Vehicle Accessory Controller
 *
 */

 // Include headers
#include <avr/power.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "main.h"
#include "uart.h"

// Pin definitions
#define PARK_FRONT   14 // A0
#define PARK_RIGHT   15 // A1
#define PARK_LEFT    16 // A2
#define PARK_REAR    17 // A3
#define POWER_STATUS 5
#define PARK_STATUS  6
#define ACC          2

// Enable Park Mode
void enableParkMode(void) {
  PORTC &= ~_BV(PORTC0); // digitalWrite(PARK_FRONT, LOW);
  PORTC &= ~_BV(PORTC1); // digitalWrite(PARK_RIGHT, LOW);
  PORTC &= ~_BV(PORTC2); // digitalWrite(PARK_LEFT, LOW);
  PORTC &= ~_BV(PORTC3); // digitalWrite(PARK_REAR, LOW);
  // Turn PARK_STATUS LED ON
  PORTD &= ~_BV(PORTD6); // digitalWrite(PARK_STATUS, LOW);
  puts("Park Mode: Enabled"); // Print park mode status message
}

// Disable Park Mode
void disableParkMode(void) {
  PORTC |= _BV(PORTC0); // digitalWrite(PARK_FRONT, HIGH);
  PORTC |= _BV(PORTC1); // digitalWrite(PARK_RIGHT, HIGH);
  PORTC |= _BV(PORTC2); // digitalWrite(PARK_LEFT, HIGH);
  PORTC |= _BV(PORTC3); // digitalWrite(PARK_REAR, HIGH);
  // Turn PARK_STATUS LED OFF
  PORTD |= _BV(PORTD6); // digitalWrite(PARK_STATUS, HIGH);
  puts("Park Mode: Disabled"); // Print park mode status message
}

// Main loop
int main(void) {
  // Power saving
  ADCSRA = 0;  // Disable ADC
  SPCR = 0; // Disable SPI
  power_adc_disable ();
  power_spi_disable();
  power_twi_disable();
  power_timer1_disable();
  // Turn off brown-out enable in software
  // MCUCR = bit (BODS) | bit (BODSE);
  // MCUCR = bit (BODS);

  // Configure IO pins
  DDRC |= _BV(DDC0);    // pinMode(PARK_FRONT, OUTPUT);
  DDRC |= _BV(DDC1);    // pinMode(PARK_RIGHT, OUTPUT);
  DDRC |= _BV(DDC2);    // pinMode(PARK_LEFT, OUTPUT);
  DDRC |= _BV(DDC3);    // pinMode(PARK_REAR, OUTPUT);
  DDRD |= _BV(DDD5);    // pinMode(POWER_STATUS, OUTPUT);
  DDRD |= _BV(DDD6);    // pinMode(PARK_STATUS, OUTPUT);
  DDRD &= ~_BV(DDD2);   // Clear the PD2 pin
  PORTD |= _BV(PORTD2); // Enable PD2 internal pull-up
  
  // UART setup
  uart_init();
  stdout = &uart_output;
  stdin  = &uart_input;

  // Interupt init
  // Enable pin change interrupt on the PCINT18 pin using Pin Change Mask Register 2 (PCMSK2)
  PCMSK2 |= _BV(PCINT18);
  // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
  PCICR |= _BV(PCIE2);
  // Turn on interrupts
  sei();

  // Turn POWER_STATUS LED ON
  PORTD &= ~_BV(PORTD5); // digitalWrite(PARK_STATUS, LOW);
  // Print system status message
  puts("System Status: OK");
 
 // Loop
  while(1) {
    // Nothing to do here
  }

  return 0;
}

ISR(PCINT2_vect)
{
  // Read PD2 using the Port D Pin Input Register (PIND)
  if (PIND & _BV(PIND2)) { // PD2 is HIGH
    enableParkMode();
  } else { // PD2 is LOW
    disableParkMode();
  }
}
