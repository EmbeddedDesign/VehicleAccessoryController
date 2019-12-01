// Include headers
#include <avr/power.h>
#include "util/delay.h"
#include <stdio.h>
#include "uart.h"

// Pin definitions
#define PARK_FRONT  14 // A0
#define PARK_REAR   15 // A1
#define PARK_STATUS 6
#define ACC         2

void setup() {
  // Turn off everything we can
  ADCSRA = 0;  // Disable ADC
  SPCR = 0; // Disable SPI
  power_adc_disable ();
  power_spi_disable();
  power_twi_disable();
  power_timer1_disable();
  // Turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS);

  // Configure IO pins
  DDRC |= _BV(PORTC0); // pinMode(PARK_FRONT, OUTPUT);
  DDRC |= _BV(PORTC1); // pinMode(PARK_REAR, OUTPUT);
  DDRD |= _BV(PORTD6); // pinMode(PARK_STATUS, OUTPUT);
  pinMode(ACC, INPUT);
  
//  attachInterrupt(digitalPinToInterrupt(ACC), ACC_ISR, CHANGE);
  
//  Serial.begin(115200);
//  Serial.println("System Status: On");
  // UART init
  uart_init();
}

void loop() {
  if(digitalRead(ACC)) {
    enableParkMode();
  } else {
    disableParkMode();
  }
  delay(200);
}

void uart_init() {
  
}

//void ACC_ISR() {
//  if(digitalRead(ACC)) {
//    enableParkMode();
//  } else {
//    disableParkMode();
//  }
//}

void enableParkMode() {
  digitalWrite(PARK_FRONT, LOW);
  digitalWrite(PARK_REAR, LOW);
  digitalWrite(PARK_STATUS, LOW);
//  Serial.println("Park Mode: Enabled");
}

void disableParkMode() {
  digitalWrite(PARK_FRONT, HIGH);
  digitalWrite(PARK_REAR, HIGH);
  digitalWrite(PARK_STATUS, HIGH);
//  Serial.println("Park Mode: Disabled");
}
