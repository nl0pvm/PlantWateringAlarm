#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include "usiTwiSlave.h"
#include "MovingAverage.h"

#define USI_SCK PA4
#define USI_MISO PA5
#define USI_CS PA6
#define LED_K PB0 
#define LED_A PB1

#define DEFAULT_ADDRESS         0x22 // 0x22
#define RAW_TEMPERATURE_OFFSET  90
// 0x20 - 94
// 0x21 - 94
// 0x22 - 90
// 0x23 - 106

#define SMOOTHING_BUFFER_SIZE   8    // MUST only be 2/4/8/16/32/64/etc.
#define FIRMWARE_VERSION        0x26 // 2.6
#define TWI_GET_CAPACITANCE     0x00
#define TWI_SET_ADDRESS         0x01
#define TWI_GET_ADDRESS         0x02
#define TWI_MEASURE_LIGHT       0x03
#define TWI_GET_LIGHT           0x04
#define TWI_GET_TEMPERATURE     0x05
#define TWI_RESET               0x06
#define TWI_GET_VERSION         0x07
#define TWI_SLEEP               0x08
#define TWI_GET_BUSY            0x09

uint16_t currCapacitance = 0;
MovingAverage <uint16_t, SMOOTHING_BUFFER_SIZE> capacitanceFilter;

uint16_t currTemperature = 0;
MovingAverage <uint16_t, SMOOTHING_BUFFER_SIZE> temperatureFilter;

MovingAverage <uint16_t, SMOOTHING_BUFFER_SIZE> lightFilter;

void(* resetFunc) (void) = 0;//declare reset function at address 0

//------------ peripherals ----------------

void inline ledOn() {
  DDRB |= _BV(LED_A) | _BV(LED_K); //forward bias the LED
  PORTB &= ~_BV(LED_K);            //flash it to discharge the PN junction capacitance
  PORTB |= _BV(LED_A);  
}

void inline ledOff() {
  DDRB &= ~(_BV(LED_A) | _BV(LED_K)); //make pins inputs
  PORTB &= ~(_BV(LED_A) | _BV(LED_K));//disable pullups
}

//------------------- initialization/setup-------------------

void inline setupGPIO() {
    PORTA |= _BV(PA0);  //nothing
    PORTA &= ~_BV(PA0);                     
    PORTA |= _BV(PA2);  //nothing
    PORTA &= ~_BV(PA2);                     
    PORTA |= _BV(PA3);  //nothing
    PORTA &= ~_BV(PA3);
    
    DDRA &= ~_BV(PA1);   //enable pullup PA1
    PORTA |= _BV(PA1);  //input

    DDRB |= _BV(PB0);   //nothing
    PORTB &= ~_BV(PB0);
    DDRB |= _BV(PB1);   //nothing
    PORTB &= ~_BV(PB1);
    
    DDRB |= _BV(PB2);   //sqare wave output
    PORTB &= ~_BV(PB2);
}

//--------------- sleep / wakeup routines --------------

ISR(ADC_vect) { 
  uint16_t result = ADCW;
  
  if (currCapacitance == 255 && currTemperature == 0) {
    currCapacitance = 254;
    ADCSRA |= _BV(ADSC); //start second conversion
  } else if (currCapacitance == 254 && currTemperature == 0) {
    currCapacitance = 0;
    capacitanceFilter.add(result);
    getTemperature();
  } else if (currCapacitance == 0 && currTemperature == 255) {
    currTemperature = 254;
    ADCSRA |= _BV(ADSC); //start second conversion
  } else if (currCapacitance == 0 && currTemperature == 254) {
    currTemperature = 0;
    temperatureFilter.add(result-RAW_TEMPERATURE_OFFSET);
    getCapacitance();
  } else {
    currCapacitance = currTemperature = 0;
    getCapacitance();
  }
}

// ------------------ capacitance measurement ------------------

void startExcitationSignal() {
  TCCR0A = 0;                 // set entire TCCR0A register to 0
  TCCR0A = _BV(WGM01)         // CTC mode
          |_BV(COM0A0);       // Toggle OC0A output on every match
  TCCR0B = 0;                 // same for TCCR0B
  TCNT0  = 0;                 // initialize counter value to 0
  
  // set compare match register for 1000000 Hz increments
  OCR0A = 0; // = 1000000 / (1 * 1000000) - 1 (must be <256)
  
  // Set CS02, CS01 and CS00 bits for 1 prescaler
  TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00);
}

void stopExcitationSignal() {
  TCCR0B = 0;
  TCCR0A = 0;
}

void getCapacitance() {
  if (currTemperature == 0) {
    currCapacitance = 255;
    ADMUX = 0;
    ADMUX |= _BV(MUX0);                 // select ADC1 as input
    
    ADCSRA |= _BV(ADPS2);               // adc clock speed = sysclk/16
    ADCSRA |= _BV(ADIE);                // enable interrupt
    ADCSRA |= _BV(ADEN);
    ADCSRA |= _BV(ADSC);                // Enable AD and start conversion
 }
}

/*
Temperature MeasurementThe temperature measurement is based on an on-chip temperature sensor that is coupled to a 
single-ended ADC8 channel. Selecting the ADC8 channel by writing the MUX5:0 bits in the ADMUX register to “100010”
enables the temperature sensor. The internal 1.1V reference must also be selected for the ADC reference source 
in the temperature sensor measurement. When the temperature sensor is enabled, the ADC converter can be used in 
single conversion mode to measure the voltage over the temperature sensor. The measured voltage has a linear 
relationship to the temperature, as described in Table 18-2 on page 129. The voltage sensitivity is 
approximately 1mV/°C, and the accuracy of the temperature measurement is ±10°C after offset calibration. Bandgap 
is always calibrated, and its accuracy is only guaranteed between 1.0V and 1.2V
*/

void getTemperature(){
  if (currCapacitance == 0) { 
    currTemperature = 255;
    ADMUX = 0;
    
    // Set internal 1.1V reference, temperature reading
    ADMUX = _BV(REFS1) | _BV(MUX5) | _BV(MUX1);   
    
    ADCSRA |= _BV(ADPS2);               // adc clock speed = sysclk/16
    ADCSRA |= _BV(ADIE);                // enable interrupt
    ADCSRA |= _BV(ADEN);
    ADCSRA |= _BV(ADSC);                // Enable AD and start conversion
  }
}

//--------------------- light measurement --------------------

ISR(PCINT1_vect) {
    GIMSK &= ~_BV(PCIE1);               //disable pin change interrupts
    TCCR1B = 0;                         //stop timer
    lightFilter.add(TCNT1);
    PCMSK1 &= ~_BV(PCINT8);
    TIMSK1 &= ~_BV(TOIE1);
    getLight();
}

ISR(TIM1_OVF_vect) {
    TCCR1B = 0;                         //stop timer
    GIMSK &= ~_BV(PCIE1);               //disable pin change interrupts
    lightFilter.add(65535);
    PCMSK1 &= ~_BV(PCINT8);
    TIMSK1 &= ~_BV(TOIE1);
    getLight();
}

void getLight() {
    // PRR &= ~_BV(PRTIM1);
    TIMSK1 |= _BV(TOIE1);               //enable timer overflow interrupt

    ledOn();

    PORTB |= _BV(LED_K);                //reverse bias LED to charge capacitance in it
    PORTB &= ~_BV(LED_A);
    DDRB &= ~_BV(LED_K);                //make Cathode input
    PORTB &= ~(_BV(LED_A) | _BV(LED_K));//disable pullups
    
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = _BV(CS12);                 //start timer1 with prescaler clk/256
    
    PCMSK1 |= _BV(PCINT8);              //enable pin change interrupt on LED_K
    GIMSK |= _BV(PCIE1); 
}

// ----------------- sensor mode loop hack ---------------------

void loopSensorMode() {
  getCapacitance();
  getLight();

  
  while(1) {
      if(usiTwiDataInReceiveBuffer()) {
      uint8_t usiRx = usiTwiReceiveByte();
      if(TWI_GET_CAPACITANCE == usiRx) {
        if (capacitanceFilter.get() != 0){
        usiTwiTransmitByte(capacitanceFilter.get() >> 8);
        usiTwiTransmitByte(capacitanceFilter.get() &0x00FF);
        } else {
          usiTwiTransmitByte((1) >> 8);
          usiTwiTransmitByte((1) &0x00FF);
        } 
        // getCapacitance();
      } else if(TWI_SET_ADDRESS == usiRx) {
        uint8_t newAddress = usiTwiReceiveByte();
        // 1st bit is reserved for protocol.
        // Several addresses in the 7-bit range are reserved
        // https://www.nxp.com/docs/en/user-guide/UM10204.pdf
        if(newAddress >= 8 && newAddress <= 123) {
          eeprom_write_byte((uint8_t*)0x01, newAddress);
        }
      } else if(TWI_GET_ADDRESS == usiRx) {
        uint8_t newAddress = eeprom_read_byte((uint8_t*) 0x01);
        usiTwiTransmitByte(newAddress);
      } else if(TWI_MEASURE_LIGHT == usiRx) {
        // getLight();
      } else if(TWI_GET_LIGHT == usiRx) {
        usiTwiTransmitByte(lightFilter.get() >> 8);
        usiTwiTransmitByte(lightFilter.get() & 0x00FF);
        // getLight();
      } else if(TWI_GET_VERSION == usiRx) {
        usiTwiTransmitByte(FIRMWARE_VERSION);
      } else if(TWI_GET_TEMPERATURE == usiRx) {
        if (temperatureFilter.get() != 0){
          usiTwiTransmitByte(temperatureFilter.get() >> 8);
          usiTwiTransmitByte(temperatureFilter.get() & 0x00FF);
        } else {
          usiTwiTransmitByte((0) >> 8);
          usiTwiTransmitByte((0) &0x00FF);
        }  
        //getTemperature();
      } else if(TWI_RESET == usiRx) {
        // clean buffer at boot
        resetFunc();
      }  else {
        while(usiTwiDataInReceiveBuffer()) {
          usiTwiReceiveByte();//clean up the receive buffer
        }
      } 
    }
  }
}


//-----------------------------------------------------------------

int main (void) {
  setupGPIO();
  ledOn();

  uint8_t address = eeprom_read_byte((uint8_t*)0x01);
  if(0 == address || 255 == address) {
    address = DEFAULT_ADDRESS;
  }

  usiTwiSlaveInit(address);

  CLKPR = _BV(CLKPCE);
  CLKPR = _BV(CLKPS1); //clock speed = clk/4 = 2Mhz
  PRR = 0;                  // Disable all powersaving
  startExcitationSignal();

  sei();

  while(1){
    if(usiTwiDataInReceiveBuffer()){
      ledOff();
      loopSensorMode();
    }
  }
}
