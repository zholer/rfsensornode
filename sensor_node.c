#define F_CPU 1000000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/sleep.h>

#define nRepeats 10
#define PULSELENGTH 100 // 1 cycle in us

/* Sensor network protocol

8 bits id
4 bits node type
10 bits data
4 bits transmit freq
2 bits battery level
4 bits checksum

Total: 32 bits, 4 bytes, 2 uint16's

Optional / Add Later:
Vref
Sampling freq

*/

static const uint32_t id_mask = 0x00ffffff;
static const uint32_t nodetype_mask = 0xff0fffff;
static const uint32_t data_mask = 0b11111111111100000000001111111111;
static const uint32_t transmit_battery_mask = 0xff;

volatile uint8_t delay_multiples = 1;
uint32_t loop_count = 0;
uint16_t delay1;
uint16_t delay2;
uint16_t delay3;
uint8_t TxPin;

void set_message(uint32_t* message, uint32_t id, uint32_t node_type, uint32_t data, uint32_t transmit_freq, uint32_t battery_level) {
  *message |= (id << 24) | (node_type << 20) | (data << 10) | (transmit_freq << 6) | (battery_level << 4);

  // Compute checksum and set that as well.
  uint32_t tmp_message = *message;
  uint8_t i;
  uint8_t checksum = 0;
  for (i = 0; i < 8; ++i) {
    checksum ^= tmp_message & 0x0F;
    tmp_message >>= 4;
  }
  *message |= checksum;
}

void delay_us(int n) {
  while(n--) {
    _delay_us(PULSELENGTH);
  }
}

void setTxPin(uint8_t pin) {
  TxPin = pin; // user sets the digital pin as output
  DDRB |= (1 << pin); // Define this to be the output pin.
}


// Transmitter uses manchester encoding
void setupTransmit(uint8_t pin) {
  setTxPin(pin);
  delay1 = 4;
  delay2 = 12;
  delay3 = 124;
}

void sendSync(void) {
  PORTB |= (1 << TxPin);
  delay_us(delay1);

  PORTB &= ~(1 << TxPin); // digitalWrite(TxPin, LOW);
  delay_us(delay3);
}

void sendZero(void) {
  PORTB |= (1 << TxPin); // digitalWrite(TxPin, HIGH);
  delay_us(delay1);

  PORTB &= ~(1 << TxPin); // digitalWrite(TxPin, LOW);
  delay_us(delay2);
}  //end of send a zero


void sendOne(void) {
  PORTB |= (1 << TxPin); // digitalWrite(TxPin, HIGH);
  delay_us(delay2);

  PORTB &= ~(1 << TxPin); // digitalWrite(TxPin, LOW);
  delay_us(delay1);
} //end of send one

void transmitArray(uint8_t numBytes, uint8_t *data) {
  uint8_t repeat_count;
  for (repeat_count = 0; repeat_count < nRepeats; ++repeat_count) {
    // Send the user data
    uint8_t i;
    for (i = 0; i < numBytes; i++)
    {
      uint16_t mask = 0x01; //mask to send bits
      uint8_t d = data[i];
      uint8_t j;
      for (j = 0; j < 8; j++)
      {
        if ((d & mask) == 0)
          sendZero();
        else
          sendOne();
        mask <<= 1; //get next bit
      }//end of byte
    }//end of data
    sendSync();

  }
}//end of send the data

void transmitMessage(uint32_t message) {
  uint8_t byteData[4] = {message & 0xFF, message >> 8, message >> 16, message >> 24};
  transmitArray(4, byteData);
}

void system_sleep(void);

void setup_watchdog(int ii) {
  cli();
  uint8_t tout;
  if (ii > 9)
    ii = 9;
  tout = ii & 7;
  if (ii > 7)
    tout |= _BV(WDP3);

  tout |= _BV(WDCE);

  MCUSR &= ~_BV(WDRF);
  cli();
  WDTCR |= _BV(WDCE) | _BV(WDE);
  WDTCR = tout;
  WDTCR |= _BV(WDIE);
  sei();
}

void system_sleep() {
  ADCSRA &= ~(1<<ADEN);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  ADCSRA |= (1<<ADEN);
}

ISR(PCINT0_vect) {
  if (++delay_multiples > 3) {
    delay_multiples = 0;
  }
  // Switch PB3 on and off delay_multiples times.
  uint8_t i;
  for (i = 0; i <= delay_multiples; ++i) {
    PORTB |= (1 << TxPin); // digitalWrite(TxPin, HIGH);
    _delay_ms(100);

    PORTB &= ~(1 << TxPin); // digitalWrite(TxPin, LOW);
    _delay_ms(100);
  }
  // Also init a transmit.
  // Will only init a transit within max of 8 seconds.
  loop_count = UINT32_MAX - 1;
}

int main (void) {
  // Setup transmitter on PB3 (pin 2).
  setupTransmit(DDB3);

  // Setup external interrupt on pin 6.
  GIMSK |= (1 << PCIE);
  PCMSK |= (1 << PCINT1); 

  wdt_disable();

  // Set watchdog to trigger every 8 seconds.
  setup_watchdog(WDTO_8S);

  ADCSRA |= (1 << ADPS1) | (1 << ADPS0); // Set ADC prescaler to 128 - 125KHz sample rate @ 16MHz

  ADMUX |= (0 << REFS0); // Set ADC ref to INTERNAL 5v
  ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading


  //ADCSRA |= (1 << ADATE);  // My change.. Tiny free-Running Mode enabled.
  ADCSRA |= (1 << ADEN);  // Enable ADC

  sei();   // Enable Global Interrupts

  for(;;)  // Loop Forever
  {
    if (loop_count < 7 * pow(10, delay_multiples)) {
      system_sleep();
      continue;
    }
    loop_count = 0;
    ADMUX &= ~((1 << MUX3) | (1 << MUX2));
    ADMUX |= (1 << MUX1); //use ADC2 on PB4
    ADCSRA |= (1 << ADSC);  // Start A2D Conversions
    while (ADCSRA & (1<<ADSC)); // Conversion still going on, so wait.
    uint8_t temp = ADCL;
    uint32_t final = (temp >> 6) | (ADCH << 2);

    // Measure internal 1.1V reference.
    ADMUX &= ~(1 << MUX1);
    ADMUX |= (1 << MUX3) | (1 << MUX2);
    delay_us(10); // Wait for vcc to stabilize
    ADCSRA |= (1 << ADSC);  // Start A2D Conversions
    while (ADCSRA & (1<<ADSC)); // Conversion still going on, so wait.
    temp = ADCL;
    uint16_t full_battery = (temp >> 6) | (ADCH << 2);
    full_battery = 1102943L / full_battery; // Vcc in millivolts
    uint32_t battery_level;
    if (full_battery < 2000) {
      battery_level = 0;
    } else if (full_battery < 2500) {
      battery_level = 1;
    } else if (full_battery < 3000) {
      battery_level = 2;
    } else {
      battery_level = 3;
    }

    // TODO: Make this dynamic based on sensor start / attiny internal id.
    uint32_t id = 123;
    uint32_t message = 0;
    set_message(&message, id, 1, final, delay_multiples, battery_level); 
    transmitMessage(message);
  }
}

ISR(WDT_vect) {
  sleep_disable();          // Disable Sleep on Wakeup
  loop_count++;
  sleep_enable();           // Enable Sleep Mode
}
