/*
This Arduino Sketch was created to test and play with the Watchdog Timer and External Interrupts.

This allows the Arduino to lower its power consumption by powering down for a number of cycles
and then restarting either where it left of (e.g. loop()) or with a complete system restart. The
external interrupts allows the Arduino to react to given inputs (e.g. sensors) while running or
while asleep.

Documentation was obtained here:
1. http://www.atmel.com/Images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet.pdf
2. http://forum.arduino.cc/index.php?topic=45239  
3. http://citizen-sensing.org/2013/07/arduino-watchdog/
4. http://www.nongnu.org/avr-libc/user-manual/modules.html

POWER SAVING:
Here are some options to maximize power saving

1. Adjust clock frequency.
  a. The <avr/power.h> library provides "clock_prescale_set(x)" & "clock_prescale_get(x)"
  b. x = clock_div_256 scales the clock by 256. 
     See http://www.nongnu.org/avr-libc/user-manual/group__avr__power.html for other prescales
  
  NOTE: Some digital sensors and shields require a particular frequency (e.g. SD shield), so one
        would need to switch between frequencies when working with these.
  
2. Turn Off the following before Sleeping:
  a. ADC - Analog to digital converter
  b. AC - Analog Comparator (disable in all but Idle and ADC Noise Reduction mode unless 
     AC is uses the internal voltage rerence as an input
  c. BOD - Brown-out detector
  d. IVR - Internal Voltage Reference (disabled if BOD is disabled)
  e. WDT - Watchdog Timer (necessary for waking up at timed intervals unless you use an 
     external clock).
  f. Port Pins

Necessary headers for particular functions
http://www.nongnu.org/avr-libc/user-manual/modules.html

*/

#include "pitches.h"
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>


// Variables being set during interrupts need to be volatile. I don't quite understand this, so maybe
// someone can elaborate on this
volatile int interrupted = 0;
volatile unsigned long PulseCounts;

// Used to see if Arduino was sleeping when an External Interrupt is raised.
// Maybe a more efficient way to do this (e.g. check flags) rather than creating a variable.
int issleeping = 0;

// Just some stuff to play to know everything is working
// notes in the melody:
int melody[] = { 
NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4 };

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {4, 8, 8, 4,4,4,4,4 };

// ISR = Interrupt Service Routine. This is called when an interrupt occurs. There are a number of different
// types of interrupts, and one could use as many of those as one desires. An ISR({interrupt_type}_vector) must 
// be defined for each interrupt that is set. If not, the Arduino will reset when an interrupt occurs.

// ISR for the Watchdog Timer Interrupt
ISR(WDT_vect) { 
  // Stuff to do for a watchdog timer interrupt. 
}

// IST for the PCINT1 Interrupt. 
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// does the arduino go back to sleep??? perhaps call sleep_cpu
// 
ISR(PCINT1_vect){  
  PulseCounts += 1;
  if (issleeping){
    sleep_without_BOD();                                      // Go To sleep  
  }
}

// This is how we access variables used in the ISR()'s. 
// see http://forum.arduino.cc/index.php?topic=45239
unsigned long CalledFromLoopToGetPulseCounts( void ){
    unsigned long c;
    {
      uint8_t SaveSREG = SREG;   // save interrupt flag
      cli();                     // disable interrupts
      c = PulseCounts;           // access the shared data
      SREG = SaveSREG;           // restore the interrupt flag
    }
    sei();
    return( c );
}

void setup(){
  save_power();
  pinMode(13,OUTPUT);
  // Serial cannot function properly at low clock speeds
  Serial.begin(9600);
  Serial.println("Startup...");
  melod(.5);
  delay(1000);
}

void loop(){
  Serial.print("Interrupted = ");
  Serial.print(CalledFromLoopToGetPulseCounts());
  Serial.println("");
  Serial.println("New Loop...");
  melod(1);
  Serial.println("Sleeping...");
  delay(scaletime(1000));
  issleeping = 1;
  power_down_with_WDT();           // Sets the WDT and powers down
  WDT_off();                       // Turn off WDT as we no longer need it
  issleeping = 0;
  //Serial.begin(9600);
  Serial.println("Awoken...");
  
  delay(scaletime(1000));
  melod(1);
  twinkle(50,20);

}


// Returns a scaled time based on the clock prescaler. t is in seconds.
long scaletime(long t){ 
  return t/pow(2,clock_prescale_get());
} 

void save_power(){
  clock_prescale_set(clock_div_256);          //Change clock prescaler to 256
    
  power_adc_disable();                        // Disable ADC.
                                              // AC automatically disabled in Power Down Mode
                                              // BOD is done right before sleep. Also disables IVR.
                                              // NOTE: ADC is needed for reading analog sensor data.
  
}

void sleep(void){
  SMCR |= (1 << SE);                           // Enable Sleep
  sleep_cpu();                                 // Go To sleep  
}

void sleep_without_BOD(void){
  MCUCR = (1 << BODS) | (1 << BODSE);          // Enable setting of BODS
  MCUCR = (1 << BODS) | (0 << BODSE);          // Disable the brown-out-detector
  sleep();                                     // Sleep
}

void external_interrupts(void){
  cli();                                       // Should disable all interrupts before setting new ones
 // EIMSK |= (1 << INT1);                      //allow external interupts on INT0 (pin 2 for Arduino Uno)
  //EICRA |= (0 << ISC11) | (0 << ISC10);      //allow low level interrupt on pin 2

  PCICR  |= (1<<PCIE1);                        // Enable Interrupts on Analog In for Aduino Uno, Pins A0-A5
  PCMSK1 |= (1<<PCINT8);                       // Enable interrupt for pin PCINT8 
                                               // (PCINT8 = A0 ... PCINT13 = A5, Arduino Uno)
  sei();                                       // Enable global interrupts
}

void disable_external_inputs(void){
  EIMSK |= (0 << INT1);                        // Disable external interupts on INT0 (pin 2 for Arduino Uno)
}
  
void power_down_with_WDT(void){
  SMCR = (0 << SM2) | (1 << SM1) | (0 << SM0); // Select Power Down Sleep Mode
  WDT_Set();                                   // Set-up the Watchdog Timmer 
  external_interrupts();                       // Enable external input interrupts
  sleep_without_BOD();                         // Go To sleep
  //Time passes by
  //I've awoken
  SMCR |= (0 << SE);                           // Disable Sleep
  WDTCSR |= (1 << WDIE);                       // reenable WDT interrupt mode to stay in this mode (interrrupt resets WDIE).
}

// Setup the Watchdog Timer. 
// Must disable global interrupts to prevent interrupts while setting the WDT.
void WDT_Set(void){
  cli();                                       // turn off global interrupt
  wdt_reset();                                 // reset the watchdog timmer
  MCUSR &= ~(1<<WDRF);                         // Clear WDRF in MCUSR
  WDTCSR |= (1<<WDCE) | (1<<WDE);              // Start timed sequence
  WDTCSR = (1<<WDE) | (1<<WDP3) ;              // Set new prescaler(time-out) 
  WDTCSR |= (1 << WDIE) | (0 << WDE);          // enable WDT interrupt mode
  sei();                                       // enable interrupt
}

// Turn off the Watchdog Timer
// Must disable global interrupts to prevent interrupts while setting the WDT.
void WDT_off(void){
  cli();                                     // turn off global interrupt
  wdt_reset();                               // reset the watchdog timmer
  MCUSR &= ~(1<<WDRF);                       // Clear WDRF in MCUSR
  WDTCSR |= (1<<WDCE) | (1<<WDE);            // Write logical one to WDCE and WDE
                                             // Keep old prescaler setting to prevent unintentional time-out
  WDTCSR = 0x00;                             // Turn off WDT
  sei();
}

// Blink LED on pin 13
void twinkle(int d, int r){
  for (int i = 0; i<r; i++){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(d);               // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(d);               // wait for a second
  }
}

// Play some sounds to a speaker/piezo on pin 8
void melod(double p) {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second 
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = scaletime(1000)/noteDurations[thisNote];
    tone(8, melody[thisNote],noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * p;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}
