/* BFO - Modified Minima's Arduino Sketch to setup BFO from another Arduino + Si570
 * Radiono - The Minima's Main Arduino Sketch
 * Copyright (C) 2013 Ashar Farhan
 * BFO mod 2016 Dimitar Pavlov (LZ1DPN)
 * 
 *  BAND_UL digital pin 6 - digital read pinout from arduino vfo (use wire from 1st arduino vfo to bfo relay)
 *  OPER_MODE digital pin 4 - see google - arduino digital read button - manual switch operation mode - CW/SSB
 *  (+5v)-(pin1 button pin2)+(pin4 arduino)+(R 10kOm)-(gnd)  
 */

#define __ASSERT_USE_STDERR
#include <assert.h>

/*
 * Wire is only used from the Si570 module but we need to list it here so that
 * the Arduino environment knows we need it.
 *
 * need to connect (2) digital pin from VFO aduino to (6) digital pin from BFO arduino to control LSB/USB IF_FREQ
 * All other pinouts set like VFO arduino. Not need tuning Potenciometer, and other exteriors wired things, only switch Buton to manual switch SSB/CW.
 *
 */

#include <Wire.h>
#include <LiquidCrystal.h>

#include <avr/io.h>
#include "Si570.h"
#include "debug.h"

#define RADIONO_VERSION "0.4"
#define RADIONO_BFO_VERSION "1.0"
#define SI570_I2C_ADDRESS 0x55

// When RUN_TESTS is 1, the Radiono will automatically do some software testing when it starts.
// Please note, that those are not hardware tests! - Comment this line to save some space.
#define RUN_TESTS 1

// my FILTER center frequency is 19999400,
//unsigned long CENTER_FREQ =  19996500L;
//unsigned long CENTER_FREQ2 = 19996500L; 
//unsigned long SSB_R = 4500L;
//unsigned long CW_R = 300L;

unsigned long BFO_USB = 19994000L;
unsigned long BFO_LSB = 19999000L;
unsigned long BFO_CW = 19999000L;

//unsigned long BFO_USB = (CENTER_FREQ - SSB_R);  // 19997000 L
//unsigned long BFO_LSB = (CENTER_FREQ + SSB_R);  // 19995000 L
//unsigned long BFO_CW = (CENTER_FREQ + CW_R);

unsigned long CW_ON = 0;
unsigned long frequency = BFO_LSB;  // default 14 mh ssb

unsigned long vfoA = BFO_USB;
unsigned long vfoB = BFO_LSB;
unsigned long ritA = BFO_USB;
unsigned long ritB = BFO_LSB;

// vars to BAND_UL (LOW/UP - State digital control set freq for BFO from VFO to switch USB/LSB from VFO - first arduino
int inState = 0;
int inState22 = 0;
int lastInState = 0;
int lastInState22 = 0;
// end BAND_UL vars

Si570 *vfo;
LiquidCrystal lcd(13, 12, 11, 10, 9, 8);

int count = 0;
char b[20], c[20], printBuff[32];

/* tuning pot stuff */
unsigned char refreshDisplay = 0;
unsigned int stepSize = 100;
int tuningPosition = 0;
unsigned char locked = 0; //the tuning can be locked: wait until it goes into dead-zone before unlocking it

/* the digital controls */

#define BAND_UL (6)
#define OPER_MODE (4)
#define FBUTTON (A3)
#define ANALOG_TUNING (A2)
#define VFO_A 0
#define VFO_B 1

char inTx = 0;
char keyDown = 0;
char isLSB = 0;
char isRIT = 0;
char vfoActive = VFO_A;
/* modes */
unsigned char isManual = 1;
unsigned ritOn = 0;

/* dds ddschip(DDS9850, 5, 6, 7, 125000000LL); */

/* display routines */
void printLine1(char const *c){
  if (strcmp(c, printBuff)){
		lcd.setCursor(0, 0);
		lcd.print(c);
		strcpy(printBuff, c);
		count++;
    }
}

void printLine2(char const *c){
	lcd.setCursor(0, 1);
	lcd.print(c);
}

void displayFrequency(unsigned long f){
	int mhz, khz, hz;

	mhz = f / 1000000l;
	khz = (f % 1000000l)/1000;
	hz = f % 1000l;
	sprintf(b, "[%02d.%03d.%03d]", mhz, khz, hz);
	printLine1(b);
}

void updateDisplay(){
	char const *vfoStatus[] = { "ERR", "RDY", "BIG", "SML" };

	sprintf(b, "%08ld", frequency);	
	sprintf(c, "%s:%.2s.%.5s %s", vfoActive == VFO_A ? "USB" : "LSB" , b,  b+2, CW_ON ? " CW" : "SSB");
	printLine1(c);
	sprintf(c, "%s%s", "LZ1DPN  stat:", vfoStatus[vfo->status]);
	printLine2(c);
}

void setup() {
  // Initialize the Serial port so that we can use it for debugging
	Serial.begin(115200);
	debug("Start RadionoBFO: %s", RADIONO_BFO_VERSION);
	lcd.begin(16, 2);

#ifdef RUN_TESTS
	run_tests();
#endif

  printBuff[0] = 0;
  printLine1("RadionoBFO ");
  lcd.print(RADIONO_BFO_VERSION);

  // The library automatically reads the factory calibration settings of your Si570
  // but it needs to know for what frequency it was calibrated for.
  // Looks like most HAM Si570 are calibrated for 56.320 Mhz.
  // If yours was calibrated for another frequency, you need to change that here
  vfo = new Si570(SI570_I2C_ADDRESS, 56320000);

  if (vfo->status == SI570_ERROR) {
		// The Si570 is unreachable. Show an error for 3 seconds and continue.
		printLine2("Si570 comm error");
		delay(3000);
    }

  // This will print some debugging info to the serial console.
  vfo->debugSi570();

  //set the initial frequency
  vfo->setFrequency(26150000L);

  //set up the pins
  
  pinMode(BAND_UL, INPUT);  // BAND U/L to switch SIDEBAND - USB/LSB
  pinMode(OPER_MODE, INPUT);  // oper mode button 

}

void readTuningPot(){
    tuningPosition = analogRead(2) - 512;
}

void checkTuning(){

  if (-50 < tuningPosition && tuningPosition < 50){
		//we are in the middle, so, let go of the lock
		if (locked) {
			locked = 0;
		}
		delay(50);
		return;
    }

  //if the tuning is locked and we are outside the safe band, then we don't move the freq.
  if (locked) {
		return;
	}
	
  //dead region between -50 and 50
  if (100 < tuningPosition){
    if (tuningPosition < 100)
      frequency += 3;
    else if (tuningPosition < 150)
      frequency += 10;
    else if (tuningPosition < 200)
      frequency += 30;
    else if (tuningPosition < 250)
      frequency += 100;
    else if (tuningPosition < 300)
      frequency += 300;
    else if (tuningPosition < 350)
      frequency += 1000;
    else if (tuningPosition < 400)
      frequency += 3000;
    else if (tuningPosition < 450){
      frequency += 100000;
      updateDisplay();
      delay(300);
    }
    else if (tuningPosition < 500){
      frequency += 1000000;
      updateDisplay();
      delay(300);
    }
  }

  if (-100 > tuningPosition){
    if (tuningPosition > -100)
      frequency -= 3;
    else if (tuningPosition > -150)
      frequency -= 10;
    else if (tuningPosition > -200)
      frequency -= 30;
    else if (tuningPosition > -250)
      frequency -= 100;
    else if (tuningPosition > -300)
      frequency -= 300;
    else if (tuningPosition > -350)
      frequency -= 1000;
    else if (tuningPosition > -400)
      frequency -= 3000;
    else if (tuningPosition > -450){
      frequency -= 100000;
      updateDisplay();
      delay(300);
    }
    else if (tuningPosition > -500){
      frequency -= 1000000;
      updateDisplay();
      delay(300);
    }
  }
  delay(50);
  refreshDisplay++;
}

void loop(){

  readTuningPot();  // keep this for tests and tuning if need 
  checkTuning();    // keep this for tests and tuning if need
    
  vfo->setFrequency(frequency);
  Serial.println(frequency);   // debug bfo freq in serial console if needs 
  
  if (refreshDisplay){
	updateDisplay();
	refreshDisplay = 0;
    }

// read band freq <> 10 mhz for lsb/usb and set bfo freq 
   
inState = digitalRead(BAND_UL);  
  if (inState != lastInState){
        if (inState == HIGH) {
	  vfoActive = VFO_B;
	  frequency = vfoB;
          }
	else {
	  vfoActive = VFO_A;
	  frequency = vfoA;
	  }

 //       Serial.println(frequency);   // print freq on serial console for debug 
        refreshDisplay++;
  
        if (refreshDisplay){
            updateDisplay();
            refreshDisplay = 0;
            }
   }
   lastInState = inState;

 // check button and switch bfo freq oper. mode - ssb/cw

inState22 = digitalRead(OPER_MODE);
  if (inState22 != lastInState22){
        if (inState22 == HIGH) {
              if (CW_ON == 0) {
                      CW_ON = 1;
                      vfoA=BFO_CW;
		              vfoB=BFO_CW;
              }
              else {
                      CW_ON = 0;
                      vfoA=BFO_USB;
		              vfoB=BFO_LSB;
              }
        refreshDisplay = 1;      
        }
       
      if (lastInState == HIGH) {
		vfoActive = VFO_B;
		frequency = vfoB;                
      }
      else {
		vfoActive = VFO_A;
		frequency = vfoA;
      }

  }
  lastInState22 = inState22;
  
  if (refreshDisplay){
        updateDisplay();
        refreshDisplay = 0;
    }
  
  //end oper mode

// end main loop

}   

// end main loop  


#ifdef RUN_TESTS

bool run_tests() {
  /* Those tests check that the Si570 libary is able to understand the
   * register values provided and do the required math with them.
   */
  // Testing for thomas - si570
  {
    uint8_t registers[] = { 0xe1, 0xc2, 0xb5, 0x7c, 0x77, 0x70 };
    vfo = new Si570(registers, 56320000);
    assert(vfo->getFreqXtal() == 114347712);
    delete(vfo);
  }

  // Testing Jerry - si570
  {
    uint8_t registers[] = { 0xe1, 0xc2, 0xb6, 0x36, 0xbf, 0x42 };
    vfo = new Si570(registers, 56320000);
    assert(vfo->getFreqXtal() == 114227856);
    delete(vfo);
  }

  Serial.println("Tests successful!");
  return true;
}

// handle diagnostic informations given by assertion and abort program execution:
void __assert(const char *__func, const char *__file, int __lineno, const char *__sexp) {
  debug("ASSERT FAILED - %s (%s:%i): %s", __func, __file, __lineno, __sexp);
  Serial.flush();
  // Show something on the screen
  lcd.setCursor(0, 0);
  lcd.print("OOPS ");
  lcd.print(__file);
  lcd.setCursor(0, 1);
  lcd.print("Line: ");
  lcd.print(__lineno);
  // abort program execution.
  abort();
}

#endif

