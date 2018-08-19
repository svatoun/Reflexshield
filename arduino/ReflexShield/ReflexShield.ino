/**
 * Copyright (c) 2016, svatopluk.dedic@gmail.com
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * I measured that analogRead() can be executed ~800 times in 100ms, which could give ~8kHz sampling frequency,
 * but because of input switches the effective max frequency will be much lower.
 * 
 * Algorith outline:
 * Measures ambient light on phototranzistors when IR LEDs are off, then measures light again with IR LEDs on. 
 * Averages several subsequent measures for low, then high.  If difference between "high" and "low" light value is 
 * above threshold the sensor is probably receiving reflected IR LED's light so it is detecting a vehicle above it.
 * 
 * A "match counter" is kept for each sensor. Match counter increases when the sensor is detecting the correct LED phase,
 * *decreases* if not. So fluctuations quickly decrease the match counter towards zero, while for detection to
 * be reported, the match counter must accumulate several subsequent positives.
 * 
 * After some defined period of time, the match counters are processed into occupancy detection bits and stored for
 * S88 bus handler.
 * 
 * Details:
 * The ADC is configured for free-running mode, which generates an interrupt after ADC conversion is done.
 * The interrupt routine fetches ADC value and accumulates it in "measureHigh" (LED on) and "measureLow" (LED off) 
 * arrays. Each input is sampled "inputADCConversions" times, then ADC is reconfigured for the next input.
 * After input changes, subsequent "discardMeasuresAfterChange" readings are discarded to allow internal capacitor 
 * to stabilize (this is instead of wait). "measureSamples" counter is incremented, so average value can be computed
 * from the accumulated measurements.
 * 
 * The ISR increments "readCycles" diagnostic variable.
 * 
 * The main loop waits for "sampleMillis" milliseconds (default = 7), then evaluates "measureHigh" and "measureLow"
 * (recorded in ADC ISR) - computes an average value, stores in "resultHigh" or "resultLow". When IR LED is on 
 * (= the second measurement phase), match counters are computed for each sensor.
 * 
 * After "measureInterval" (97 milliseconds), match counters propagate to "sensorStateBits" which is being read by S88
 * handler.
 * 
 * The other interrupts come from S88 clock/reset and causes bits to be promoted into S88 register or shifted.
 * 
 * When changing any variables used by ISRs, noInterrupts()/interrupts() must be called to protect ISRs from seeing
 * an inconsistent computation state.
 * 
 */

#include <Bounce2.h>
#include <EEPROM.h>

const int debug = 0;            // set to >0 to activate debug output on the serial console. The collector will use delays 200ms and will print stats each 2s
const int debugControl = 0;     // debug control commands
const int debugS88 = 0;
const int numChannels = 8;      // number of sensors used. Max 5 on Arduino UNO, 8 on Nano.

const int reductionLight = 250;   // the percentage will be reduced 4 times if the ambient light opens tranzistor above this level.

const int counterThreshold = 3;     // minimum sensor counter level to consider the sensor seeing the reflected LED light
const int defaultThreshold= 450;    // default sensor threshold for 'occupied'
const int occupiedThresholdReduction = 75;  // when occupied, the threshold is lowered to this percentage of the configured value.
const int defaultDebounce = 200;    // default debounce, when the sensor goes off

// The layout of the optoshield assumes, that HIGH on common LED control output will trigger a tranzistor to connect LEDs to GND.
// If experimenting with the Arduino alone on the breadboard, it would be better to trigger LED with LOW, connect LED to D12-GND 
// and the phototranzistor to +5V-Ax
const int ledOnValue  = HIGH;
const int ledOffValue  = (ledOnValue == HIGH ? LOW : HIGH);

const int BUTTON_PLUS     = 9;
const int BUTTON_MINUS    = 8;
const int BUTTON_NEXT     = 7;

const int longButtonPress = 500;       // long press, millseconds
const int configButtonPress = 2000;     // config button press, milliseconds
const int resetButtonPress = 5000;     // reset button press, milliseconds
const int calibrationTime = 10000;      // how long is the input measured during calibration
const long configIdleTimeExit = 60000; // after 60 sec of inactivity the configuration closes
 
const int LOAD_INT_0      = 2 ;        // 2 LOAD 0 int
const int CLOCK_INT_1     = 3 ;        // 3 CLOCK 1 int
const int DATA_IN         = 4 ;        // data in
const int DATA_OUT        = 5 ;        // data out
const int LED_SIGNAL      = 13;       // signal LED

const int LED_ACK         = 11;       // ACK LEd
const int LED_STATUS      = 10;       // ACK LEd

const int reportS88Loss = 1;          // will flash LED if S88 CLK signal is not present

const int inputADCConversions = 5;  // how many ADC conversions is done on a particular input before moving to the next one
const int sampleMillis = 7;         // Milliseconds between ADC collections
const int measureInterval = 97;     // Interval for measuring sensor inputs to produce yes/no occupation result

const int senseMax = 600;           // minimum sensitivity
const int senseMin = 100;           // maximum sensitivity

const int debounceMin = 0;      
const int debounceMax = 1000;

const int analogInputs[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
const int digitalOutputs[] = { 12, 12, 12, 12, 12, 12, 12, 12 };  // LEDs for individual detectors. Now controlled by single pin, all LEDs are lit/dark at the same time.
const int discardMeasuresAfterChange = 2; // how many ADC readings should be discarded after input channel change; this is instead of delay :) One reading = 104 usec.

const int eepromThresholdBase = 0x00;
const int eepromDebounceBase = 0x10;
const int eepromChecksum = 0x30;

// The OUTPUT; this is the S88 sensor value, A0 is mapped to bit 0. Bits for unused channels are set to 0.
volatile byte sensorStateBits;

int sensorThresholds[] = {
   defaultThreshold, defaultThreshold, defaultThreshold, defaultThreshold, defaultThreshold, defaultThreshold, defaultThreshold, defaultThreshold
};

int sensorDebounces[] = {
  defaultDebounce, defaultDebounce, defaultDebounce, defaultDebounce, defaultDebounce, defaultDebounce, defaultDebounce, defaultDebounce, 
};

long sensorLastUp[] = {
  -1, -1, -1, -1, -1, -1, -1, -1  
};

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

const int maxADCValue = 1024;
int mux;  // the ADCMUX value saved after initialization; will be used as a base for channel selection


// Variables access from the ADC interrupt routine. Before reading from the main code or other ISR,
// one MUST call noInterrupts() to avoid concurrent access.
volatile long measureLow[numChannels];      // inputs measured in 'low' state without LED; cummulative value
volatile long measureHigh[numChannels];     // inputs measured in 'high' state with LED on; cummulative value
volatile int measureSamples[numChannels];   // how many samples are accumulated in individual channel's input. Since the ADC conversion is driven by interrupt,
volatile int ledState;                      // the current LED light state; applies to all channels
volatile int adcConversionCounter;          // number of ADC conversion left for the current input

int resultLow[numChannels];   // averaged value for 'low' led state
int resultHigh[numChannels];  // averaged value for 'high' led state
int sensorCounters[numChannels];  // sense counters for each sensor

int calibrationMin = 1000;
int calibrationMax = -1;

// ---------- Variables used by ADC value collecting routing ---------------
int input;     // the current input number; cycles from 0 to numChannels - 1
int wasSwitch; // nonzero if input switch happened; discard one ADC reading after the switch

volatile long intCount;     // debug only: the number of ADC interrupts. 
volatile int readCycles;    // debug only: how many complete read cycles through all the sensors were perfomed since the data collection
int lastReadCycles;         // debug only: number of measure cycles used during the last ADC collection.
volatile long collectIntCount; // debug only: count of interrupts occuring in the last ADC collection period

long lastMillis;          // time of the last evaluation of occupancy
long lastDisplayMillis;   // time of the last diagnostic display on serial line
int collectDisplayCount = 1;
boolean sometimesDebug;

int configChannel = 0;
int cfgState;
long configLastCommand;

#define CONFIG_NONE 0           // no configuration in progress
#define CONFIG_INPUT_ID   1     // ID of the input being configured is transmitted
#define CONFIG_SENSE  2     // receiving commands for input configuration
#define CONFIG_DELAY 3     // signaling input config 
#define CONFIG_CALIBRATE_HIGH 4 // calibrating high sensitivity
#define CONFIG_CALIBRATE_LOW  5 // calibrating low sensitivity
#define CONFIG_CALIBRATE_WAIT  6 // calibrating low sensitivity


void startAnalogRead(int channel) {
  noInterrupts();
  ADMUX = (mux & ~0x07) | channel;
  ADCSRA |= (1 << ADSC);  // start ADC measurements 
  adcConversionCounter = inputADCConversions;
  interrupts();
}

void initialSignalBlink() {
  // LED off
  digitalWrite(LED_SIGNAL, LOW);

  // Signal that the code has started OK: {blink two times, wait 1 sec}, repeat 3x
  for (int sig = 0; sig < 3; sig++) {
    digitalWrite(LED_SIGNAL, HIGH);
    delay(50);
    digitalWrite(LED_SIGNAL, LOW);
    delay(300);
    digitalWrite(LED_SIGNAL, HIGH);
    delay(50);
    digitalWrite(LED_SIGNAL, LOW);
    delay(1000);
  }
}

void initializeS88Pins() {
  // Initialize S88 pins
  pinMode(LOAD_INT_0, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(LOAD_INT_0), loadInt, RISING) ;
  
  pinMode(CLOCK_INT_1, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(CLOCK_INT_1), clockInt, RISING) ;

  pinMode(DATA_IN, INPUT) ;
  
  pinMode(DATA_OUT, OUTPUT) ;
  digitalWrite(DATA_OUT, LOW) ;
}

void setup() {
  // put your setup code here, to run once:
  if (debug || debugS88 || debugControl) {
    Serial.begin(115200);
    Serial.println("Starting up...");
  }

  // DISABLE internal pull-ups for analog inputs, there's an external pull-down resistor.
  for (int i = 0; i < numChannels; i++) {
    pinMode(analogInputs[i], INPUT);
    digitalWrite(analogInputs[i], LOW);
  }

  // initialize digital (IR LED power) outputs
  for (int i = 0; i < sizeof(digitalOutputs)/sizeof(digitalOutputs[0]); i++) {
    pinMode(digitalOutputs[i], OUTPUT);
    digitalWrite(digitalOutputs[i], ledOffValue);
  }

  initializeS88Pins();

  // Signal LED
  pinMode(LED_SIGNAL, OUTPUT);
  initialSignalBlink();

  // ACK LED
  pinMode(LED_ACK, OUTPUT);
  digitalWrite(LED_ACK, LOW);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_STATUS, LOW);

  pinMode(BUTTON_PLUS, INPUT);
  pinMode(BUTTON_MINUS, INPUT);
  pinMode(BUTTON_NEXT, INPUT);
  // internal pullups
  digitalWrite(BUTTON_PLUS, HIGH);
  digitalWrite(BUTTON_MINUS, HIGH);
  digitalWrite(BUTTON_NEXT, HIGH);
  
  // --------------- Shamelessly copied from http://yaab-arduino.blogspot.cz/2015/02/fast-sampling-from-analog-input.html
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
  
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (01)
  ADMUX |= B01000000;
  
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
  
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
  
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00100000;
  
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;

  // Set the Prescaler to 32 (16000KHz/32 = 500KHz)
  ADCSRA = (ADCSRA & B11111000) | B00000101;
  
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;
  
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.

  // save the ADMUX value, will be combined with desired analog input later.
  mux = ADMUX;

  // start from input #0
  input = 0;
  sei();
  startAnalogRead(input);
  delay(10);
  initialLoadEEPROM();
}

// Interrupt service routine for the ADC completion
ISR(ADC_vect)
{
  int readLo = ADCL; // ADCL must be read first; reading ADCH will trigger next coversion.
  int readHi = ADCH; 
  if (wasSwitch) {
    wasSwitch--;
    // discard the first reading(s) after input switch. 
    startAnalogRead(input);
    return;
  }
  intCount++;
  int v = (readHi << 8) | readLo;
  if (ledState) {
    measureHigh[input] += v;
  } else {
    measureLow[input] += v;
  }
  measureSamples[input]++;
  
  if (adcConversionCounter--) {
    return;
  }
  // the proper number of conversions passed, advance the input.
  input = (input + 1) % numChannels;
  if (input == 0) {
    readCycles++;
  }
  wasSwitch = discardMeasuresAfterChange;
  startAnalogRead(input);
}

int errors; // accumulate cases where the samples were read although readCycles was not incremented
int lastCycle; // last cycle observed by the main loop.

int collectADC() {
  int cycles = readCycles - lastCycle;
  lastReadCycles = cycles;
  lastCycle = readCycles;
  if (cycles == 0) {
    if (debug) {
      Serial.println("No read cycles!");
    }
    return 0;
  }
  noInterrupts(); // prevent the ADC interrupt to fiddle with the longs.
  // compute averages
  if (sometimesDebug) {
    Serial.print(ledState ? "High: " : "Low:  ");
    Serial.print("Cycles: "); Serial.print(cycles); Serial.print(" ");
  }
  collectIntCount = intCount;
  intCount = 0;
  for (int i = 0; i < numChannels; i++) {
    int count = measureSamples[i];
    long sum = ledState ? measureHigh[i] : measureLow[i];
    int ave = sum / count;  // round up/down
    (ledState ? resultHigh : resultLow)[i] = ave;
    if (sometimesDebug) {
      Serial.print("#"); Serial.print(i); Serial.print(" = "); Serial.print(ave); Serial.print(", ");
    }

    if (ledState) {
      measureHigh[i] = 0;
    } else {
      measureLow[i] = 0;
    }
    measureSamples[i] = 0;
  }
  readCycles = lastCycle = 0;
  interrupts();
  if (sometimesDebug) {
    Serial.println();
  }
  return 1;
}

// update tracking counters for each sensor, when the LED is (currently) lit
void updateSensorCounters() {
    if (!ledState) {
      return;
    }
    for (int x = 0; x < numChannels; x++) {
        int res = resultHigh[x];

        if ((configChannel > -1) && (configChannel == x)) {
          if (calibrationMin > res) {
            calibrationMin = res;
          }
          if (calibrationMax < res) {
            calibrationMax = res;
          }
        }
        
        int base = resultLow[x];
        int range = base;

        int diff = resultHigh[x] - base;
        int comp = sensorThresholds[x];
        // This is ugly hack -- some kind of hysterezis; if a sensor detected previously,
        // much lesser threshold is applied to detect again.
        if ((sensorStateBits & (1 << x)) > 0) {
          comp = (comp * occupiedThresholdReduction) / 100;
        }

        if (diff > comp) {
          // the sensor received light above the base in the 'proper' time
          sensorCounters[x]++;
        } else {
          sensorCounters[x] --;
        }
        if (sometimesDebug) {
          Serial.print("C-"); Serial.print(x); 
          Serial.print(", Low = "); Serial.print(resultLow[x]);
          Serial.print(", High = "); Serial.print(resultHigh[x]);
          Serial.print(", Comp = "); Serial.print(comp);
          Serial.print(", Range = "); Serial.print(range);
//          Serial.print(", Thresh = "); Serial.print(thr);
          Serial.print(", CNT = "); Serial.print(sensorCounters[x]); Serial.println("");
        }
    }
    if (sometimesDebug) {
      Serial.println();
    }
}

int switchLEDState() {
  noInterrupts();
  if (sometimesDebug) {
    Serial.print(F("Led state was: ")); Serial.println(ledState);
  }
  ledState = ledState ? 0 : 1;
  for (int i = 0; i < numChannels; i++) {
    digitalWrite(digitalOutputs[i], ledState ? ledOnValue : ledOffValue);
  }
  if (sometimesDebug) {
    Serial.print(F("Switched LED ")); Serial.println(ledState);
  }
  interrupts();
  return ledState;
}

/**
 * Evaluates sensor state, builds the S88 response byte
 */
void updateSensorBits() {
  long t = millis();
  byte result = 0;
  noInterrupts();
  byte lastState = sensorStateBits;
  for (int i = 0; i < numChannels; i++) {
    int v = 1 << i;
    if (sensorCounters[i] > counterThreshold) {
      result |= v;
      sensorLastUp[i] = t;
    } else if ((lastState & (1 << i)) > 0) {
      long up = sensorLastUp[i];
      // if going down, wait at least the 'debounce' time before dropping the sensor bit from the result.
      if (up != -1 && ((t - up) < sensorDebounces[i])) {
        if (sometimesDebug) {
          Serial.print(F("Sensor debounced: ")); Serial.print(i); Serial.print(F(", time left: ")); Serial.println(t - sensorLastUp[i]);
        }
        result |= v;
      }
    }
    sensorCounters[i] = 0;
  }
  sensorStateBits = result;
  interrupts();
  // signal the status to the "control panel" LED
  if (configChannel >= 0) {
    int ledState = result & (1 << configChannel) ? HIGH : LOW;
    digitalWrite(LED_STATUS, ledState);
  } else {
    digitalWrite(LED_STATUS, LOW);
  }
}

long lastSignalMillis;
long  lastS88Millis;
boolean blinkState;

void handleSignalLed() {
  long current = millis();
  if (cfgState != CONFIG_NONE) {
    digitalWrite(LED_SIGNAL, LOW);
    return;
  }
  if (reportS88Loss) {
    if ((current - lastS88Millis > 1000)) {
      if (current - lastSignalMillis >= 500) {
        digitalWrite(LED_SIGNAL, blinkState ? HIGH : LOW);
        blinkState = !blinkState;
        lastSignalMillis = current;
      }
      return;
    }
  }
  if (sensorStateBits > 0) {
    digitalWrite(LED_SIGNAL, HIGH);
  } else {
    digitalWrite(LED_SIGNAL, LOW);
  }
}

void loop() {
  if (cfgState == CONFIG_NONE) {
    configChannel = -1;
  }
  sometimesDebug = 0;
  delay(sampleMillis);
  if (debug) {
    if (--collectDisplayCount == 0) {
      collectDisplayCount = 13;
      sometimesDebug = 1;
    }
  }
  if (collectADC()) {
    updateSensorCounters();
    int res = switchLEDState();
    delayMicroseconds(50);
  } else {
    errors++;
    if (debug) {
      Serial.print("Error");
    }
  }
    
  long curMillis = millis();
  if (curMillis - lastMillis >= measureInterval) {
    updateSensorBits(); 
    handleSignalLed();
    if (sometimesDebug) {
      Serial.print(" IntCount: ");  Serial.print(intCount); 
      Serial.print(" Errors: "); Serial.print(errors);
      Serial.print(" bit state = ");
      Serial.println(sensorStateBits);
      Serial.print("Last read cycles: "); Serial.println(lastReadCycles);
      lastDisplayMillis = millis();
    }
    lastMillis = curMillis;
  }
  handleButtons();
  handleAckLed();
  handleCalibration();
}

// -------------------- Configuration automaton ------------------------
Bounce plusButton = Bounce(BUTTON_PLUS, 20);
Bounce minusButton = Bounce(BUTTON_MINUS, 20);
Bounce nextButton = Bounce(BUTTON_NEXT, 20);

long plusButtonStart = -1;
long minusButtonStart = -1;
long nextButtonStart = -1;
int plusLen = -1;
int minusLen = -1;
int nextLen = -1;

const int blinkConfig[] = { 500, 200, 500, 200, 500, 200, 0 }; // fast blink 3x, enter configuration mode
const int blinkConfigEnd[] = { 500, 200, 100, 200, 100, 200, 0 }; // fast blink 3x, enter configuration mode
const int blinkShort[] = { 250, 250, 0 };   // blink once, short
const int blinkLong[] = { 1000, 250, 0 };   // blink once, very long
const int blinkTwice[] = { 250, 250, 250, 250, 0 };   
const int blinkThrice[] = { 250, 250, 250, 500, 250, 250, 250, 250, 0 };
const int blinkCalibrateLow[] = { 1000, 250, 250, 500, 0 };
const int blinkCalibrateCont[] =  { 1000, 250, 250, 250, 250, 500 , 0 };
const int blinkCalibrateEnd[] =  { 1000, 250, 1000, 250, 250, 500 , 0 };
const int blinkReset[] = { 1000, 250, 1000, 250, 1000, 250, 1000, 250, 0 };  // blink 4x, long; reset

// ========================= EEPROM functions ================================
int eepromWriteInt(int addr, int t, int& checksum) {
    checksum = checksum ^ t;
    EEPROM.update(addr++, (t & 0xff));
    EEPROM.update(addr++, (t >> 8) & 0xff);
    if (debugControl) {
      Serial.print(t & 0xff, HEX); Serial.print((t >> 8) & 0xff, HEX); Serial.print(" ");
    }
    return addr;
}

void writeEEPROM() {
  if (debugControl) {
    Serial.println("Save to EEPROM"); 
    Serial.println("Writing sensitivity");
  }
  int eeAddr = eepromThresholdBase;
  int check = 0;
  for (int i = 0; i < numChannels; i++) {
    int t = sensorThresholds[i];
    eeAddr = eepromWriteInt(eeAddr, t, check);
  }
  eeAddr = eepromDebounceBase;
  if (debugControl) {
    Serial.println("\nWriting debounces");
  }
  for (int i = 0; i < numChannels; i++) {
    int t = sensorDebounces[i];
    eeAddr = eepromWriteInt(eeAddr, t, check);
  }
  
  // checksum
  if (debugControl) {
    Serial.print(F("\nEEPROM checksum: ")); Serial.println(check & 0xff, HEX);
  }
  EEPROM.write(eepromChecksum, check & 0xff);
}

void initializeEEPROM() {
  if (debugControl) {
    Serial.println("Initializing EEPROM");
  }
  int eeAddr = eepromThresholdBase;
  int check = 0;
  for (int i = 0; i < numChannels; i++) {
    sensorThresholds[i] = defaultThreshold;
    sensorDebounces[i] = defaultDebounce;
  }
  writeEEPROM();
  
  // signal through main LED
  // LED off
  digitalWrite(LED_SIGNAL, LOW);
  // Signal that the code has started OK: {blink two times, wait 1 sec}, repeat 3x
  for (int sig = 0; sig < 3; sig++) {
    digitalWrite(LED_SIGNAL, HIGH);
    delay(1000);
    digitalWrite(LED_SIGNAL, LOW);
    delay(500);
  }
}

void resetEEPROM() {
  initializeEEPROM();
  initialLoadEEPROM();
  makeLedAck(&blinkReset[0]);
}

int readEepromInt(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr) + (EEPROM.read(addr + 1) << 8);
    addr += 2;
    checksum = checksum ^ v;
    Serial.print(v & 0xff, HEX); Serial.print((v >> 8) & 0xff, HEX); Serial.print(" ");
    if (v != 0) {
      allzero = false;
    }
    return v;
}

void initialLoadEEPROM() {
  if (debugControl) {
    Serial.println("Loading from EEPROM");
  }
  int thresholds[numChannels];
  int eeAddr = eepromThresholdBase;
  int check = 0;
  boolean allzero = true;
  for (int i = 0; i < numChannels; i++) {
      sensorThresholds[i] = readEepromInt(eeAddr, check, allzero);
  }
  eeAddr = eepromDebounceBase;
  for (int i = 0; i < numChannels; i++) {
      sensorDebounces[i] = readEepromInt(eeAddr, check, allzero);
  }
  int eeChecksumVal = EEPROM.read(eepromChecksum);
  if (allzero || ((check & 0xff) != eeChecksumVal)) {
    if (debugControl) {
      Serial.print(F("EEPROM checksum does not match: ")); Serial.print(check); Serial.print(" / "); Serial.println(eeChecksumVal);
    }
    initializeEEPROM();
  }
  if (debugControl) {
    Serial.println("Loading done.");
  }
}

const int* blinkPtr = NULL;
long blinkLastMillis;
byte pos = 0;
boolean ackLedState = false;
int pulseCount = 0;

void restoreChannel() {
  if (debugControl) {
    Serial.print(F("Reset to default channel ")); Serial.println(configChannel);
  }
  int eeAddr = eepromThresholdBase + 2 * configChannel;
  int v = EEPROM.read(eeAddr) + (EEPROM.read(eeAddr) << 8);
  sensorThresholds[configChannel] = v;

  eeAddr = eepromDebounceBase + 2 * configChannel;
  v = EEPROM.read(eeAddr) + (EEPROM.read(eeAddr) << 8);
  sensorDebounces[configChannel] = v;
  makeLedAck(&blinkThrice[0]);
}

void makeLedAck(const int *  ledSequence) {
    blinkPtr = ledSequence;
    pos = 0;
    ackLedState = 1;
    blinkLastMillis = millis();
    digitalWrite(LED_ACK, HIGH);
    if (debugControl) {
      Serial.print(F("LED ACK: ")); Serial.println(blinkPtr[pos]);
    }
}

void pulseChannelNumber() {
  if (debugControl) {
    Serial.print(F("ACK channel ID ")); Serial.println(configChannel);
  }
  pulseCount = configChannel;
  makeLedAck(&blinkShort[0]);
}

void handleAckLed() {
  if (blinkPtr == NULL) {
    return;
  }
  long t = millis();
  long l = t - blinkLastMillis;
  if (l < blinkPtr[pos]) {
    return;
  }
  blinkLastMillis = t;
  pos++;
  if (debugControl) {
    Serial.print(F("Next ACK time: ")); Serial.println(blinkPtr[pos]);
  }
  if (blinkPtr[pos] == 0) {
    ackLedState = 0;
    digitalWrite(LED_ACK, LOW);
    if (pulseCount > 0) {
      pulseCount--;
      makeLedAck(&blinkShort[0]);
    } else {
      if (debugControl) {
        Serial.println("ACK done");
      }
      blinkPtr = NULL;
      blinkLastMillis = -1;
    }
    return;
  }
  ackLedState = !ackLedState;
  digitalWrite(LED_ACK, ackLedState ? HIGH : LOW);
}

void resetButtons() {
  plusButtonStart = minusButtonStart = nextButtonStart = -1;
  plusLen = minusLen = nextLen = -1;
}

void handleButtons() {
  plusButton.update();
  minusButton.update();
  nextButton.update();

  long t = millis();
  if (plusButton.fell()) {
    if (debugControl) {
      Serial.println(F("Plus UP"));
    }
    plusButtonStart = t;
    plusLen = -1;
  }
  if (minusButton.fell()) {
    if (debugControl) {
      Serial.println(F("Minus UP"));
    }
    minusButtonStart = t;
    minusLen = -1;
  }
  if (nextButton.fell()) {
    if (debugControl) {
      Serial.println(F("Next UP"));
    }
    nextButtonStart = t;
    nextLen = -1;
  }
  
  if (plusButtonStart > 0 && plusButton.rose()) {
    plusLen = (t - plusButtonStart);
    if (debugControl) {
      Serial.println(F("Plus DOWN"));
    }
  }
  if (minusButtonStart > 0 && minusButton.rose()) {
    minusLen = (t - minusButtonStart);
    if (debugControl) {
      Serial.println(F("Minus DOWN"));
    }
  }
  if (nextButtonStart > 0 && nextButton.rose()) {
    nextLen = (t - nextButtonStart);
    if (debugControl) {
      Serial.println(F("Next DOWN"));
    }
  }
  if (nextButtonStart > 0 && nextLen == -1) {
    return;
  }
  if (plusButtonStart > 0 && plusLen == -1) {
    return;
  }
  if (minusButtonStart > 0 && minusLen == -1) {
    return;
  }
  if (nextLen == -1 && plusLen == -1 && minusLen == -1) {
    long m = millis();
    if ((cfgState != CONFIG_NONE) && ((m - configLastCommand) > configIdleTimeExit)) {
      exitConfiguration();
    }
    return;
  }
  if (debugControl) {
    Serial.print(F("Buttons: + = ")); Serial.print(plusLen); 
    Serial.print(F(", - = ")) ; Serial.print(minusLen);
    Serial.print(F(", next = ")); Serial.println(nextLen);
  }
  configLastCommand = millis();
  interpretButtons();
  resetButtons();
}

void maybeEnterConfiguration() {
  if (plusLen != -1 || minusLen != -1) {
    return;
  }
  if (nextLen > resetButtonPress) {
    // reinitialize EEPROM
    resetEEPROM();
    return;
  } else if (nextLen > configButtonPress) {
    if (debugControl) {
      Serial.println("Enter configuration");
    }
    cfgState = CONFIG_INPUT_ID;
    configChannel = 0;
    makeLedAck(&blinkConfig[0]);
    return;
  }
}

/*
 * Plus/Minus pressed alone increases/decreases the channel.
 * Next alone will select the channel
 */
void selectChannel() {
  if (plusLen > 0 && plusLen < longButtonPress) {
    if (minusLen != -1 || nextLen != -1) {
      return;
    }
    configChannel++;
  }
  if (minusLen > 0) {
    if (plusLen != -1 || nextLen != -1) {
      return;
    }
    if (minusLen > resetButtonPress) {
      restoreChannel();
      return;
    }
    configChannel--;
  }
  if (nextLen > 0 && nextLen < longButtonPress) {
    if (plusLen != -1 || minusLen != -1) {
      return;
    }
    if (debugControl) {
      Serial.print(F("Selected channel: ")); Serial.println(configChannel);
    }
    cfgState = CONFIG_SENSE;
    makeLedAck(&blinkLong[0]);
    return;
  }
  if (nextLen > longButtonPress) {
    if (debugControl) {
      Serial.print(F("Blinking channel number: ")); Serial.print(configChannel + 1);
      pulseChannelNumber();
    }
  }
  // cycle around
  configChannel = configChannel % numChannels;
  if (debugControl) {
    Serial.print(F("Change channel: ")); Serial.println(configChannel);
  }
  // plus and minus
  if (configChannel == 0) {
    makeLedAck(&blinkLong[0]);
  } else {
    makeLedAck(&blinkShort[0]);
  }
}

/**
 * Configures channel's debounce time. Short presses increase/decrease debounce time by 50ms. Long presses by 100ms. Reset will set it to
 */
void configChannelDebounce() {
  if (nextLen > 0) {
    if (plusLen != -1 || minusLen != -1) {
      return;
    }
    if (nextLen < longButtonPress) {
      writeEEPROM();
      makeLedAck(&blinkTwice[0]);
      if (debugControl) {
        Serial.print(F("Configuring debounce for channel ")); Serial.println(configChannel);
        Serial.print(F("Debounce val: ")); Serial.println(configChannel);
      }
      cfgState = CONFIG_INPUT_ID;
      return;
    }
    if (nextLen < configButtonPress) {
      // discard, return:
      if (debugControl) {
        Serial.print(F("Discard changes, load EEPROM"));
      }
      initialLoadEEPROM();
      makeLedAck(&blinkThrice[0]);
      return;
    }
  }

  int kind = 0;
  int diff = 0;
  if (plusLen > 0) {
    if (minusLen != -1 || nextLen != -1) {
      return;
    }
    if (plusLen > configButtonPress) {
      sensorThresholds[configChannel] = debounceMax;
      kind = 2;
    } else if (plusLen > longButtonPress) {
      diff = 100;
      kind = 1;
    } else {
      diff = 20;
    }
  }
  if (minusLen > 0) {
    if (plusLen != -1 || nextLen != -1) {
      return;
    }
    if (minusLen > resetButtonPress) {
      restoreChannel();
      return;
    } else if (minusLen > configButtonPress) {
      sensorThresholds[configChannel] = debounceMin;
      kind = 2;
    } else if (minusLen > longButtonPress) {
      diff = -100;
      kind = 1;
    } else {
      diff = -20;
    }
  }
  if (diff != 0) {
    int v = sensorDebounces[configChannel] + diff;
    int nv = v;
    if (v > debounceMax) {
      nv = debounceMax;
    } else if (v < debounceMin) {
      nv = debounceMin;
    }
    if (debugControl) {
      Serial.print(F("Debounce adjusted for: ")); Serial.println(configChannel);
      Serial.print(F(", diff=")); Serial.print(diff); Serial.print(F(", val=")); Serial.print(v); Serial.print(F(", bounded=")); Serial.print(nv);
    }
    if (nv == v) {
      makeLedAck(kind == 0 ? &blinkShort[0] : &blinkLong[0]);
    } else {
      makeLedAck(&blinkTwice[0]);
    }
    sensorDebounces[configChannel] = nv;
  } else if (kind == 2) {
    if (debugControl) {
      Serial.print(F("Sense reset to min/max: ")); Serial.print(sensorDebounces[configChannel]);
    }
    makeLedAck(&blinkThrice[0]);
  }
}

void endCalibrateSensitivity() {
  int diff = calibrationMax - calibrationMin;
  int bottom = (calibrationMin + (diff * 15) / 100);    // add 15% to low
  sensorThresholds[configChannel] = bottom;
  makeLedAck(&blinkThrice[0]);
}

long calibrationStart = -1;
int emptyHigh;
int emptyLow;

void startCalibration() {
  cfgState = CONFIG_CALIBRATE_LOW;
  makeLedAck(&blinkCalibrateLow[0]);
  setupCalibration();
}

void setupCalibration() {
  calibrationMax = -1;
  calibrationMin = 1000;
  calibrationStart = millis();
  if (debugControl) {
    Serial.println("Calibration started");
  }
}

void handleCalibration() {
  switch (cfgState) {
    case CONFIG_CALIBRATE_LOW: {
      long m = millis();
      if ((calibrationStart > -1) && 
          ((m - calibrationStart) > calibrationTime)) {
        makeLedAck(&blinkCalibrateCont[0]);
        calibrationStart = -1;
        emptyHigh = calibrationMax;
        emptyLow = calibrationMin;
        if (debugControl) {
          Serial.print(F("Calibration: empty read, low = ")); Serial.print(emptyLow); Serial.print(F(", high = ")); Serial.println(emptyHigh);
        }
      }
      return;
    }
    case CONFIG_CALIBRATE_HIGH:  
      long m = millis();
      if (calibrationStart > -1 && 
          ((m - calibrationStart) > calibrationTime)) {
        makeLedAck(&blinkCalibrateEnd[0]);
        calibrationStart = -1;
        cfgState = CONFIG_SENSE;
        computeSensitivity();
        return;
      }
  }
}

void computeSensitivity() {
   int d = calibrationMax - calibrationMin;
   Serial.print(F("Calibration: occupied read, low = ")); Serial.print(calibrationMin); Serial.print(F(", high = ")); Serial.print(calibrationMax); Serial.print(F(", diff = ")); Serial.println(d);
   int c = max(0, calibrationMin - ((d * 20) / 100)); // 20% less - tolerance
   int m = emptyHigh + ((emptyHigh - emptyLow) * 40) / 100; // + 40% more reflexivity

   Serial.print(F("Occupied from: ")); Serial.print(c); Serial.print(F(", empty max: ")); Serial.println(m);
   int v = max(c, m);
   Serial.print(F("Sensor threshold: ")); Serial.println(v);
   sensorThresholds[configChannel] = v;
   
}

/**
 * Configures channel's sensitivity. Short press of +- will increase/decrease by 1. Long press will inc/dec by 20. Config press will set the sensitivity to senseMax or senseMin.
 * Short "next" will confirm and advance to hysteresis threshold. Long "next" will cancel, returning to channel selection.
 */
void configChannelSense() {
  if (nextLen > 0) {
    if (nextLen > configButtonPress) {
        startCalibration();
        return;
    }
    if (plusLen != -1 || minusLen != -1) {
      return;
    }
    if (nextLen < longButtonPress) {
      writeEEPROM();
      makeLedAck(&blinkTwice[0]);
      if (debugControl) {
        Serial.print(F("Configuring delay for channel ")); Serial.println(configChannel);
      }
      cfgState = CONFIG_DELAY;
      return;
    }
    if (nextLen < configButtonPress) {
      // discard, return:
      if (debugControl) {
        Serial.print(F("Discard changes, load EEPROM"));
      }
      initialLoadEEPROM();
      makeLedAck(&blinkThrice[0]);
      return;
    }
  }
  
  int kind = 0;
  int diff = 0;
  if (plusLen > 0) {
    if (minusLen != -1 || nextLen != -1) {
      return;
    }
    if (plusLen > configButtonPress) {
      sensorThresholds[configChannel] = senseMax;
      kind = 2;
    } else if (plusLen > longButtonPress) {
      diff = -20;
      kind = 1;
    } else {
      diff = -1;
    }
  }
  if (minusLen > 0) {
    if (plusLen != -1 || nextLen != -1) {
      return;
    }
    if (minusLen > resetButtonPress) {
      restoreChannel();
      return;
    } else if (minusLen > configButtonPress) {
      sensorThresholds[configChannel] = senseMin;
      kind = 2;
    } else if (minusLen > longButtonPress) {
      diff = 20;
      kind = 1;
    } else {
      diff = 1;
    }
  }
  if (diff != 0) {
    int v = sensorThresholds[configChannel] + diff;
    int nv = v;
    if (v > senseMax) {
      nv = senseMax;
    } else if (v < senseMin) {
      nv = senseMin;
    }
    if (debugControl) {
      Serial.print(F("Sense adjusted for: ")); Serial.println(configChannel);
      Serial.print(F(", diff=")); Serial.print(diff); Serial.print(F(", val=")); Serial.print(v); Serial.print(F(", bounded=")); Serial.print(nv);
    }
    if (nv == v) {
      makeLedAck(kind == 0 ? &blinkShort[0] : &blinkLong[0]);
    } else {
      makeLedAck(&blinkTwice[0]);
    }
    sensorThresholds[configChannel] = nv;
  } else if (kind == 2) {
    if (debugControl) {
      Serial.print(F("Sense reset to min/max: ")); Serial.print(sensorThresholds[configChannel]);
    }
    makeLedAck(&blinkThrice[0]);
  }
}

void exitConfiguration() {
    cfgState = CONFIG_NONE;
    if (debugControl) {
      Serial.println(F("Exited configuration"));
    }
    initialLoadEEPROM();
    makeLedAck(&blinkConfigEnd[0]);
    configChannel = -1;
}

void interpretButtons() {
  // next pressed for "reset" duration ALONE triggers EEPROM reset and reload, terminates configuration if opened.
  if ((nextLen > resetButtonPress) && (plusLen == -1) && (minusLen == -1)) {
    // reinitialize EEPROM
    resetEEPROM();
    cfgState = CONFIG_NONE;
    return;
  }
  if ((cfgState != CONFIG_NONE) && (nextLen > configButtonPress) && (plusLen == -1) && (minusLen == -1)) {
    exitConfiguration();
    return;
  }

  switch (cfgState) {
    case CONFIG_NONE:
      maybeEnterConfiguration();
      return;
    case CONFIG_INPUT_ID:
      selectChannel();
      return;
    case CONFIG_SENSE:
      configChannelSense();
      return;
    case CONFIG_DELAY:
      configChannelDebounce();
      return;
    case CONFIG_CALIBRATE_LOW:
      if (calibrationStart == -1 && nextLen > 0) {
        if (debugControl) {
          Serial.println(F("Performing occupied calibration"));
        }
        makeLedAck(&blinkTwice[0]);
        cfgState = CONFIG_CALIBRATE_HIGH;
        calibrationStart = millis();
      }
      return;
  }
}

// -------------------- S88 Section ----------------------------
// Shamelessly copied from https://sites.google.com/site/sidloweb/elektrika/s88-ir-detektor
// Copyright (c) Sidlo

byte data = 0 ;                   // data byte
int bitCounter = 0 ;              // bit counter

/***************************************************************************
 * Interrupt 0 LOAD.
 */
void loadInt() {
  data = sensorStateBits ;
  bitCounter = 0 ;
  lastS88Millis = millis();
}

/***************************************************************************
 * Interrupt 1 CLOCK.
 */
void clockInt() {
  int xferBit = (bitCounter++) % 8;
  digitalWrite(DATA_OUT, bitRead(data, xferBit)) ;

  // read input
  int x = digitalRead(DATA_IN);
  bitWrite(data, xferBit, (x > 0 ? 1 : 0));

  if (debugS88) {
    if ((bitCounter  % 8) == 0 && bitCounter > 7 && data != 0) {
      Serial.print(F("S88 read byte: ")); Serial.print(bitCounter / 8 + 65); Serial.print(F(" = "));
      Serial.println(data);
    }
  }
}


