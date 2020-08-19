/**
 * Copyright (c) 2016, 2020, svatopluk.dedic@gmail.com
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
 * Ver 1.1: In order to reduce power consumption (initially: 40mA * 8 LEDs = 0,32A !! per detector board), the board
 * was changed to light just 2 leds at a time, through ULN2003 transistor array. 4 sensors are read during each cycle:
 * the 2 that are lit at the moment, and the 2 sensors that HAVE been lit during the previous cycle.
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
#include "Common.h"
#include "Settings.h"

#define STARTUP_MSG "ReflexShield (c) Belgarat@klfree.net, v. 1.2, 9/2020"

const int debug = 1;            // set to >0 to activate debug output on the serial console. The collector will use delays 200ms and will print stats each 2s
const int debugSensors = 1;
const int debugLow = 0;
const int debugControl = 1;     // debug control commands
const int debugMgmt = 1;
const int debugS88 = 0;
const int numChannels = 8;      // number of sensors used. Max 5 on Arduino UNO, 8 on Nano.

const int reductionLight = 250;   // the percentage will be reduced 4 times if the ambient light opens tranzistor above this level.

const int counterThreshold = 3;     // minimum sensor counter level to consider the sensor seeing the reflected LED light
const int defaultThreshold= 450;    // default sensor threshold for 'occupied'
const int occupiedThresholdReduction = 65;  // when occupied, the threshold is lowered to this percentage of the configured value.
const int defaultDebounce = 200;    // default debounce, when the sensor goes off

// The layout of the optoshield assumes, that HIGH on common LED control output will trigger a tranzistor to connect LEDs to GND.
// If experimenting with the Arduino alone on the breadboard, it would be better to trigger LED with LOW, connect LED to D12-GND 
// and the phototranzistor to +5V-Ax
const int ledOnValue  = HIGH;
const int ledOffValue  = (ledOnValue == HIGH ? LOW : HIGH);

const int BUTTON_PLUS     = 8;
const int BUTTON_MINUS    = 8;
const int BUTTON_NEXT     = 8;

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

const int LED_ACK         = 7;       // ACK LEd
const int LED_STATUS      = 6;       // ACK LEd

const int reportS88Loss = 1;          // will flash LED if S88 CLK signal is not present

const int inputADCConversions = 5;  // how many ADC conversions is done on a particular input before moving to the next one
const int sampleMillis = 7;         // Milliseconds between ADC collections
const int measureInterval = 97;     // Interval for measuring sensor inputs to produce yes/no occupation result

const int senseMax = 600;           // minimum sensitivity
const int senseMin = 100;           // maximum sensitivity

const int debounceMin = 0;      
const int debounceMax = 1000;

const int analogInputs[] = { A0, A1, A2, A3, A4, A5, A6, A7 };
const int digitalOutputs[] = { 12, 12, 11, 11, 9, 9, 10, 10};  // LEDs for individual detectors. Now controlled by single pin, all LEDs are lit/dark at the same time.
const int discardMeasuresAfterChange = 2; // how many ADC readings should be discarded after input channel change; this is instead of delay :) One reading = 104 usec.

const int eepromThresholdBase = 0x00;
const int eepromDebounceBase = 0x10;
const int eepromInvertBase = 0x20;
const int eepromChecksum = 0x30;

const int ledSlotsCount = 4;
const int ledSlotSize = 2;
const short ledSlots[ledSlotsCount][ledSlotSize] = {
  { 0, 1 },
  { 2, 3 },
  { 4, 5 },
  { 6, 7 }
};

extern void (* charModeCallback)(char);

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

byte sensorInvert = 0x00;

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
volatile short scanInputs[numChannels + ledSlotSize + 1] = { -1 };
volatile boolean startNextCycle = false;
volatile boolean scanningDisabled = true;

const int scanInputsCount = (sizeof(scanInputs) / sizeof(scanInputs[0]));

short enabledChannels = 0xff;

int resultLow[numChannels];   // averaged value for 'low' led state
int resultHigh[numChannels];  // averaged value for 'high' led state
int sensorCounters[numChannels];  // sense counters for each sensor

int calibrationMin = 1000;
int calibrationMax = -1;

// ---------- Variables used by ADC value collecting routing ---------------
int input;     // the current input number; cycles from 0 to numChannels - 1
int wasSwitch; // nonzero if input switch happened; discard one ADC reading after the switch
boolean intrLedState; // copy of the LED state for the interrupt routine

volatile long intCount;     // debug only: the number of ADC interrupts. 
volatile int readCycles;    // debug only: how many complete read cycles through all the sensors were perfomed since the data collection
int lastReadCycles;         // debug only: number of measure cycles used during the last ADC collection.
volatile long collectIntCount; // debug only: count of interrupts occuring in the last ADC collection period

long lastMillis;          // time of the last evaluation of occupancy
long lastDisplayMillis;   // time of the last diagnostic display on serial line
int collectDisplayCount = 1;
boolean sometimesDebug;

int configChannel = -1;
int cfgState;
long configLastCommand;

#define CONFIG_NONE 0           // no configuration in progress
#define CONFIG_INPUT_ID   1     // ID of the input being configured is transmitted
#define CONFIG_SENSE  2     // receiving commands for input configuration
#define CONFIG_DELAY 3     // signaling input config 
#define CONFIG_CALIBRATE_HIGH 4 // calibrating high sensitivity
#define CONFIG_CALIBRATE_LOW  5 // calibrating low sensitivity
#define CONFIG_CALIBRATE_WAIT  6 // calibrating low sensitivity
#define CONFIG_CALIBRATE_DEBOUNCE 7

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
  Serial.begin(115200);
  Serial.println(STARTUP_MSG);
  Serial.println("Starting up...");

  // DISABLE internal pull-ups for analog inputs, there's an external pull-down resistor.
  for (int i = 0; i < numChannels; i++) {
    int pno = analogInputs[i];
    pinMode(pno, INPUT);
    digitalWrite(pno, LOW);
  }

  // initialize digital (IR LED power) outputs
  for (int i = 0; i < sizeof(digitalOutputs)/sizeof(digitalOutputs[0]); i++) {
    pinMode(digitalOutputs[i], OUTPUT);
    digitalWrite(digitalOutputs[i], ledOffValue);
  }

  setupTerminal();

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
  configureScanInputs();
  
  sei();
  startAnalogRead(input);
  delay(10);

  initTerminal();
  initialLoadEEPROM();

  registerLineCommand("SAV", &commandSave);
  registerLineCommand("DMP", &commandDump);
  registerLineCommand("INF", &commandInfo);
  registerLineCommand("CAL", &commandCalibrate);
  registerLineCommand("MON", &commandMonitor);
  registerLineCommand("CAD", &commandCalibrateDebounce);
  registerLineCommand("SEN", &commandSensitivity);

}

// Interrupt service routine for the ADC completion
ISR(ADC_vect)
{
  // the proper number of conversions passed, advance the input.
  int prevInput = scanInputs[input];
  
  if (startNextCycle) {
    initScanCycle();
    prevInput = -1;
  } else if (prevInput >= 0) {
    int readLo = ADCL; // ADCL must be read first; reading ADCH will trigger next coversion.
    int readHi = ADCH; 
    if (wasSwitch) {
      wasSwitch--;
      // discard the first reading(s) after input switch. 
      startAnalogRead(prevInput);
      return;
    }
    intCount++;
    int v = (readHi << 8) | readLo;
    if (ledState) {
      measureHigh[prevInput] += v;
    } else {
      measureLow[prevInput] += v;
    }
    measureSamples[prevInput]++;
    
    if (adcConversionCounter--) {
      return;
    }
    input = input + 1;
  }
  
  if ((input >= scanInputsCount) || (scanInputs[input] < 0)) {
    input = 0;
    readCycles++;
  }

  int nextInput = scanInputs[input];

  changeLeds(prevInput, nextInput);
  
  wasSwitch = discardMeasuresAfterChange;
  // also enables interrupts.
  if (nextInput >= 0) {
    scanningDisabled = false;
    startAnalogRead(nextInput);
  } else {
    scanningDisabled = true;
  }
}

void initScanCycle() {
    input = 0;
    for (short i = 0; i < numChannels; i++) {
      digitalWrite(digitalOutputs[i], LOW);
    }
    intrLedState = ledState;
}

void changeLeds(int prevInput, int nextInput) {
  if (debugLow) {
    Serial.print("prev: "); Serial.print(prevInput); Serial.print(", next: "); Serial.print(nextInput); Serial.print(" input: "); Serial.println(input);
  }
  if (nextInput < 0) {
//    digitalWrite(prevInput, LOW/);
    return;
  }
  short ledNext = digitalOutputs[nextInput];
  
  if (prevInput >= 0) {
    short ledPrev = digitalOutputs[prevInput];
    if (ledPrev == ledNext) {
      return;
    }
    if (debugLow) {
      Serial.print("OFF: "); Serial.print(ledPrev);
    }
//    digitalWrite(ledPrev, LOW);/
  }
  if (debugLow) {
    Serial.print(" ON: "); Serial.print(ledNext);
  }
//  digitalWrite(ledNext, intrLedState ?/ HIGH : LOW);
  if (debugLow) {
    Serial.println();
  }
}

int errors; // accumulate cases where the samples were read although readCycles was not incremented
int lastCycle; // last cycle observed by the main loop.

void configureScanInputs() {
  noInterrupts();
  int mask = 0x01;
  int index = 0;

  int alreadyEnabled = 0x00;
  for (int i = 0; i < numChannels; i++) {
    boolean en = (enabledChannels & mask) > 0;
    if (!en) {
      measureSamples[i] = 0;
      continue;
    }
    if ((alreadyEnabled & (1 << i)) > 0) {
      continue;
    }
    boolean c = true;
    for (int slot = 0; c && (slot < ledSlotsCount); slot++) {
      for (int x = 0; c && (x < ledSlotSize); x++) {
        if (ledSlots[slot][x] == i) {

          for (int cp = 0; cp < ledSlotSize; cp++) {
            int l = ledSlots[slot][cp];
            if (l == -1) {
              break;
            }
            scanInputs[index++] = l;
            alreadyEnabled = (1 << l);
          }
          c = false;
        }
      }
    }
  }
  scanInputs[index++] = -1;
  if (debugMgmt) {
    Serial.print("Scanning channels: ");
    for (int i = 0; i < scanInputsCount; i++) {
      Serial.print(scanInputs[i]); Serial.print(", ");
    }
    Serial.println();
  }
}

int collectADC() {
  int cycles = readCycles - lastCycle;
  lastReadCycles = cycles;
  lastCycle = readCycles;
  if (cycles == 0) {
    if (debugSensors) {
      Serial.println(F("No read cycles!"));
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
    if (count > 0) {
      (ledState ? resultHigh : resultLow)[i] = ave;
      if (sometimesDebug) {
        Serial.print("#"); Serial.print(i); Serial.print(" = "); Serial.print(ave); Serial.print(", ");
      }
    }
    if (ledState) {
      measureHigh[i] = 0;
    } else {
      measureLow[i] = 0;
    }
    measureSamples[i] = 0;
  }
  readCycles = lastCycle = 0;
//  configureScanInputs();
//  startNextCycle = true;
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

void switchLEDState() {
  noInterrupts();
  if (sometimesDebug) {
    Serial.print(F("Led state was: ")); Serial.println(ledState);
  }
  ledState = ledState ? 0 : 1;
  if (sometimesDebug) {
    Serial.print(F("Switched LED ")); Serial.println(ledState);
  }
  interrupts();
}

volatile long maxConfigSensorDebounce = 0;
long configSensorDebounce = 0;

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
    } else {
      if ((lastState & (1 << i)) > 0) {
        long up = sensorLastUp[i];
        long diff = (t - up);
        if (i == configChannel) {
          configSensorDebounce = diff;
          if (maxConfigSensorDebounce < configSensorDebounce) {
            maxConfigSensorDebounce = configSensorDebounce;
          }
        }
        // if going down, wait at least the 'debounce' time before dropping the sensor bit from the result.
        if (up != -1 && (diff < sensorDebounces[i])) {
          if (sometimesDebug) {
            Serial.print(F("Sensor debounced: ")); Serial.print(i); Serial.print(F(", time left: ")); Serial.println(t - sensorLastUp[i]);
          }
          result |= v;
        }
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
  if (debugSensors) {
    if (--collectDisplayCount == 0) {
      collectDisplayCount = 13;
      sometimesDebug = 1;
    }
  }
  if (collectADC()) {
    updateSensorCounters();
    switchLEDState();
    delayMicroseconds(50);
  } else {
    errors++;
    if (debugSensors) {
      Serial.print("Error");
    }
  }
    
  long curMillis = millis();
  if (curMillis - lastMillis >= measureInterval) {
    updateSensorBits(); 
    handleSignalLed();
    if (sometimesDebug) {
      Serial.print(" IntCount: ");  Serial.print(collectIntCount); 
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
  handleMonitor();
  processTerminal();
}

// ========================= EEPROM functions ================================
int eepromWriteByte(int addr, byte t, int& checksum) {
    checksum = checksum ^ t;
    EEPROM.update(addr++, (t & 0xff));
}

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
    Serial.println(F("Save to EEPROM"));
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
  eepromWriteByte(eepromInvertBase, sensorInvert, check);
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
  sensorInvert = 0;
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

int readEepromByte(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr);
    addr ++;
    checksum = checksum ^ v;
    return v;
}

int readEepromInt(int &addr, int& checksum, boolean& allzero) {
    int v = EEPROM.read(addr) + (EEPROM.read(addr + 1) << 8);
    addr += 2;
    checksum = checksum ^ v;
    Serial.print(v & 0xff, HEX); Serial.print("-"); Serial.print((v >> 8) & 0xff, HEX); Serial.print(" ");
    if (v != 0) {
      allzero = false;
    }
    Serial.print(F(" = ")); Serial.println(v);
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
  eeAddr = eepromInvertBase;
  sensorInvert = readEepromByte(eeAddr, check, allzero);
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

// -------------------- S88 Section ----------------------------
// Shamelessly copied from https://sites.google.com/site/sidloweb/elektrika/s88-ir-detektor
// Copyright (c) Sidlo

byte data = 0 ;                   // data byte
int bitCounter = 0 ;              // bit counter
short byteIndex = 0;

byte s88BusState[16] = {};

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

  // if received the whole byte, remember it in the bus state variable.
  if (xferBit == 7) {
    if (byteIndex < sizeof(s88BusState)) {
      s88BusState[byteIndex++] = data;
    }
  }

  if (debugS88) {
    if ((bitCounter  % 8) == 0 && bitCounter > 7 && data != 0) {
      Serial.print(F("S88 read byte: ")); Serial.print(bitCounter / 8 + 65); Serial.print(F(" = "));
      Serial.println(data);
    }
  }
}
