/**
   Copyright (c) 2016, 2020, svatopluk.dedic@gmail.com

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License
   as published by the Free Software Foundation; either version 2
   of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   I measured that analogRead() can be executed ~800 times in 100ms, which could give ~8kHz sampling frequency,
   but because of input switches the effective max frequency will be much lower.

   Algorith outline:
   Measures ambient light on phototranzistors when IR LEDs are off, then measures light again with IR LEDs on.
   Averages several subsequent measures for low, then high.  If difference between "high" and "low" light value is
   above threshold the sensor is probably receiving reflected IR LED's light so it is detecting a vehicle above it.

   A "match counter" is kept for each sensor. Match counter increases when the sensor is detecting the correct LED phase,
 * *decreases* if not. So fluctuations quickly decrease the match counter towards zero, while for detection to
   be reported, the match counter must accumulate several subsequent positives.

   After some defined period of time, the match counters are processed into occupancy detection bits and stored for
   S88 bus handler.

   Details:
   The ADC is configured for free-running mode, which generates an interrupt after ADC conversion is done.
   The interrupt routine fetches ADC value and accumulates it in "measureHigh" (LED on) and "measureLow" (LED off)
   arrays. Each input is sampled "inputADCConversions" times, then ADC is reconfigured for the next input.
   After input changes, subsequent "discardMeasuresAfterChange" readings are discarded to allow internal capacitor
   to stabilize (this is instead of wait). "measureSamples" counter is incremented, so average value can be computed
   from the accumulated measurements.

   The ISR increments "readCycles" diagnostic variable.

   The main loop waits for "sampleMillis" milliseconds (default = 7), then evaluates "measureHigh" and "measureLow"
   (recorded in ADC ISR) - computes an average value, stores in "resultHigh" or "resultLow". When IR LED is on
   (= the second measurement phase), match counters are computed for each sensor.

   Ver 1.1: In order to reduce power consumption (initially: 40mA * 8 LEDs = 0,32A !! per detector board), the board
   was changed to light just 2 leds at a time, through ULN2003 transistor array. 4 sensors are read during each cycle:
   the 2 that are lit at the moment, and the 2 sensors that HAVE been lit during the previous cycle.

   After "measureInterval" (97 milliseconds), match counters propagate to "sensorStateBits" which is being read by S88
   handler.

   The other interrupts come from S88 clock/reset and causes bits to be promoted into S88 register or shifted.

   When changing any variables used by ISRs, noInterrupts()/interrupts() must be called to protect ISRs from seeing
   an inconsistent computation state.

*/

#include <Bounce2.h>
#include <EEPROM.h>
#include "Common.h"
#include "Settings.h"
#include "Relays.h"

#define STARTUP_MSG "ReflexShield (c) Belgarat@klfree.net, v. 1.3, 6/2021"

const int debug = 1;            // set to >0 to activate debug output on the serial console. The collector will use delays 200ms and will print stats each 2s
const int debugSensors = 0;
const int debugLow = 0;
const int debugControl = 1;     // debug control commands
const int debugLed = 0;
const int debugMgmt = 0;
const int debugS88 = 0;
const int debugS88Low = 0;
const int debugVirtual = 0;
const int numChannels = 8;      // number of sensors used. Max 5 on Arduino UNO, 8 on Nano.
const int maxVirtualSensors = 16; // maximum number of virtual sensors

const int reductionLight = 250;   // the percentage will be reduced 4 times if the ambient light opens tranzistor above this level.

const int counterThreshold = 3;     // minimum sensor counter level to consider the sensor seeing the reflected LED light
const int defaultThreshold = 250;   // default sensor threshold for 'occupied'
const int occupiedThresholdReduction = 40;  // when occupied, the threshold is lowered to this percentage of the configured value.
const int defaultDebounce = 200;    // default debounce, when the sensor goes off
const int virtualSensorFadeOffDelay = 300; // fade off of virtual sensor.
const int defaultSensorFadeOffDelay = 0; // default fade off for physical sensor. 0 means immediate state change.
const int defaultSensorFadeOnDelay = 0;  // default fade on for physical sensor. 0 means immediate state change.

// The layout of the optoshield assumes, that HIGH on common LED control output will trigger a tranzistor to connect LEDs to GND.
// If experimenting with the Arduino alone on the breadboard, it would be better to trigger LED with LOW, connect LED to D12-GND
// and the phototranzistor to +5V-Ax
const int ledOnValue  = HIGH;
const int ledOffValue  = (ledOnValue == HIGH ? LOW : HIGH);

const int BUTTON_PLUS     = 6;
const int BUTTON_MINUS    = 7;
const int BUTTON_NEXT     = 8;

const int RELAY_1 = 6;
const int RELAY_2 = 7;
const int RELAY_3 = 8;

const int longButtonPress = 500;       // long press, millseconds
const int configButtonPress = 2000;     // config button press, milliseconds
const int resetButtonPress = 10000;     // reset button press, milliseconds
const int calibrationTime = 10000;      // how long is the input measured during calibration
const long configIdleTimeExit = 60000; // after 60 sec of inactivity the configuration closes

const int LOAD_INT_0      = 2 ;        // 2 LOAD 0 int
const int CLOCK_INT_1     = 3 ;        // 3 CLOCK 1 int
const int DATA_IN         = 4 ;        // data in
const int DATA_OUT        = 5 ;        // data out
const int LED_SIGNAL      = 13;       // signal LED

const int LED_ACK         = 13;       // ACK LED

const int reportS88Loss = 1;          // will flash LED if S88 CLK signal is not present

const int inputADCConversions = 5;  // how many ADC conversions is done on a particular input before moving to the next one
const int sampleMillis = 7;         // Milliseconds between ADC collections
const int measureInterval = 53;     // Interval for measuring sensor inputs to produce yes/no occupation result

const int senseMax = 600;           // minimum sensitivity
const int senseMin = 100;           // maximum sensitivity

const int debounceMin = 10;
const int debounceMax = 1000;
const int discardMeasuresAfterChange = 3; // how many ADC readings should be discarded after input channel change; this is instead of delay :) One reading = 104 usec.

const int analogInputs[] = { A0, A1, A2, A3, A4, A5, A6, A7 };

// LEDs for individual detectors. Controlled in chunks defined in ledSlots below.
const byte digitalOutputs[] = { 12, 12, 11, 11, 9, 9, 10, 10};
const int eepromThresholdBase = 0x00;
const int eepromVirtualBase = 0x60;
const int eepromRelayBase = 0xb0;
const int eepromChecksum = 0xf0;

const int ledSlotsCount = 4;
const int ledSlotSize = 2;

// Chunks of LEDs controlled by individual outputs, must be consistent with digitalOutputs
const byte ledSlots[ledSlotsCount][ledSlotSize] = {
  { 0, 1 },
  { 2, 3 },
  { 4, 5 },
  { 6, 7 }
};

extern void (* charModeCallback)(char);

// This is the internal sensor value, after reading / stabilizing the inputs. Values are copied from internal to sensorStateBits after fadeIn / fadeOut
// timeout, if defined.
byte internalSensorStateBits = 0x00;

// Received sensor states, captured from S88. They wiull be copied to sensorStateBits after appropriate fadeIn/fadeOut delay.
volatile unsigned long capturedSensorStates = 0x00;

// The OUTPUT; this is the S88 sensor value, A0 is mapped to bit 0. Bits for unused channels are set to 0.
volatile unsigned long sensorStateBits = 0x00;

long sensorLastUp[] = {
  -1, -1, -1, -1, -1, -1, -1, -1
};

// if non-zero, generates virtual sensors.
int virtualSensorDefaultDelay = 0x00;

struct AttachedSensor {
  short threshold;
  short debounce;

  short fadeOnTime;
  short fadeOffTime;

  byte  occupiedReduction : 7;
  boolean invert : 1;

  AttachedSensor() : threshold(defaultThreshold), debounce(defaultDebounce), fadeOnTime(0), fadeOffTime(0), occupiedReduction(0), invert(false) {}
};

struct VirtualSensor {
  static const VirtualSensor NONE;

  unsigned short monitoredId = 0;
  unsigned short sensorDelay = 0;

  VirtualSensor() : VirtualSensor(0, 0) {}
  VirtualSensor(short monitored, short delay) : monitoredId(0), sensorDelay(0) {}

  boolean isFree() const {
    return monitoredId == 0;
  }
  boolean isNone() const {
    return this == &NONE;
  }

  static VirtualSensor& find(int id);
};

static const VirtualSensor VirtualSensor::NONE = VirtualSensor();

struct VirtualSensorState {
  unsigned long fadeOnTime;
  unsigned long fadeOffTime;

  VirtualSensorState() : fadeOnTime(0), fadeOffTime(0) {}

  void clear() {
    fadeOnTime = fadeOffTime = 0;
  }
};

VirtualSensor virtualSensors[maxVirtualSensors];
VirtualSensorState virtualSensorStates[maxVirtualSensors];
AttachedSensor attachedSensors[numChannels];

unsigned long knownSensorStates = 0x00;
long sensorFadeTimeout[numChannels] = { 0 };

byte virtualSensorCount = 0;
byte sensorOverrides = 0;

// ---------------------------------------------------------------------------------------------------------------------------------------------------------------------

short numBufferedBits = 8;
const int maxADCValue = 1024;
int mux;  // the ADCMUX value saved after initialization; will be used as a base for channel selection


// Variables access from the ADC interrupt routine. Before reading from the main code or other ISR,
// one MUST call noInterrupts() to avoid concurrent access.
volatile long measureLow[numChannels];      // inputs measured in 'low' state without LED; cummulative value
volatile long measureHigh[numChannels];     // inputs measured in 'high' state with LED on; cummulative value
volatile int measureSamples[numChannels];   // how many samples are accumulated in individual channel's input. Since the ADC conversion is driven by interrupt,
volatile boolean ledState;                      // the current LED light state; applies to all channels
volatile int adcConversionCounter;          // number of ADC conversion left for the current input
volatile short scanInputs[numChannels + ledSlotSize + 1] = { -1 };
volatile boolean startNextCycle = false;
volatile boolean scanningDisabled = true;

const int scanInputsCount = (sizeof(scanInputs) / sizeof(scanInputs[0]));

short enabledChannels = 0xff;
boolean outputExtensionAttached = false;

int resultLow[numChannels];   // averaged value for 'low' led state
int resultHigh[numChannels];  // averaged value for 'high' led state
short sensorCounters[numChannels];  // sense counters for each sensor

int calibrationMin = 1000;
int calibrationMax = -1;
extern long calibrationStart;
long calibrationSum;
short calibrationCount;

// ---------- Variables used by ADC value collecting routing ---------------
short input;     // the current input number; cycles from 0 to numChannels - 1
short wasSwitch; // nonzero if input switch happened; discard one ADC reading after the switch
boolean intrLedState; // copy of the LED state for the interrupt routine

volatile long intCount;     // debug only: the number of ADC interrupts.
volatile int readCycles;    // debug only: how many complete read cycles through all the sensors were perfomed since the data collection
int lastReadCycles;         // debug only: number of measure cycles used during the last ADC collection.
volatile long collectIntCount; // debug only: count of interrupts occuring in the last ADC collection period

long lastMillis;          // time of the last evaluation of occupancy
long lastDisplayMillis;   // time of the last diagnostic display on serial line
int collectDisplayCount = 1;
boolean sometimesDebug;

short configChannel = -1;
short cfgState;
long configLastCommand;
extern long lastLedSignalled;

boolean relaysPresent = false;

long curMillis = 0;

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
  attachInterrupt(digitalPinToInterrupt(LOAD_INT_0), loadInt, RISING);

  pinMode(CLOCK_INT_1, INPUT_PULLUP) ;
  attachInterrupt(digitalPinToInterrupt(CLOCK_INT_1), clockInt, RISING) ;

  pinMode(DATA_IN, INPUT) ;

  pinMode(DATA_OUT, OUTPUT) ;
  digitalWrite(DATA_OUT, LOW) ;
}

void checkButtonsConnected() {
  if (atLeastOneRelayConnected()) {
    outputExtensionAttached = true;
  } else {
    // test if the PLUS button is connected
    pinMode(BUTTON_NEXT, INPUT_PULLUP);
    //  digitalWrite(BUTTON_NEXT, LOW);

    long now = millis();
    int cnt = 0, total = 0;
    long t;

    do {
      t = millis();
      boolean state = digitalRead(BUTTON_NEXT);
      total++;
      if (state) {
        cnt++;
      }
    } while (t - now < 200);
    outputExtensionAttached = (cnt < (total / 50));
    Serial.print("Output high: "); Serial.print(cnt); Serial.print(", reads: "); Serial.println(total);
  }
  if (outputExtensionAttached) {
    Serial.println(F("Output extension board attched."));
  } else {
    Serial.println(F("Output extension board not present, assuming button pins"));
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F(STARTUP_MSG));
  Serial.println(F("Starting up..."));

  // DISABLE internal pull-ups for analog inputs, there's an external pull-down resistor.
  for (int i = 0; i < numChannels; i++) {
    int pno = analogInputs[i];
    pinMode(pno, INPUT);
    digitalWrite(pno, LOW);
  }

  // initialize digital (IR LED power) outputs
  for (int i = 0; i < sizeof(digitalOutputs) / sizeof(digitalOutputs[0]); i++) {
    pinMode(digitalOutputs[i], OUTPUT);
    digitalWrite(digitalOutputs[i], ledOffValue);
  }

  initializeS88Pins();

  // Signal LED
  pinMode(LED_SIGNAL, OUTPUT);
  initialSignalBlink();

  pinMode(BUTTON_PLUS, INPUT_PULLUP);
  pinMode(BUTTON_MINUS, INPUT_PULLUP);
  pinMode(BUTTON_NEXT, INPUT_PULLUP);

  initialLoadEEPROM();
  checkButtonsConnected();

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

  setupTerminal();
  initTerminal();

  registerLineCommand("SAV", &commandSave);
  registerLineCommand("DMP", &commandDump);
  registerLineCommand("INF", &commandInfo);
  registerLineCommand("CAL", &commandCalibrate);
  registerLineCommand("CAC", &commandContinueCalibration);
  registerLineCommand("MON", &commandMonitor);
  registerLineCommand("CAD", &commandCalibrateDebounce);
  registerLineCommand("SEN", &commandSensitivity);
  registerLineCommand("SFT", &commandFadeTime);
  registerLineCommand("OTR", &commandReduction);
  registerLineCommand("INV", &commandInvert);

  registerLineCommand("VSN", &commandVirtualSensor);

  registerLineCommand("SSN", &commandSetSensor);
  registerLineCommand("S8L", &commandS88Load);
  registerLineCommand("S8D", &commandS88Data);
  registerLineCommand("S8T", &commandS88Tick);

  registerLineCommand("MSR", &commandMeasure);

  registerLineCommand("REL", &commandRelay);

  initRelays();
}

int prevInput = -1;

// Interrupt service routine for the ADC completion
ISR(ADC_vect)
{
  // the proper number of conversions passed, advance the input.
  noInterrupts();
  if (startNextCycle) {
    initScanCycle();
    startNextCycle = false;
  } else if (prevInput >= 0) {
    int readLo = ADCL; // ADCL must be read first; reading ADCH will trigger next coversion.
    int readHi = ADCH;
    int v = (readHi << 8) | readLo;
    if (wasSwitch) {
      wasSwitch--;
      // discard the first reading(s) after input switch.
      startAnalogRead(prevInput);
      return;
    }
    intCount++;
    if (debugLow) {
      Serial.print("* Measure: "); Serial.print(prevInput); Serial.print(" = "); Serial.print(v); Serial.print("/");
      Serial.println(digitalRead(digitalOutputs[prevInput]));
    }
    if (intrLedState) {
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
  if (nextInput >= 0) {
    scanningDisabled = false;
    prevInput = nextInput;
    // also enables interrupts.
    startAnalogRead(nextInput);
    interrupts();
  } else {
    if (debugLow) {
      Serial.println("* Scanning disabled");
    }
    prevInput = nextInput;
    scanningDisabled = true;
    interrupts();
  }
}

void initScanCycle() {
  if (debugLow) {
    Serial.println("* new scan cycle");
  }
  input = 0;
  intrLedState = ledState;
  wasSwitch = discardMeasuresAfterChange;
}

void changeLeds(int prevInput, int nextInput) {
  if (debugLed) {
    Serial.print("prev: "); Serial.print(prevInput); Serial.print(", next: "); Serial.print(nextInput); Serial.print(" input: "); Serial.println(input);
  }
  short ledNext = -1;
  if (nextInput >= 0) {
    if (prevInput != nextInput) {
      wasSwitch = discardMeasuresAfterChange;
    }
    ledNext = digitalOutputs[nextInput];
  }
  if (prevInput >= 0) {
    short ledPrev = digitalOutputs[prevInput];
    if (ledPrev != ledNext) {
      if (debugLed) {
        Serial.print("OFF: "); Serial.print(ledPrev);
      }
      digitalWrite(ledPrev, LOW);
    } else {
      if (!startNextCycle) {
        return;
      }
    }
  }
  if (nextInput < 0) {
    return;
  }

  if (debugLed) {
    Serial.print(" ON: "); Serial.print(ledNext);
    Serial.print(" state: "); Serial.print(intrLedState);
  }
  digitalWrite(ledNext, intrLedState ? HIGH : LOW);
  if (debugLed) {
    Serial.println();
  }
}

int errors; // accumulate cases where the samples were read although readCycles was not incremented
int lastCycle; // last cycle observed by the main loop.

// must be called with interrupts disabled !
void configureScanInputs() {
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
  if (debugSensors && sometimesDebug) {
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
      if (debugSensors && sometimesDebug) {
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
  configureScanInputs();
  switchLEDState();
  startNextCycle = true;
  interrupts();
  if (debugSensors && sometimesDebug) {
    Serial.println();
  }
  return 1;
}

// update tracking counters for each sensor, when the LED is (currently) lit
void updateSensorCounters() {
  for (int x = 0; x < numChannels; x++) {
    const AttachedSensor& sen = attachedSensors[x];
    int res = resultHigh[x];

    if ((configChannel > -1) && (configChannel == x)) {
      if (calibrationMin > res) {
        calibrationMin = res;
      }
      if (calibrationMax < res) {
        calibrationMax = res;
      }
      calibrationSum += res;
      calibrationCount++;
      if ((calibrationStart >= 0) && (cfgState == CONFIG_CALIBRATE_HIGH || cfgState == CONFIG_CALIBRATE_LOW)) {
        printCalibrationStats();
      }
    }

    int base = resultLow[x];
    int range = base;

    int diff = resultHigh[x] - base;
    if (diff < 0) {
      diff = -diff;
    }
    int comp = sen.threshold;
    int thr = comp;
    // This is ugly hack -- some kind of hysterezis; if a sensor detected previously,
    // much lesser threshold is applied to detect again.
    if ((internalSensorStateBits & (1 << x)) > 0) {
      int red = sen.occupiedReduction;
      if (red == 0) {
        red = occupiedThresholdReduction;
      }
      thr = (int)(((long)comp * red) / 100);
    }
    /*
    if (x == 2) {
      Serial.print("Thr: "); Serial.print(thr); Serial.print(", diff: "); Serial.println(diff);
    }
    */

    if (diff > thr) {
      // the sensor received light above the base in the 'proper' time
      sensorCounters[x]++;
    } else {
      sensorCounters[x]--;
    }
    if (sometimesDebug && debugSensors) {
      Serial.print(F("C-")); Serial.print(x);
      Serial.print(F(", Low = ")); Serial.print(resultLow[x]);
      Serial.print(F(", High = ")); Serial.print(resultHigh[x]);
      Serial.print(F(", Comp = ")); Serial.print(comp);
      Serial.print(F(", Range = ")); Serial.print(diff);
      Serial.print(F(", Thresh = ")); Serial.print(thr);
      Serial.print(F(", CNT = ")); Serial.print(sensorCounters[x]); Serial.println("");
    }
  }
  if (debugSensors && sometimesDebug) {
    Serial.println();
  }
}

void switchLEDState() {
  if (debugSensors && sometimesDebug) {
    Serial.print(F("Led state was: ")); Serial.println(ledState);
  }
  ledState = ledState ? 0 : 1;
  if (debugSensors && sometimesDebug) {
    Serial.print(F("Switched LED ")); Serial.println(ledState);
  }
}

volatile long maxConfigSensorDebounce = 0;
long configSensorDebounce = 0;


// Sets detected sensor state. First goes to the 'internal' state. Then, possibly after a delay
// propagates into S88.
void setDetectedSensorBit(int sensor, boolean state) {
  int v = 1 << sensor;
  if ((sensorOverrides & v) > 0) {
    return;
  }
  // no change.
  if (((internalSensorStateBits & v) > 0) == (state > 0)) {
    return;
  }

  if (state) {
    internalSensorStateBits |= v;
  } else {
    internalSensorStateBits &= ~v;
  }

  if (((sensorStateBits & v) > 0) == (state > 0)) {
    // no visible change -> reset the fade tiemout
    sensorFadeTimeout[sensor] = 0;
    return;
  }
  const AttachedSensor &as = attachedSensors[sensor];
  short tm = state ? as.fadeOnTime : as.fadeOffTime;
  if (tm > 0) {
    sensorFadeTimeout[sensor] = curMillis + tm;
    return;
  }
  if (state) {
    sensorStateBits |= v;
  } else {
    sensorStateBits &= ~v;
  }
  captureSensorBit(sensor + 1, state);
}

void delayedSensorChanges() {
  for (int i = 0; i < numChannels; i++) {
    long tt = sensorFadeTimeout[i];
    if (tt > 0 && tt < curMillis) {
      sensorFadeTimeout[i] = 0;

      int v = (1 << i);
      boolean state = (internalSensorStateBits & v) > 0;

      if (state) {
        sensorStateBits |= v;
      } else {
        sensorStateBits &= ~v;
      }
      captureSensorBit(i + 1, state);
    }
  }
}

void printBinaryData(unsigned long data, int width) {
  unsigned long m = 1;
  for (int i = 0; i < width; i++) {
    Serial.print((data & m) > 0 ? '1' : '0');
    m = m << 1;
  }
}


void captureSensorBit(int sensorId, boolean state) {
  if (debugVirtual) {
    Serial.print(F("Captured sensor ")); Serial.print(sensorId); Serial.print('='); Serial.println(state);
  }
  unsigned long m = 0;
  int vIndex = -1;
  for (byte i = 0; i < virtualSensorCount; i++) {
    const VirtualSensor& s = virtualSensors[i];
    if (s.monitoredId == sensorId) {
      vIndex = i;
      break;
    }
  }
  if (vIndex < 0) {
    return;
  }
  m = 1L << vIndex;
  if (debugVirtual) {
    Serial.print(F("Captured input #")); Serial.print(vIndex + 1); Serial.print(':'); Serial.println(state);
  }
  if (state) {
    capturedSensorStates |= m;
  } else {
    capturedSensorStates &= ~m;
  }
}

/**
   Evaluates sensor state, builds the S88 response byte
*/
long lastUpdateMillis = 0;

void updateSensorBits() {
  long t = millis();
  /*
  if (lastUpdateMillis > 0) {
    Serial.print("Update delay: "); Serial.println((t - lastUpdateMillis));
  }
  lastUpdateMillis = t;
  */
  byte result = 0;
  noInterrupts();
  byte lastState = internalSensorStateBits;
  for (int i = 0; i < numChannels; i++) {
    const AttachedSensor& sen = attachedSensors[i];

    int v = 1 << i;
    boolean invertedSensor = sen.invert;
    boolean litValue = !invertedSensor;
    boolean lastBit = (lastState & (1 << i)) > 0;
    boolean vb = !litValue;

    if (sensorCounters[i] > counterThreshold) {
      vb = litValue;
      sensorLastUp[i] = t;
    } else {
      if (lastBit) {
        long up = sensorLastUp[i];
        long diff = (t - up);
        if (i == configChannel) {
          configSensorDebounce = diff;
          if (maxConfigSensorDebounce < configSensorDebounce) {
            maxConfigSensorDebounce = configSensorDebounce;
          }
        }
        // if going down, wait at least the 'debounce' time before dropping the sensor bit from the result.
        if ((up != -1) && (diff < sen.debounce)) {
          if (sometimesDebug) {
            Serial.print(F("Sensor debounced: ")); Serial.print(i); Serial.print(F(", time left: ")); Serial.println(t - sensorLastUp[i]);
          }
          vb = litValue;
        }
      }
    }
    sensorCounters[i] = 0;
    setDetectedSensorBit(i, vb);
  }
  interrupts();
  // signal the status to the "control panel" LED
  if (configChannel >= 0) {
    if ((lastLedSignalled - t) > 1000) {
      int ledState = result & (1 << configChannel) ? HIGH : LOW;
      digitalWrite(LED_SIGNAL, ledState);
    }
  }
}

long lastLedSignalled;

long lastSignalMillis;
long  lastS88Millis;
long lastS88Signalled = -1;
boolean blinkState;

const int lostS88[] = { 500, 500, 500, 500, 200, 500, 200, 500, 0 };

void handleSignalLed() {
  long current = millis();
  if ((current - lastLedSignalled) < 1000) {
    return;
  }
  if (cfgState == CONFIG_NONE) {
    if ((current - lastS88Millis > 1000)) {
      if (reportS88Loss && !isAckRunning()) {
        if (current - lastS88Signalled > 30000) {
          Serial.println("S88 connection lost");
          makeLedAck(&lostS88[0]);
          lastS88Signalled = current;
          return;
        }
      }
    }
    if (internalSensorStateBits > 0) {
      digitalWrite(LED_SIGNAL, HIGH);
    } else {
      digitalWrite(LED_SIGNAL, LOW);
    }
  } else {
    if (configChannel >= 0) {
      if ((internalSensorStateBits & (1 << configChannel)) > 0) {
        digitalWrite(LED_SIGNAL, HIGH);
      } else {
        digitalWrite(LED_SIGNAL, LOW);
      }
    }
  }
}


void loop() {
  if (cfgState == CONFIG_NONE && charModeCallback == NULL) {
    configChannel = -1;
  }
  sometimesDebug = 0;
  delay(sampleMillis);
  if (collectDisplayCount-- == 0) {
    collectDisplayCount = 13;
    sometimesDebug = 1;
  }
  if (collectADC()) {
    updateSensorCounters();
    delayMicroseconds(50);
  } else {
    errors++;
    if (debugSensors) {
      Serial.print("Error");
    }
  }

  curMillis = millis();

  if (curMillis - lastMillis >= measureInterval) {
    updateSensorBits();
    handleSignalLed();
    if (debugSensors && sometimesDebug) {
      Serial.print(F(" IntCount: "));  Serial.print(collectIntCount);
      Serial.print(F(" Errors: ")); Serial.print(errors);
      Serial.print(F(" bit state = "));
      Serial.println(internalSensorStateBits);
      Serial.print(F("Last read cycles: ")); Serial.println(lastReadCycles);
      lastDisplayMillis = millis();
    }
    lastMillis = curMillis;
  }
  handleButtons();
  handleAckLed();
  handleCalibration();
  handleMonitor();
  handleMeasure();
  processVirtualSensorInputs();
  fadeOffVirtualSensors();
  processRelays();
  processTerminal();
}

// ========================= EEPROM functions ================================
void writeEEPROM() {
  if (debugControl) {
    Serial.println(F("Save to EEPROM"));
  }
  int eeAddr = eepromThresholdBase;
  eeBlockWrite(0xad, eeAddr, &attachedSensors[0], sizeof(attachedSensors));

  eeAddr = eepromVirtualBase;
  eeBlockWrite(0xaa, eeAddr, &virtualSensors[0], sizeof(virtualSensors));

  eeAddr = eepromRelayBase;
  eeBlockWrite(0xaf, eeAddr, &relays[0], sizeof(relays));
}

void initializeEEPROM() {
  if (debugControl) {
    Serial.println("Initializing EEPROM");
  }
  int check = 0;
  for (int i = 0; i < numChannels; i++) {
    attachedSensors[i] = AttachedSensor();
  }
  for (int i = 0; i < maxVirtualSensors; i++) {
    virtualSensors[i] = VirtualSensor();
  }
  refreshSensorCount();

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

void initialLoadEEPROM() {
  if (debugControl) {
    Serial.println("Loading from EEPROM");
  }

  int check = 0;
  boolean allOK;

  int eeAddr = eepromThresholdBase;
  allOK = eeBlockRead(0xad, eeAddr, &attachedSensors[0], sizeof(attachedSensors));
  if (!allOK) {
    Serial.println("Attached sensors bad");
  }

  eeAddr = eepromVirtualBase;
  allOK &= eeBlockRead(0xaa, eeAddr, &virtualSensors[0], sizeof(virtualSensors));
  if (!allOK) {
    Serial.println("Virtual sensors bad");
  }

  eeAddr = eepromRelayBase;
  allOK &= eeBlockRead(0xaf, eeAddr, &relays[0], sizeof(relays));
  if (!allOK) {
    Serial.println("Relay bad");
  }

  if (!allOK) {
    Serial.print(F("EEPROM checksum does not match: "));
    resetEEPROM();
    return;
  }
  refreshSensorCount();
  if (debugControl) {
    Serial.println(F("Loading done."));
  }
}

// -------------------- S88 Section ----------------------------
// Shamelessly copied from https://sites.google.com/site/sidloweb/elektrika/s88-ir-detektor
// Copyright (c) Sidlo

unsigned long data = 0 ;                   // data byte
short bitCounter = 0 ;              // bit counter
short byteIndex = 0;

// State of the S88 bus _after_ this sensor board. The immediately next sensor board (8bit) occupies byte #0.
byte s88BusState[16] = {};

volatile unsigned long s88InputData = 0x00;
volatile short s88InputDataLength = 0;

/***************************************************************************
   Interrupt 0 LOAD.
*/
void loadInt() {
  data = sensorStateBits;

  if (virtualSensorCount > 8) {
    numBufferedBits = 24;
  } else if (virtualSensorCount > 0) {
    numBufferedBits = 16;
  } else {
    numBufferedBits = 8;
  }
  if (debugS88) {
    Serial.print("Sensor bits: ");
    for (int i = 0; i < numBufferedBits; i++) {
      unsigned long m = (1 << i);
      Serial.print((sensorStateBits & m) ? '1' : '0');
    }
    Serial.println();
  }
  bitCounter = 0 ;
  lastS88Millis = millis();
  byteIndex = 0;
}

/***************************************************************************
   Interrupt 1 CLOCK.
*/
void clockInt() {
  int xferBit = (bitCounter++) % numBufferedBits;
  int ov = bitRead(data, xferBit);
  if (debugS88) {
    Serial.print("S88: "); Serial.println(ov);
  }
  digitalWrite(DATA_OUT, ov) ;

  // read input
  int x;
  if (s88InputDataLength > 0) {
    s88InputDataLength--;
    x = s88InputData & 0x01;
    s88InputData = s88InputData >> 1;
  } else {
    x = digitalRead(DATA_IN);
  }
  bitWrite(data, xferBit, (x > 0 ? 1 : 0));

  // if received the whole byte, remember it in the bus state variable.
  if (byteIndex < sizeof(s88BusState)) {
    if (xferBit == 7) {
      s88BusState[byteIndex++] = data;
    } else if (xferBit == 15) {
      s88BusState[byteIndex++] = data >> 8;
    } else if (xferBit == 23) {
      s88BusState[byteIndex++] = data >> 16;
    }
  }

  captureSensorBit(bitCounter + numBufferedBits, x);

  if (debugS88) {
    if ((bitCounter  % 8) == 0) {
      unsigned long v = data;
      int shift = (bitCounter - 8 ) % numBufferedBits;
      if (shift) {
        v = v >> shift;
      }
      v = v & 0xff;
      Serial.print(F("S88 read byte: ")); Serial.print(bitCounter / 8); Serial.print(F(" = "));
      Serial.println(v);
    } else {
      Serial.print(F("Bit counter: ")); Serial.println(bitCounter);
    }
  }
}
