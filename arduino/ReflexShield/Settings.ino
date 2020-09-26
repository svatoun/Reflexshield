

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

const int* blinkPtr = NULL;
long blinkLastMillis;
byte pos = 0;
boolean ackLedState = false;
int pulseCount = 0;

boolean isAckRunning() {
  return blinkPtr != NULL;
}

void handleAckLed() {
  if (blinkPtr == NULL) {
    return;
  }
  long t = millis();
  long l = t - blinkLastMillis;
  lastLedSignalled = t;
  if (l < blinkPtr[pos]) {
    return;
  }
  blinkLastMillis = t;
  pos++;
  if (debugLed) {
    Serial.print(F("Next ACK time: ")); Serial.println(blinkPtr[pos]);
  }
  if (blinkPtr[pos] == 0) {
    ackLedState = 0;
    digitalWrite(LED_ACK, LOW);
    if (pulseCount > 0) {
      pulseCount--;
      makeLedAck(&blinkShort[0]);
    } else {
      if (debugLed) {
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

void makeLedAck(const int *  ledSequence) {
    blinkPtr = ledSequence;
    pos = 0;
    ackLedState = 1;
    blinkLastMillis = millis();
    lastLedSignalled = blinkLastMillis;
    digitalWrite(LED_ACK, HIGH);
    if (debugLed) {
      Serial.print(F("LED ACK start: ")); Serial.println(blinkPtr[pos]);
    }
}

void pulseChannelNumber() {
  if (debugLed) {
    Serial.print(F("ACK channel ID ")); Serial.println(configChannel);
  }
  pulseCount = configChannel;
  makeLedAck(&blinkShort[0]);
}


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
  if (minusLen != -1 || (plusLen != -1 && nextLen != -1)) {
    return;
  }
  if (nextLen > resetButtonPress) {
    // reinitialize EEPROM
    resetEEPROM();
    return;
  } else if (plusLen > configButtonPress) {
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
  if (configChannel < 0) {
    configChannel = 0;
  }
  configChannel = configChannel % numChannels;
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
      sensorDebounces[configChannel] = debounceMax;
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
      sensorDebounces[configChannel] = debounceMin;
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

/**
 * calibration start time, millis(). The calibration cycle will end after
 * some additional millis
 */
long calibrationStart = -1;

/**
 * Max measured sensor in empty state
 */
int emptyHigh;
/**
 * Min measured sensor in empty state
 */
int emptyLow;

/**
 * Calibration confirmed by the terminal, rather
 * than button
 */
boolean terminalConfirm = false;

void startCalibration() {
  Serial.print(F("Calibrating sensor #")); Serial.print(configChannel + 1); Serial.println(F("Measuing data on CLEAR"));
  cfgState = CONFIG_CALIBRATE_LOW;
  makeLedAck(&blinkCalibrateLow[0]);
  setupCalibration();
}

void setupCalibration() {
  calibrationMax = -1;
  calibrationMin = 1000;
  calibrationSum = 0;
  calibrationCount = 0;
  calibrationStart = millis();
  configLastCommand = calibrationStart;
  terminalConfirm = false;
  if (debugControl) {
    Serial.println("Calibration started");
  }
}

void startCalibrationHigh() {
  // leave min/max from the preceding attempt.
  cfgState = CONFIG_CALIBRATE_HIGH;
  calibrationStart = millis();
  terminalConfirm = false;
  makeLedAck(&blinkCalibrateCont[0]);
  if (debugControl) {
    Serial.println("Calibration started");
  }
}

boolean terminalCalibration = false;

void handleCalibration() {
  long m = millis();
  switch (cfgState) {
    case CONFIG_CALIBRATE_LOW: {
      if ((calibrationStart > -1) && 
          ((m - calibrationStart) > calibrationTime)) {
        makeLedAck(&blinkCalibrateCont[0]);
        calibrationStart = -1;
        emptyHigh = calibrationMax;
        emptyLow = calibrationMin;
        Serial.print(F("\nCalibration: empty read, low = ")); Serial.print(emptyLow); Serial.print(F(", high = ")); Serial.println(emptyHigh);
        Serial.println(F("Empty calibration complete, cover sensor and press ENTER to calibrate obstacle"));
      }
      return;
    }
    case CONFIG_CALIBRATE_HIGH:  
      if (calibrationStart > -1 && 
          ((m - calibrationStart) > calibrationTime)) {
        makeLedAck(&blinkCalibrateEnd[0]);
        calibrationStart = -1;
        computeSensitivity();
        Serial.println(F("\ndCalibration complete"));
        if (terminalCalibration) {
          cfgState = CONFIG_NONE;
          resetTerminal();
          return;
        } else {
          cfgState = CONFIG_SENSE;
        }
        return;
      }

    case CONFIG_CALIBRATE_DEBOUNCE:
      if (calibrationStart > -1 && 
          ((m - calibrationStart) > calibrationTime)) {
          long m = maxConfigSensorDebounce;
          long debounceTime = (m * 4 / 3);
          if (debounceTime < 30) {
            debounceTime = defaultDebounce;
          }
          if (debounceTime > 1000) {
            debounceTime = 1000;
          }
          sensorDebounces[configChannel] = debounceTime;
          Serial.print(F("Debounce calibration complete. Set to "));   Serial.println(debounceTime);
          if (terminalCalibration) {
            resetTerminal();
          }
          cfgState = CONFIG_NONE;
      }
      return;
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
      terminalCalibration = false;
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
      diff = 20;
      kind = 1;
    } else {
      diff = 1;
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
      diff = -20;
      kind = 1;
    } else {
      diff = -1;
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

boolean exitConfigurationNoReload() {
    if (cfgState == CONFIG_NONE) {
      return false;
    }
    cfgState = CONFIG_NONE;
    if (debugControl) {
      Serial.println(F("Exited configuration"));
    }
    makeLedAck(&blinkConfigEnd[0]);
    configChannel = -1;
    terminalCalibration = false;
    terminalConfirm = false;
    resetTerminal();
    return true;
}

void exitConfiguration() {
  if (exitConfigurationNoReload()) {
    initialLoadEEPROM();
  }
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
    Serial.print("Exiting config. "); Serial.print("cfgState:"); Serial.print(cfgState); Serial.print("nextLen"); Serial.print(nextLen);
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
      if (calibrationStart == -1 && (terminalConfirm || nextLen > 0)) {
        if (debugControl) {
          Serial.println(F("Performing occupied calibration"));
        }
        makeLedAck(&blinkTwice[0]);
        cfgState = CONFIG_CALIBRATE_HIGH;
        setupCalibration();
      }
      return;
  }
}
