
extern VirtualSensor virtualSensors[maxVirtualSensors];

void commandClear() {
  resetEEPROM();
  commandReset();
}

/**
   Vypise stav senzoru
*/
void commandInfo() {
  Serial.println(F("Sensor status:"));
  printSensorStatus();
  Serial.println();
}

int sensorStatusLen = 0;

void printSensorStatus() {
  byte b = sensorStateBits;
  sensorStatusLen = 0;
  for (int mask = 0x80; mask > 0; mask >>= 1) {
    Serial.print(b & mask ? "1" : "0");
    sensorStatusLen++;
    if (mask > 1) {
      sensorStatusLen++;
      Serial.print("-");
    }
  }
}

void calibrationBlockKeys(char x) {
  if (x == 0) {
    exitConfigurationNoReload();
    return;
  }
  configLastCommand = millis();
  if (x == '\n' || x == '\r') {
    switch (cfgState) {
      case CONFIG_CALIBRATE_LOW:
      case CONFIG_CALIBRATE_HIGH:
        nextLen = 5;
        terminalConfirm = true;
        break;
    }
  }
}

void printCalibrationStats() {
  Serial.print(F("Calibrating: "));
  Serial.print(F("Min = ")); Serial.print(calibrationMin);
  int cnt = calibrationCount == 0 ? 1 : calibrationCount;
  Serial.print(F(", \tAvg = ")); Serial.print(calibrationSum / calibrationCount);
  Serial.print(F(", \tMax = ")); Serial.print(calibrationMax);
  Serial.print(F("      "));
  for (short i = 0; i < 100; i++) {
    Serial.print("\b");
  }
  Serial.print("\r");
}

void commandContinueCalibration() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  long m = millis();
  if (calibrationSum <= 0 ||
      ((m - (calibrationStart + calibrationTime)) > 10000)) {
    Serial.println(F("No preceding CAL found."));
  }
  configChannel = channel - 1;
  terminalCalibration = true;
  charModeCallback = &calibrationBlockKeys;
  startCalibrationHigh();
}

void commandCalibrate() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  configChannel = channel - 1;
  terminalCalibration = true;
  charModeCallback = &calibrationBlockKeys;
  startCalibration();
}

void commandSensitivity() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  int threshold = nextNumber();
  if (threshold < 0 || threshold > 1000) {
    Serial.println(F("Bad threshold"));
    return;
  }
  int debounce = nextNumber();
  if (debounce == -2) {
    debounce = defaultDebounce;
  }
  channel--;
  AttachedSensor &as = attachedSensors[channel];
  
  as.threshold = threshold;
  as.debounce = debounce;
  printSensorDef(channel, as);
}

void commandFadeTime() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  int fadeOn = nextNumber();
  if (fadeOn < 0 || fadeOn > 30000) {
    Serial.println(F("Bad fadeOn"));
    return;
  }
  int fadeOff = nextNumber();
  if (fadeOff < 0 || fadeOff > 30000) {
    Serial.println(F("Bad fadeOff"));
    return;
  }

  AttachedSensor &as = attachedSensors[channel];
  as.fadeOnTime = fadeOn;
  as.fadeOffTime = fadeOff;
}

void commandReduction() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  int percent = nextNumber();
  if ((percent != 0 && percent < 10) || percent > 100) {
    Serial.println(F("Bad threshold"));
    return;
  }

  AttachedSensor &as = attachedSensors[channel];
  as.occupiedReduction = percent;
}

void printSensorDef(int i, const AttachedSensor& as) {
    int sens = as.threshold;
    int deb = as.debounce;
    Serial.print("SEN:"); Serial.print(i + 1); Serial.print(':'); Serial.print(sens);
    if (deb != defaultDebounce) {
      Serial.print(':'); Serial.print(deb);
    }
    Serial.println();
    if (as.invert) {
      Serial.print("INV:"); Serial.println(i + 1);
    }

    if (as.fadeOnTime > 0 || as.fadeOffTime > 0) {
      Serial.print("SFT:"); Serial.print(as.fadeOnTime); Serial.print(':'); Serial.println(as.fadeOffTime);
    }
    if (as.occupiedReduction > 0) {
      Serial.print("OTR:"); Serial.println(as.occupiedReduction); 
    }
}

void commandSave() {
  writeEEPROM();
  Serial.println(F("Saved."));
}

void commandDump() {
  for (int i = 0; i < numChannels; i++) {
    const AttachedSensor as = attachedSensors[i];
    printSensorDef(i, as);
  }
  for (int i = 0; i < maxVirtualSensors; i++) {
    const VirtualSensor& s = virtualSensors[i];
    int id = s.monitoredId;
    if (id == 0) {
      continue;
    }
    int del = s.sensorDelay;
    Serial.print(F("VSN:")); Serial.print(i + 1);
    Serial.print(':'); Serial.print(id);
    Serial.print(':'); Serial.println(del);
  }
  dumpRelays();
}

void monitorCallback(char c) {
  switch (c) {
    case '\n':
      Serial.println("\n");
      break;
    case 'q':
      Serial.println(F("\nMonitoring stopped.\n"));
      resetTerminal();
      break;
  }
}

void commandCalibrateDebounce() {
  int ch = nextNumber();
  if (ch < 1 || ch > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  ch--;
  configChannel = ch;
  cfgState = CONFIG_CALIBRATE_DEBOUNCE;
  setupCalibration();
  maxConfigSensorDebounce = 0;
  terminalCalibration = true;
  charModeCallback = &calibrationBlockKeys;
}

const int monitorTimeThreshold = 200;
long lastMonitorTime = 0;

void handleMonitor() {
  if (charModeCallback != &monitorCallback) {
    return;
  }

  long t = millis();
  if ((t - lastMonitorTime) < monitorTimeThreshold) {
    return;
  }

  lastMonitorTime = t;
  printSensorStatus();
  for (int i = 0; i < sensorStatusLen; i++) {
    Serial.print("\b");
  }
  Serial.print("\r");
}

void commandMonitor() {
  Serial.println(F("Monitoring sensors"));
  for (int i = 1; i <= numChannels; i++) {
    if (i < 10) {
      Serial.print(' ');
    } else {
      Serial.print(i / 10);
    }
    Serial.print(' ');
  }
  Serial.println();
  int count = 0;
  for (int i = numChannels; i > 0; i--) {
    Serial.print(i % 10);
    Serial.print(' ');
    count += 2 ;
  }
  Serial.println();
  for (int i = 1; i < count; i++) {
    Serial.print('=');
  }
  Serial.println();
  lastMonitorTime = 0;
  charModeCallback = &monitorCallback;
}

void commandInvert() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  int flag = nextNumber();
  boolean v = false;

  if (flag < -1 || flag > 0) {
    v = true;
  }
  attachedSensors[channel - 1].invert = v;
  if (v) {
    Serial.println(F("Channel inverted."));
  } else {
    Serial.println(F("Channel back to normal."));
  }
}

void commandVirtualDelay() {

  int v = nextNumber();

  if (v == -1) {
    Serial.println(F("Invalid delay."));
    return;
  }

  if (v < -1) {
    v = 1000;
  }
  if (v) {
    Serial.print(F("Virtual sensor delay: ")); Serial.println(v);
  } else {
    Serial.println(F("Virtual sensor OFF."));
  }
  virtualSensorDefaultDelay = v;
}

void refreshSensorCount() {
  int maxSensor = 0;
  for (int i = 0; i < maxVirtualSensors; i++) {
    const VirtualSensor& s = virtualSensors[i];
    if (s.monitoredId > 0) {
      maxSensor = i + 1;
    }
  }
  if (maxSensor > 8) {
    virtualSensorCount = 16;
  } else if (maxSensor > 0) {
    virtualSensorCount = 8;
  } else {
    virtualSensorCount = 0;
  }
}

void measureCallback(char c) {
  switch (c) {
    case '\r':
    case '\n':
      Serial.println("\n\n");
      resetCalibrationStats();
      break;
    case 'q':
      Serial.println(F("\nMeasurement stopped.\n"));
      configChannel = -1;
      resetTerminal();
      break;
    case 'r':
      resetCalibrationStats();
      break;     
  }
}

void print3(int val) {
  if (val <= 9) {
    Serial.print('0');
  }
  if (val <= 99) {
    Serial.print('0');
  }
  Serial.print(val);
}

void handleMeasure() {
  if (charModeCallback != &measureCallback) {
    return;
  }
  Serial.print('\r');
  Serial.print(configChannel + 1); 
  Serial.print(F(": high=")); print3(resultHigh[configChannel]);
  Serial.print(F(", low=")); print3(resultLow[configChannel]);
  Serial.print(F(", min=")); print3(calibrationMin);
  Serial.print(F(", max=")); print3(calibrationMax);
  Serial.print(F(", avg=")); print3(calibrationCount == 0 ? 0 : calibrationSum / calibrationCount);
  Serial.print(F("      "));
}

void commandMeasure() {
  int channel = nextNumber();
  if (channel < 1 || channel > numChannels) {
    Serial.println(F("Bad channel"));
    return;
  }
  configChannel = channel - 1;
  charModeCallback = &measureCallback;
  resetCalibrationStats();
}

void commandVirtualSensor() {
  int vsIndex = nextNumber();
  if (vsIndex < -1) {
    for (int i = 0; i < maxVirtualSensors; i++) {
      const VirtualSensor& s = virtualSensors[i];
      if (s.monitoredId == 0) {
        vsIndex = i + 1;
        break;
      }
    }
  }
  if (vsIndex <= 0 || vsIndex > maxVirtualSensors) {
    Serial.println(F("Invalid index"));
    return;
  }
  vsIndex--;

  int id = nextNumber();
  if (id < 0 || id > 255) {
    Serial.println(F("Invalid sensor ID"));
    return;
  }
  if (id == 0) {
    virtualSensors[vsIndex] = VirtualSensor();
    refreshSensorCount();
    Serial.print(F("VSensor cleared: ")); Serial.println(vsIndex + 1);
    Serial.print(F("Buffering bits: ")); Serial.println(virtualSensorCount + 8);
    return;
  }
  int d = nextNumber();
  if (d <= -2) {
    d = virtualSensorDefaultDelay;
  }
  if (d < 10 || d > 20000) {
    Serial.println(F("Invalid delay"));
    return;
  }
  virtualSensors[vsIndex] = VirtualSensor(id, d);
  refreshSensorCount();
  Serial.print(F("VSensor defined: ")); Serial.print(vsIndex + 1); Serial.print(F(" = ")); Serial.println(id);
  Serial.print(F("Buffering bits: ")); Serial.println(virtualSensorCount + 8);
}
