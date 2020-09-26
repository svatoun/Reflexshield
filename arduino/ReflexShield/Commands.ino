
void commandClear() {
  resetEEPROM();
  commandReset();
}

/**
 * Vypise stav senzoru
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
  sensorThresholds[channel] = threshold;
  sensorDebounces[channel] = debounce;
}

void commandSave() {
  writeEEPROM();
  Serial.println(F("Saved."));
}

void commandDump() {
  for (int i = 0; i < numChannels; i++) {
    int sens = sensorThresholds[i];
    int deb = sensorDebounces[i];
    Serial.print("SEN:"); Serial.print(i + 1); Serial.print(':'); Serial.print(sens);
    if (deb != defaultDebounce) {
      Serial.print(':'); Serial.print(deb);
    }
    Serial.println();
  }
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
    count +=2 ;
  }
  Serial.println();
  for (int i = 1; i < count; i++) {
    Serial.print('=');
  }
  Serial.println();
  lastMonitorTime = 0;
  charModeCallback = &monitorCallback;
}
