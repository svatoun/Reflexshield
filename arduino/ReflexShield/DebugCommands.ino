

void commandSetSensor() {
  int n = nextNumber();
  if (n < 1 || n > numChannels) {
    Serial.println(F("Invalid channel"));
    return;
  }
  int v = (1 << (n - 1));
  if (*inputPos == 'x') {
    sensorOverrides = (sensorOverrides & ~v);
    Serial.print(F("Sensor override removed: ")); Serial.println(n);
  }
  int s = nextNumber();
  if (s < 0 || s > 1) {
    Serial.println(F("Invalid state"));
    return;
  }
  // own sensors will set the state immediately
  if (n <= 8) {
    sensorOverrides |= v;
    if (s) {
      sensorStateBits |= v;
    } else {
      sensorStateBits &= ~v;
    }
  }
  captureSensorBit(n, (s > 0) ? 1 : 0);
}

void commandS88Load() {
  Serial.println(F("DEBUG: s88 Load int"));
  loadInt();
}

void commandS88Data() {
  unsigned long m = 0x01;
  unsigned long d = 0x00;

  int count = 0;
  while (*inputPos) {
    switch (*inputPos) {
      case 'H': case '+': case '1':
        d |= m;
        break;
      case 'L': case '-': case '0':
        break;
      default:
        Serial.println(F("Invalid value"));
        return;
    }
    m <<= 1;
    count++;
    inputPos++;
  }
  Serial.print(F("Injecting bits: ")); Serial.println(count);
  s88InputData = d;
  s88InputDataLength = count;
}

void commandS88Tick() {
  Serial.println(F("DEBUG: s88 tick"));
  clockInt();
}
