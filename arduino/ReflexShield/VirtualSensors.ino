

VirtualSensor& VirtualSensor::find(int id) {
  for (int i = 0; i < maxVirtualSensors; i++) {
    if (virtualSensors[i].monitoredId == id) {
      return virtualSensors[i];
    }
  }
}

void processVirtualSensorInputs() {
  for (int i = 0; i < virtualSensorCount; i++) {
    updateVirtualSensor(i);
  }
}

void updateVirtualSensor(int vIndex) {
  if (vIndex < 0 || vIndex >= maxVirtualSensors) {
    return;
  }
  int sensorId = virtualSensors[vIndex].monitoredId;
  if (sensorId == 0) {
    return;
  }
  VirtualSensor& sens = virtualSensors[vIndex];
  VirtualSensorState& ss = virtualSensorStates[vIndex];
  
  unsigned long m = (1L << vIndex);
  boolean state = (capturedSensorStates & m) > 0;

  boolean prevState = (knownSensorStates & m) > 0;
  if (state == prevState) {
    return;
  }

  if (debugS88 || debugVirtual) {
    Serial.print(F("Sensor #"));  Serial.print(vIndex);  Serial.print('/'); Serial.print(sensorId); Serial.print(F(" => ")); Serial.println(state);
  }

  if (state) {
    knownSensorStates |= m;
  } else {
    knownSensorStates &= ~m;
  }
  if (debugVirtual) {
    Serial.print(F("Known sensor bits = ")); printBinaryData(knownSensorStates, numBufferedBits); Serial.println();
  }

  unsigned long v = 1 << (vIndex + 8);

  if (state) {
    if (debugVirtual) {
      Serial.println(F("Virutal Armed"));
    }
    ss.fadeOnTime = 0;
    ss.fadeOffTime = 0;
    sensorStateBits &= ~v;

    Serial.print(F("State bits = ")); printBinaryData(sensorStateBits, numBufferedBits); Serial.println();
    return;
  }

  long t = millis();
  if (ss.fadeOnTime == 0) {
    if (debugVirtual) {
      Serial.println(F("Virtual timeout reset"));
    }
    ss.fadeOnTime = t + sens.getSensorDelay();
    ss.fadeOffTime = 0;
  } else {
    if (t > ss.fadeOnTime) {
      sensorStateBits |= v;
      ss.fadeOffTime = t + virtualSensorFadeOffDelay;
      if (debugVirtual) {
        Serial.println(F("Fadeon timeout elapsed"));
        Serial.print(F("State bits = ")); printBinaryData(sensorStateBits, numBufferedBits); Serial.println();
      }
    }
  }
}

void fadeOffVirtualSensors() {
  long t = millis();
  for (byte i = 0; i < virtualSensorCount; i++) {
    if (virtualSensors[i].isFree()) {
      continue;
    }
    unsigned long v = 1 << (i + 8);
    VirtualSensorState& state = virtualSensorStates[i];
    
    if ((sensorStateBits & v) == 0) {
      if (state.fadeOnTime > 0) {
        if (t > state.fadeOnTime) {
          sensorStateBits |= v;
          state.fadeOffTime = t + virtualSensorFadeOffDelay;
          state.fadeOnTime = 0;
          if (debugVirtual) {
            Serial.println(F("Fadeon timeout elapsed"));
            Serial.print(F("State bits = ")); Serial.print(sensorStateBits); Serial.print("  "); printBinaryData(sensorStateBits, numBufferedBits); Serial.println();
          }
        }
      }
      continue;
    } else if (state.fadeOffTime > 0) {
      if (t > state.fadeOffTime) {
        sensorStateBits = (sensorStateBits & ~v);
        state.fadeOffTime = 0;
        if (debugVirtual) {
          Serial.println(F("Fade-off timeout elapsed"));
          Serial.print(F("State bits = ")); printBinaryData(sensorStateBits, numBufferedBits); Serial.println();
        }
      }
    }
  }
}
