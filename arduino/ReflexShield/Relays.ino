
#include "Relays.h"

Relay relays[maxRelays];
RelayState relayStates[maxRelays];

boolean isRelayDefined(const Relay& r) {
  return r.sensorA > 0 || r.sensorB > 0 || r.sensorC > 0;
}

boolean atLeastOneRelayConnected() {
  for (int i = 0; i < maxRelays; i++) {
    if (isRelayDefined(relays[i])) {
      return true;
    }
  }
  return false;
}

void initRelays() {
  for (int i = 0; i < maxRelays; i++) {
    if (isRelayDefined(relays[i])) {
      pinMode(relayPorts[i], OUTPUT);
      boolean x = (relayStates[i].state == relays[i].onValue) ? HIGH : LOW;
      Serial.print(F("Init relay ")); Serial.print(i + 1); Serial.print(F(" to ")); Serial.println(x);
      digitalWrite(relayPorts[i], x);
      relayStates[i].offTimeout = 0;

      relaysPresent = true;
    }
  }
}

void commandRelay() {
  int rel = nextNumber();
  if (rel < 1 || rel > maxRelays) {
    Serial.println(F("Bad relay"));
    return;
  }
  Relay rcopy = relays[rel - 1];
  char c = 0;
  if ((*inputPos != 0) && (*(inputPos + 1) != '=')) {
    if (*inputPos == '0') {
      relays[rel - 1] = Relay();
      relayStates[rel - 1] = RelayState();
      initRelays();
      Serial.print(F("Relay ")); Serial.print(rel); Serial.println(F(" erased."));
      return;
    }
  }
  
  while (*inputPos != 0) {
    if (*(inputPos + 1) != '=') {
        break;
    }
    int sen = -1;
    int offDelay = -1;
    boolean state = true;
    switch (*inputPos) {
      case 'a': case 'b': case 'c':
      case 'A': case 'B': case 'C':
        c = tolower(*inputPos);
        inputPos += 2;
        sen = nextNumber();
        if (sen < 0 || sen > 8) {
          Serial.println(F("Invalid sensor"));
          return;
        }
        break;

      case 't': case 'T':
        c = tolower(*inputPos);
        inputPos += 2;
        offDelay = nextNumber();
        if (offDelay < 0 || offDelay > 30000) {
          Serial.println(F("Invalid delay"));
          return;
        }
        rcopy.offDelay = offDelay; 
        break;
      case 's': case 'S':
        inputPos += 2;
        switch (*inputPos) {
          case '-': case '0':
            state = false;
            break;
          case '+': case '1':
            state = true;
            break;
          case '=':
            if (*(inputPos + 1) == '0') {
              state = false;
              break;
            } else if (*(inputPos + 1) == '1') {
              state = true;
              break;
            }
            // fall through
          default:
            Serial.println(F("Invalid state"));
            return;
        }
        rcopy.onValue = state;
        break;
      default:
        Serial.print(F("Invalid property: ")); Serial.println(*inputPos);
        return;
    }
     
    switch (c) {
      case 'a': rcopy.sensorA = sen; break;
      case 'b': rcopy.sensorB = sen; break;
      case 'c': rcopy.sensorC = sen; break;
    }
  }
  if (rcopy.sensorA == 0 && rcopy.sensorB == 0 && rcopy.sensorC == 0) {
      relays[rel - 1] = Relay();
      relayStates[rel - 1] = RelayState();
      initRelays();
      Serial.print(F("Relay ")); Serial.print(rel); Serial.println(F(" erased."));
      return;
  }
  Relay& r = relays[rel - 1];
  r = rcopy;
  Serial.println(F("Relay defined:"));
  dumpRelay(rel - 1, r);
}

void dumpRelay(int index, const Relay& r) {
  if (r.sensorA == 0 && r.sensorB == 0 && r.sensorC == 0) {
    return;
  }
  Serial.print(F("REL:")); Serial.print(index + 1); 
  if (r.sensorA > 0) {
    Serial.print(':'); Serial.print('A'); Serial.print('='); Serial.print(r.sensorA);
  }
  if (r.sensorB > 0) {
    Serial.print(':'); Serial.print('B'); Serial.print('='); Serial.print(r.sensorB);
  }
  if (r.sensorC > 0) {
    Serial.print(':'); Serial.print('C'); Serial.print('='); Serial.print(r.sensorC);
  }
  if (r.offDelay > 0) {
    Serial.print(':'); Serial.print('T'); Serial.print('='); Serial.print(r.offDelay);
  }
  Serial.print(':'); Serial.print('S'); Serial.print('='); Serial.print(r.onValue ? '1' : '0');
}

void dumpRelays() {
  for (int i = 0; i < maxRelays; i++) {
    const Relay &r = relays[i];
    if (r.sensorA == 0 && r.sensorB == 0 && r.sensorC == 0) {
      continue;
    }
    dumpRelay(i, r);
    Serial.println();
  }
}

void processRelays() {
  for (int i = 0; i < maxRelays; i++) {
    const Relay& r = relays[i];
    if (!isRelayDefined(r)) {
      continue;
    }

    RelayState& rs = relayStates[i];
    int mask = 0;
    if (r.sensorA > 0) {
      mask |= (1 << (r.sensorA - 1));
    }
    if (r.sensorB > 0) {
      mask |= (1 << (r.sensorB - 1));
    }
    if (r.sensorC > 0) {
      mask |= (1 << (r.sensorC - 1));
    }
    boolean on = (sensorStateBits & mask) > 0;
    boolean x = (on == relays[i].onValue) ? HIGH : LOW;

    if (on) {
      if (on != rs.state) {
        Serial.print(F("Activate relay ")); Serial.print(i + 1); Serial.print(F(" to ")); Serial.println(x);        
        rs.state = on;
      }
      rs.offTimeout = 0;
      digitalWrite(relayPorts[i], x);
      digitalWrite(relayPorts[i], x);
    } else if (on != rs.state) {
      if (rs.offTimeout == 0) {
        if (r.offDelay > 0) {
          rs.offTimeout = curMillis + r.offDelay;
          return;
        }
      } else if (curMillis < rs.offTimeout) {
          return;
      } 
      rs.offTimeout = 0;
      rs.state = on;
      Serial.print(F("Deactivate relay ")); Serial.print(i + 1); Serial.print(F(" to ")); Serial.println(x);        
      digitalWrite(relayPorts[i], x);
    }
  }
}
