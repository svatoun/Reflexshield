#ifndef __relays_h__
#define __relays_h__

const int maxRelays = 3;

byte relayPorts[maxRelays]= {6, 7, 8};

struct Relay {
  byte sensorA;
  byte sensorB;
  byte sensorC;
  short offDelay;
  boolean onValue;
  
  Relay() : sensorA(0), sensorB(0), sensorC(0), offDelay(0), onValue(false) {}
};

struct RelayState {
  boolean state;
  long offTimeout;

  RelayState() : state(false), offTimeout(0) {}
};

extern Relay relays[maxRelays];

#endif
