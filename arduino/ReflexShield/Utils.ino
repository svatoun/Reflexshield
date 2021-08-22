#include <EEPROM.h>

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
  //  Serial.print(v & 0xff, HEX); Serial.print("-"); Serial.print((v >> 8) & 0xff, HEX); Serial.print(" ");
  if (v != 0) {
    allzero = false;
  }
  //  Serial.print(F(" = ")); Serial.println(v);
  return v;
}

/**
   Writes a block of data into the EEPROM, followed by a checksum
*/
void eeBlockWrite(byte magic, int eeaddr, const void* address, int size) {
  if (debugControl) {
    Serial.print(F("Writing EEPROM ")); Serial.print(eeaddr, HEX); Serial.print('-'); Serial.print(eeaddr + size - 1, HEX); Serial.print(F(":")); Serial.print(size);  Serial.print(F(", source: ")); Serial.println((int)address, HEX);
  }
  const byte *ptr = (const byte*) address;
  byte hash = magic;
  EEPROM.write(eeaddr, magic);
  eeaddr++;
  for (; size > 0; size--) {
    byte b = *ptr;
    EEPROM.write(eeaddr, b);
    ptr++;
    eeaddr++;
    hash = hash ^ b;
  }
  EEPROM.write(eeaddr, hash);
}

void eeBlockRead2(int eeaddr, void* address, int size) {
  if (debugControl) {
    Serial.print(F("Reading EEPROM ")); Serial.print(eeaddr, HEX); Serial.print(F(":")); Serial.print(size); Serial.print(F(", dest: ")); Serial.println((int)address, HEX);
  }
  int a = eeaddr;
  byte x;
  byte *ptr = (byte*) address;
  for (int i = 0; i < size; i++, a++) {
    x = EEPROM.read(a);
    *ptr = x;
    ptr++;
  }
}

/**
   Reads block of data from the EEPROM, but only if the checksum of
   that data is correct.
*/
boolean eeBlockRead(byte magic, int eeaddr, void* address, int size) {
  if (debugControl) {
    Serial.print(F("Reading EEPROM ")); Serial.print(eeaddr, HEX); Serial.print(F(":")); Serial.print(size); Serial.print(F(", dest: ")); Serial.println((int)address, HEX);
  }
  int a = eeaddr;
  byte hash = magic;
  byte x;
  x = EEPROM.read(a);
  if (x != magic) {
    if (debugControl) {
      Serial.println(F("No magic header found"));
    }
    return false;
  }
  a++;
  for (int i = 0; i < size; i++, a++) {
    x = EEPROM.read(a);
    hash = hash ^ x;
  }
  x = EEPROM.read(a);
  if (hash != x) {
    if (debugControl) {
      Serial.println(F("Checksum does not match"));
    }
    return false;
  }

  a = eeaddr + 1;
  byte *ptr = (byte*) address;
  for (int i = 0; i < size; i++, a++) {
    x = EEPROM.read(a);
    *ptr = x;
    ptr++;
  }
}
