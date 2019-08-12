#ifndef __common_h__
#define __common_h__

struct LineCommand {
  const char* cmd;
  void (*handler)();

  LineCommand() : cmd(NULL), handler(NULL) {}
};

void registerLineCommand(const char* cmd, void (*aHandler)());


enum ModuleCmd {
  initialize,
  eepromLoad,
  eepromSave,
  status,
  reset,
  dump,
  periodic
};

struct ModuleChain {
  static ModuleChain* head;
  
  byte priority;
  ModuleChain *next;
  void (*handler)(ModuleCmd);

  ModuleChain(const char* n, byte priority, void (*h)(ModuleCmd));

  static void invokeAll(ModuleCmd cmd);
};

extern char printBuffer[];

 __attribute__((always_inline)) char* append(char* &ptr, char c) {
  *(ptr++) = c;
  return ptr;
}

__attribute__((noinline)) char* initPrintBuffer() {
  printBuffer[0] = 0;
  return printBuffer;
}

__attribute__((noinline)) char *printNumber(char *out, int no, int base) {
  itoa(no, out, base);
  return out + strlen(out);
}

extern char *inputPos;
extern char *inputEnd;


#endif

