#ifndef HOST_ARDUINO_H
#define HOST_ARDUINO_H
#include <stdint.h>
#include <string>
#include <sstream>
typedef unsigned char byte;
struct __FlashStringHelper {};
#define F(x) reinterpret_cast<const __FlashStringHelper*>(x)
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
static unsigned long fakeNow=0;
static int analogPins[70]={0};
static int inputPins[70]={0};
inline unsigned long millis(){return fakeNow;}
inline void delay(unsigned long ms){fakeNow+=ms;}
inline void pinMode(byte p,int mode){if(mode==INPUT_PULLUP)inputPins[p]=HIGH;}
inline void digitalWrite(byte,int){}
inline int digitalRead(byte p){return inputPins[p];}
inline void analogWrite(byte p,int v){analogPins[p]=v;}
inline void noInterrupts(){}
inline void interrupts(){}
struct SerialStub {
  std::ostringstream output;
  void begin(int){}
  int available(){return 0;}
  int read(){return -1;}
  void print(const __FlashStringHelper* s){output<<reinterpret_cast<const char*>(s);}
  void print(unsigned char v){output<<(int)v;}
  template<class T> void print(T v){output<<v;}
  template<class T> void println(T v){print(v);output<<'\n';}
};
static SerialStub Serial;
#endif
