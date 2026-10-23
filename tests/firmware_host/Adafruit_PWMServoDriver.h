#pragma once
struct Adafruit_PWMServoDriver {
  Adafruit_PWMServoDriver(int){}
  bool begin(){return true;}
  void setPWMFreq(int){}
  unsigned char readPrescale(){return 121;}
  int setPWM(unsigned char,int,int){return 0;}
};
