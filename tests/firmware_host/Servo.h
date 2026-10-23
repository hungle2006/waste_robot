#pragma once
struct Servo {
  bool active=false;
  int pin=-1;
  int pulse=0;
  bool attached(){return active;}
  void attach(unsigned char p,int,int){active=true;pin=p;}
  void writeMicroseconds(int us){pulse=us;}
  void detach(){active=false;}
};
