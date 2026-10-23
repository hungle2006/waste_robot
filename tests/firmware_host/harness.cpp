#include "Arduino.h"
#include <cassert>
#define __AVR_ATmega2560__ 1
#include "../../arduino/robot_control_v3/robot_control_v3.ino"
void send(const char* text){char line[80];strncpy(line,text,79);line[79]=0;handle(line);}
bool zero(){for(int i=0;i<4;i++)if(motors[i])return false;return true;}
int main(){
  setup();assert(stopped && zero());
  assert(TOF_ROI_SIZE==10 && tofSensor.roiWidth==10 && tofSensor.roiHeight==10 && tofSensor.roiCenter==199);
  assert(tofMinMm==200 && tofMaxMm==250);
  assert(GIFT_PIN==4 && !giftServo.attached());
  for(byte i=0;i<4;i++)assert(RP[i]!=GIFT_PIN && LP[i]!=GIFT_PIN);
  for(byte i=0;i<INPUT_COUNT;i++)assert(INPUTS[i]!=GIFT_PIN);
  send("1 RESUME");
  for(int peak=1;peak<=255;peak+=7){
    int last=0;
    for(int t=0;t<=120;t++){int p=smoothPwm(peak,t,120,0);assert(p>=last && p<=peak);last=p;}
    assert(last==peak);
    for(int t=120;t<=240;t++){int p=smoothPwm(peak,t,120,0);assert(p<=last && p>=0);last=p;}
    assert(last==0);
  }
  send("2 G 100 0 0 20 120 0");unsigned long start=fakeNow;
  assert(smoothActive && zero());fakeNow=start+60;loop();assert(motors[0]==-10 && motors[3]==10);
  fakeNow=start+120;loop();assert(motors[0]==-20 && motors[1]==-20 && motors[2]==-20 && motors[3]==20);
  send("3 C -100 0 0 20 100");assert(smoothActive); // Cannot replace a running ramp.
  fakeNow=start+180;loop();assert(motors[0]==-10);
  fakeNow=start+240;loop();assert(zero() && !smoothActive);
  send("4 G 100 0 0 20 120 0");fakeNow+=60;loop();assert(!zero());
  send("5 XM");assert(zero() && !smoothActive);fakeNow+=200;loop();assert(zero());
  send("6 G 100 0 0 20 120 0");fakeNow+=60;loop();inputPins[19]=raw[3]=stable[3]=LOW;
  fakeNow+=10;loop();assert(zero() && !smoothActive);inputPins[19]=raw[3]=stable[3]=HIGH;
  send("7 G 100 0 0 20 120 0");fakeNow+=60;loop();inputPins[18]=raw[2]=stable[2]=LOW;
  fakeNow+=10;loop();assert(zero() && !smoothActive);inputPins[18]=raw[2]=stable[2]=HIGH;
  send("8 G 100 0 0 20 120 0");fakeNow+=60;loop();send("9 STOP");
  assert(stopped && zero() && !smoothActive);fakeNow+=200;loop();assert(zero());
  send("10 G 100 0 0 20 120 0");assert(!smoothActive);
  send("11 RESUME");send("12 G 100 0 0 20 0 0");assert(!smoothActive);
  send("13 G 100 0 0 20 120 0");fakeNow+=2100;loop();assert(stopped && zero());
  send("14 STATUS");assert(Serial.output.str().find("\"smooth_protocol\":1")!=std::string::npos);
  // V3.14: real sketch's distance gate, with synthetic I2C readings.
  assert(tofPresent);send("15 RESUME");
  tofSensor.mm=800;tofSensor.ready=true;fakeNow+=60;loop();
  assert(tofValid && tofSeq>0 && tofMm==800);
  send("16 TF 250 300");assert(tofGuard && tofWindowCount==0);
  send("17 G 100 0 0 20 80 0");fakeNow+=40;loop();assert(!zero());
  tofSensor.mm=280;tofSensor.ready=true;fakeNow+=10;loop();
  assert(zero() && !smoothActive && tofWindowCount==1);
  for(int i=0;i<2;i++){tofSensor.ready=true;fakeNow+=60;loop();}
  assert(tofWindowCount==3);
  send("18 G 100 0 0 20 80 0");assert(!smoothActive && zero());
  tofSensor.ranging_data.range_status=VL53L1X::Invalid;tofSensor.ready=true;fakeNow+=60;loop();
  assert(!tofValid && tofWindowCount==0);
  send("19 G 100 0 0 20 80 0");assert(!smoothActive);
  tofSensor.ranging_data.range_status=VL53L1X::RangeValid;tofSensor.mm=800;
  tofSensor.ready=true;fakeNow+=60;loop();send("20 G 100 0 0 20 160 240");
  fakeNow+=80;loop();assert(!zero());fakeNow+=180;loop();assert(zero() && !smoothActive);
  send("21 G 100 0 0 20 80 0");assert(!smoothActive); // stale sample
  send("22 TF OFF");assert(!tofGuard);send("23 G 100 0 0 20 80 0");assert(smoothActive);
  send("24 STOP");assert(stopped && !tofGuard && zero());
  send("25 STATUS");assert(Serial.output.str().find("\"tof_protocol\":1")!=std::string::npos);

  // V3.16.4: only AUTO ignores missing/invalid/stale ranges; strict TF unchanged above.
  send("26 RESUME");tofPresent=false;tofValid=false;tofSeq=0;
  send("27 TF AUTO 250 300");assert(tofGuard && tofIgnoreInvalid);
  send("28 G 100 0 0 20 80 0");fakeNow+=40;loop();assert(!zero());
  send("29 XM");tofPresent=true;tofValid=false;tofMm=280;tofSeq=1;
  send("30 G 100 0 0 20 80 0");fakeNow+=40;loop();assert(!zero());
  tofSensor.ranging_data.range_status=VL53L1X::RangeValid;
  tofSensor.mm=280;tofSensor.ready=true;fakeNow+=10;loop();assert(zero());
  send("31 G 100 0 0 20 80 0");assert(!smoothActive);
  tofMm=800;tofValid=true;tofAt=fakeNow-5000;
  send("32 G 100 0 0 20 80 0");fakeNow+=40;loop();assert(!zero());
  send("33 STOP");assert(stopped && zero() && !tofIgnoreInvalid);
  send("34 RESUME");send("35 TF AUTO 250 300");
  tofValid=false;send("36 G 100 0 0 20 80 0");fakeNow+=40;loop();assert(!zero());
  inputPins[19]=raw[3]=stable[3]=LOW;fakeNow+=10;loop();assert(zero());
  inputPins[19]=raw[3]=stable[3]=HIGH;
  send("37 G 100 0 0 20 80 0");fakeNow+=40;loop();assert(!zero());
  inputPins[18]=raw[2]=stable[2]=LOW;fakeNow+=10;loop();assert(zero());
  // D4 accepts exactly 500..2500 us; boot, STOP and RESUME never auto-open it.
  send("43 STOP");assert(!giftServo.attached() && pulses[7]==0);
  send("44 RESUME");assert(!giftServo.attached());
  send("45 S 4 500");assert(giftServo.attached() && pulses[7]==500);
  send("46 S 4 2500");assert(pulses[7]==2500);
  assert(giftServo.pin==4 && giftServo.pulse==2500);
  send("52 P 4 1600");assert(pulses[4]==1600 && pulses[7]==2500);
  send("53 S 4 600");assert(pulses[4]==1600 && giftServo.pulse==600);
  send("54 S 4 2500");
  send("55 S 22 600");assert(pulses[7]==2500 && giftServo.pin==4);
  send("47 S 4 499");assert(pulses[7]==2500);
  send("48 S 4 2501");assert(pulses[7]==2500);
  send("49 STATUS");
  assert(Serial.output.str().find("\"roi_size_configured\":10")!=std::string::npos);
  assert(Serial.output.str().find("\"gift_protocol\":3")!=std::string::npos);
  assert(Serial.output.str().find("\"d4\":2500")!=std::string::npos);
  send("50 STOP");assert(!giftServo.attached() && pulses[7]==0);
  send("51 RESUME");assert(!giftServo.attached());
}
