/* Waste Robot V3.16.11 BOX + VL53L1X AUTO. Arduino Mega 2560 ONLY.
   Python owns all poses. Firmware accepts S2/P1 500..3500us, arm/link/grip joints500..2500us; gift D4 500..2500us.
   M1=7/8 M2=5/6 M3=11/12 M4=9/10. D2=link D3=grip D4=gift (no boot motion).
   PCA 0x40 SDA20 SCL21; P1,P10,P11,P9,P4 only.
   IR=D19, bin sensor=D18 active-low. D15/D16 remain optional legacy inputs.
   PCA9685 channel 15 is PWM output and cannot read the IR OUT signal.
   Protocol: requestId COMMAND -> requestId OK / ERR / STATUS JSON.
   Example: 1 RESUME, 2 P 1 1300, 3 S 2 500, 4 STATUS, 5 STOP.
   Starts latched STOP; RESUME does not move servos.
   No command >2000ms: motor off, servo pulses off, latch STOP.
   Pulse status is the last commanded value, NOT position feedback.
   M1 front-right forward<0; M2 rear-left forward<0;
   M3 rear-right forward<0; M4 front-left forward>0. X rollers.
   C FORWARD RIGHT CLOCKWISE PWM MS -> normalized Mecanum drive.
   D M1 M2 M3 M4 MS -> independent raw PWM for bench testing.
   G F R CW PWM RAMP_MS HOLD_MS -> finite smooth profile; total<=1000ms.
   XM/STOP/watchdog bypass the ramp and stop immediately.
   VL53L1X 0x29 shares SDA20/SCL21 with PCA0x40. Pololu library >=1.3.0.
   Short, ROI10x10 center199, budget50ms/period60ms. Nonblocking dataReady/read(false).
   TF MIN_MM MAX_MM arms a distance wheel gate; TF OFF disarms it.
   While armed, invalid/stale or distance<=MAX_MM immediately brakes wheels.
   STATUS contains tof_protocol=1 and tof {present,valid,mm,status,seq,age_ms,...}.
   C axes -100..100, PWM 1..255, lease 50..500ms; old V/W rejected.
*/
#if !defined(__AVR_ATmega2560__)
#error Select Arduino Mega 2560. Motor pins require the Mega timer map.
#endif
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_PWMServoDriver.h>
#include <VL53L1X.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "SmoothDrive.h"
// X rollers. Forward/right/clockwise inputs -100..100. Max raw PWM 1..255.
inline bool mixMecanum(long forward,long right,long clockwise,long power,int next[4]) {
  if(forward< -100||forward>100||right< -100||right>100||
     clockwise< -100||clockwise>100||power<1||power>255)return false;
  // M1 front-right (-forward), M2 rear-left (-forward),
  // M3 rear-right (-forward), M4 front-left (+forward).
  long raw[4]={-(forward-right-clockwise),-(forward-right+clockwise),
               -(forward+right-clockwise),forward+right+clockwise};
  long scale=100;
  for(int i=0;i<4;i++){long magnitude=raw[i]<0?-raw[i]:raw[i];if(magnitude>scale)scale=magnitude;}
  for(int i=0;i<4;i++){
    long magnitude=raw[i]<0?-raw[i]:raw[i];
    int value=(int)((magnitude*power+scale/2)/scale);
    next[i]=raw[i]<0?-value:value;
  }
  return true;
}


Adafruit_PWMServoDriver pca(0x40);
Servo linkServo, gripServo, giftServo;
const byte GIFT_PIN=4; // Digital D4 on Mega, NOT PCA9685 channel P4.
const byte RP[4]={7,5,11,9}, LP[4]={8,6,12,10};
const byte CHANNELS[5]={1,10,11,9,4};
const byte INPUTS[]={15,16,18,19};
const byte INPUT_COUNT=sizeof(INPUTS)/sizeof(INPUTS[0]);
int pulses[8]={0,0,0,0,0,0,0,0}, motors[4]={0,0,0,0};
byte raw[INPUT_COUNT], stable[INPUT_COUNT];
unsigned long changed[INPUT_COUNT]={0}, lastContact=0, lastDrive=0;
unsigned long driveDuration=650;
bool pcaReady=false, stopped=true, smoothActive=false;
// V3.16.23: D19 does not block AUTO search/approach motors; Python arms it only at final grip.
// True on boot, STOP, RESUME, or connection watchdog (manual/default fail-safe).
bool irDriveGuard=true;
long smoothAxes[3]={0,0,0};
int smoothPower=0;
unsigned long smoothRamp=0,smoothHold=0,lastSmoothTick=0;
byte prescale=121;
char buffer[80];
byte used=0;
bool overflow=false;

VL53L1X tofSensor;
const uint32_t TOF_BUDGET_US=50000;
const uint16_t TOF_PERIOD_MS=60, TOF_MAX_AGE_MS=250;
const uint8_t TOF_ROI_SIZE=10, TOF_ROI_CENTER=199;
bool tofPresent=false,tofValid=false,tofGuard=false,tofIgnoreInvalid=false;
uint16_t tofMm=0,tofMinMm=200,tofMaxMm=250;
uint8_t tofStatus=255,tofWindowCount=0;
unsigned long tofSeq=0,tofAt=0,tofPollAt=0;

bool tofBlocksDrive(unsigned long now){
  if(tofGuard && tofIgnoreInvalid){
    return tofPresent && tofValid && tofSeq && tofMm>0 && tofMm<=tofMaxMm;
  }
  return tofGuard && (!tofPresent || !tofValid || !tofSeq ||
         now-tofAt>TOF_MAX_AGE_MS || tofMm<=tofMaxMm);
}
void updateTof(unsigned long now){
  if(!tofPresent || now-tofPollAt<5)return;
  tofPollAt=now;
  if(tofSeq && now-tofAt>TOF_MAX_AGE_MS)tofWindowCount=0;
  bool ready=tofSensor.dataReady();
  if(tofSensor.last_status){tofValid=false;tofWindowCount=0;return;}
  if(!ready)return;
  uint16_t mm=tofSensor.read(false);
  bool timedOut=tofSensor.timeoutOccurred();
  tofStatus=(uint8_t)tofSensor.ranging_data.range_status;
  tofValid=!timedOut && !tofSensor.last_status &&
           tofStatus==(uint8_t)VL53L1X::RangeValid && mm>0 && mm<=4000;
  tofMm=mm;tofAt=millis();if(++tofSeq==0)++tofSeq;
  if(tofGuard && tofValid && mm>=tofMinMm && mm<=tofMaxMm){
    if(tofWindowCount<255)tofWindowCount++;
  }else tofWindowCount=0;
}

void motorStop(){
  smoothActive=false;
  for(byte i=0;i<4;i++){
    analogWrite(RP[i],0); analogWrite(LP[i],0); motors[i]=0;
  }
}
void allStop(){
  motorStop(); stopped=true;irDriveGuard=true;tofGuard=false;tofIgnoreInvalid=false;tofWindowCount=0;
  linkServo.detach(); gripServo.detach(); giftServo.detach();
  pinMode(2,OUTPUT); pinMode(3,OUTPUT); pinMode(GIFT_PIN,OUTPUT);
  digitalWrite(2,LOW); digitalWrite(3,LOW); digitalWrite(GIFT_PIN,LOW);
  if(pcaReady) for(byte i=0;i<16;i++) pca.setPWM(i,0,4096);
  for(byte i=0;i<8;i++) pulses[i]=0;
}
void prefix(unsigned long id){Serial.print(id);Serial.print(' ');}
void ok(unsigned long id){prefix(id);Serial.println(F("OK"));}
void err(unsigned long id,const __FlashStringHelper *msg){
  prefix(id);Serial.print(F("ERR "));Serial.println(msg);
}
void status(unsigned long id){
  prefix(id);Serial.print(F("STATUS {\"pca\":"));Serial.print(pcaReady?F("true"):F("false"));
  Serial.print(F(",\"smooth_protocol\":1,\"gift_protocol\":3,\"drive_protocol\":4,\"ir_drive_guard\":"));Serial.print(irDriveGuard?F("true"):F("false"));Serial.print(F(",\"stopped\":"));Serial.print(stopped?F("true"):F("false"));
  Serial.print(F(",\"joints\":{"));
  for(byte i=0;i<8;i++){
    if(i)Serial.print(',');
    Serial.print('"');Serial.print(i<5?'p':'d');
    Serial.print(i<5?CHANNELS[i]:(i==7?GIFT_PIN:i-3));Serial.print(F("\":"));
    if(pulses[i])Serial.print(pulses[i]);else Serial.print(F("null"));
  }
  Serial.print(F("},\"motors\":["));
  for(byte i=0;i<4;i++){if(i)Serial.print(',');Serial.print(motors[i]);}
  Serial.print(F("],\"sensors\":{"));
  for(byte i=0;i<INPUT_COUNT;i++){
    if(i)Serial.print(',');
    Serial.print(F("\"D"));Serial.print(INPUTS[i]);
    Serial.print(F("\":"));Serial.print(stable[i]);
  }
  Serial.print(F("},\"tof_protocol\":1,\"tof_auto_continue\":true,\"tof\":{\"present\":"));Serial.print(tofPresent?F("true"):F("false"));
  Serial.print(F(",\"roi_size_configured\":"));Serial.print(TOF_ROI_SIZE);
  Serial.print(F(",\"valid\":"));Serial.print(tofValid?F("true"):F("false"));
  Serial.print(F(",\"mm\":"));if(tofSeq)Serial.print(tofMm);else Serial.print(F("null"));
  Serial.print(F(",\"status\":"));Serial.print(tofStatus);
  Serial.print(F(",\"seq\":"));Serial.print(tofSeq);
  Serial.print(F(",\"age_ms\":"));if(tofSeq)Serial.print(millis()-tofAt);else Serial.print(F("null"));
  Serial.print(F(",\"guard\":"));Serial.print(tofGuard?F("true"):F("false"));
  Serial.print(F(",\"invalid_policy\":"));Serial.print(tofIgnoreInvalid?F("\"continue\""):F("\"stop\""));
  Serial.print(F(",\"window_count\":"));Serial.print(tofWindowCount);
  Serial.print(F(",\"min_mm\":"));Serial.print(tofMinMm);
  Serial.print(F(",\"max_mm\":"));Serial.print(tofMaxMm);
  Serial.println(F("}}"));
}
bool setPulse(char type,long channel,long us){
  const long maxUs=(type=='P'&&channel==1)?3500:2500;
  if(us<500||us>maxUs)return false;
  if(type=='P'){
    if(!pcaReady)return false;
    int idx=-1;
    for(byte i=0;i<5;i++)if(channel==CHANNELS[i])idx=i;
    if(idx<0)return false;
    // 25MHz oscillator, divider read from hardware after setPWMFreq(50).
    uint16_t ticks=(uint16_t)((us*25UL+(prescale+1)/2)/(prescale+1));
    if(pca.setPWM((byte)channel,0,ticks)!=0)return false;
    pulses[idx]=(int)us;
    return true;
  }
  if(type=='S'&&(channel==2||channel==3||channel==GIFT_PIN)){
    Servo &s=channel==GIFT_PIN?giftServo:(channel==2?linkServo:gripServo);
    // Avoid transient default 1500us on first attach.
    noInterrupts();
    if(!s.attached())s.attach((byte)channel,500,2500);
    s.writeMicroseconds((int)us);
    interrupts();
    pulses[channel==GIFT_PIN?7:(channel==2?5:6)]=(int)us;
    return true;
  }
  return false;
}
bool opposite(int a,int b){return a&&b&&((a>0)!=(b>0));}
void writeMotors(const int next[4]){
  for(byte i=0;i<4;i++){
    analogWrite(RP[i],0);analogWrite(LP[i],0);
    if(next[i]>0)analogWrite(RP[i],next[i]);
    if(next[i]<0)analogWrite(LP[i],-next[i]);
    motors[i]=next[i];
  }
}
void drive(const int next[4]){
  smoothActive=false;writeMotors(next);lastDrive=millis();
}
void updateSmooth(unsigned long now){
  if(!smoothActive || stopped || now-lastSmoothTick<10)return;
  if(stable[2]==LOW || (irDriveGuard && stable[3]==LOW)){motorStop();return;}
  if(tofBlocksDrive(now)){motorStop();return;}
  lastSmoothTick=now;
  unsigned long elapsed=now-lastDrive;
  if(elapsed>=2*smoothRamp+smoothHold){motorStop();return;}
  int power=smoothPwm(smoothPower,elapsed,smoothRamp,smoothHold);
  int next[4]={0,0,0,0};
  if(power)mixMecanum(smoothAxes[0],smoothAxes[1],smoothAxes[2],power,next);
  writeMotors(next); // Never renew the lease while updating PWM.
}
void handle(char *line){
  // Require numeric id so Python can correlate every acknowledgement.
  if(*line<'0'||*line>'9'){Serial.println(F("0 ERR REQUEST_ID_REQUIRED"));return;}
  char *end;
  unsigned long id=strtoul(line,&end,10);
  if(*end!=' '){err(id,F("SYNTAX"));return;}
  char *cmd=end+1;
  lastContact=millis();
  if(!strcmp(cmd,"STOP")){allStop();ok(id);return;}
  if(!strcmp(cmd,"STATUS")){status(id);return;}
  if(!strcmp(cmd,"XM")){motorStop();ok(id);return;}
  if(!strcmp(cmd,"RESUME")){stopped=false;irDriveGuard=true;ok(id);return;}
  if(stopped){err(id,F("STOP_LATCHED"));return;}
  // IG 0 ignores D19 during search/wheel alignment; IG 1 arms D19 for final grasp.
  // Switching modes always brakes wheels; D18, VL53 and STOP remain active.
  if(!strcmp(cmd,"IG 0") || !strcmp(cmd,"IG 1")){
    motorStop();irDriveGuard=(cmd[3]=='1');ok(id);return;
  }
  if(!strcmp(cmd,"TF OFF")){motorStop();tofGuard=false;tofIgnoreInvalid=false;tofWindowCount=0;ok(id);return;}
  if(cmd[0]=='T' && cmd[1]=='F'){
    if(strncmp(cmd,"TF ",3)){err(id,F("TOF_RANGE"));return;}
    long minimum=0,maximum=0;char tail=0;
    bool autoContinue=!strncmp(cmd,"TF AUTO ",8);
    const char *range=cmd+(autoContinue?8:3);
    if(sscanf(range,"%ld %ld %c",&minimum,&maximum,&tail)!=2 ||
       minimum<100 || maximum>1000 || minimum>=maximum){err(id,F("TOF_RANGE"));return;}
    if(!autoContinue && !tofPresent){err(id,F("TOF_NOT_PRESENT"));return;}
    motorStop();tofMinMm=(uint16_t)minimum;tofMaxMm=(uint16_t)maximum;
    tofGuard=true;tofIgnoreInvalid=autoContinue;tofWindowCount=0;ok(id);return;
  }
  char op=0,extra=0;long a=0,b=0,c=0,d=0,duration=0;
  if(cmd[0]=='G'){
    long ramp=0,hold=0;
    if(sscanf(cmd,"%c %ld %ld %ld %ld %ld %ld %c",&op,&a,&b,&c,&d,&ramp,&hold,&extra)!=7 ||
       ramp<0 || hold<0 || !validSmooth((unsigned long)ramp,(unsigned long)hold)){
      err(id,F("SMOOTH_RANGE"));return;
    }
    int next[4];
    if(!mixMecanum(a,b,c,d,next)){err(id,F("MECANUM_RANGE"));return;}
    if(stable[2]==LOW || (irDriveGuard && stable[3]==LOW)){err(id,F("IR_BLOCKS_AUTO_DRIVE"));return;}
    if(tofBlocksDrive(millis())){err(id,F("TOF_BLOCKS_AUTO_DRIVE"));return;}
    bool moving=smoothActive;
    for(byte i=0;i<4;i++)if(motors[i])moving=true;
    if(moving){err(id,F("STOP_BEFORE_SMOOTH"));return;}
    motorStop();smoothAxes[0]=a;smoothAxes[1]=b;smoothAxes[2]=c;
    smoothPower=(int)d;smoothRamp=(unsigned long)ramp;smoothHold=(unsigned long)hold;
    driveDuration=2*smoothRamp+smoothHold;lastDrive=lastSmoothTick=millis();
    smoothActive=true;ok(id);return;
  }
  if(cmd[0]=='C'||cmd[0]=='D'){
    if(smoothActive){err(id,F("SMOOTH_BUSY"));return;}
    if(sscanf(cmd,"%c %ld %ld %ld %ld %ld %c",&op,&a,&b,&c,&d,&duration,&extra)!=6||
       duration<50||duration>500){
      err(id,F("DRIVE_PULSE_RANGE"));return;
    }
    int next[4];
    if(tofBlocksDrive(millis())){err(id,F("TOF_BLOCKS_AUTO_DRIVE"));return;}
    if(op=='C'){
      if(!mixMecanum(a,b,c,d,next)){err(id,F("MECANUM_RANGE"));return;}
    }else{
      if(a< -255||a>255||b< -255||b>255||c< -255||c>255||d< -255||d>255){
        err(id,F("MOTOR_RANGE"));return;
      }
      next[0]=(int)a;next[1]=(int)b;next[2]=(int)c;next[3]=(int)d;
    }
    for(byte i=0;i<4;i++)if(opposite(motors[i],next[i])){err(id,F("STOP_BEFORE_REVERSE"));return;}
    driveDuration=(unsigned long)duration;drive(next);ok(id);return;
  }
  if(cmd[0]=='V'||cmd[0]=='W'){err(id,F("USE_MECANUM_C_OR_D"));return;}
  if(sscanf(cmd,"%c %ld %ld %c",&op,&a,&b,&extra)!=3){err(id,F("SYNTAX"));return;}
  if(op=='P'||op=='S'){
    if(setPulse(op,a,b))ok(id);else err(id,F("PULSE_RANGE_CHANNEL_OR_PCA"));
  }else err(id,F("UNKNOWN_COMMAND"));
}
void setup(){
  Serial.begin(115200);
  for(byte i=0;i<4;i++){pinMode(RP[i],OUTPUT);pinMode(LP[i],OUTPUT);}
  // Preserve previous relay D30=HIGH. Confirm polarity in actual wiring.
  digitalWrite(30,HIGH);pinMode(30,OUTPUT);
  for(byte i=0;i<INPUT_COUNT;i++){
    pinMode(INPUTS[i],INPUT_PULLUP);raw[i]=stable[i]=digitalRead(INPUTS[i]);changed[i]=millis();
  }
  Wire.begin();
  Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000,true);
#endif
  pcaReady=pca.begin();
  if(pcaReady){pca.setPWMFreq(50);delay(10);prescale=pca.readPrescale();}
  allStop();
  tofSensor.setTimeout(200);
  tofPresent=tofSensor.init();
  if(tofPresent){
    tofPresent=tofSensor.setDistanceMode(VL53L1X::Short) &&
               tofSensor.setMeasurementTimingBudget(TOF_BUDGET_US);
    if(tofPresent){
      tofSensor.setROISize(TOF_ROI_SIZE,TOF_ROI_SIZE);
      tofSensor.setROICenter(TOF_ROI_CENTER);
      tofSensor.startContinuous(TOF_PERIOD_MS);
      if(tofSensor.last_status)tofPresent=false;
    }
  }
  lastContact=millis();
  Serial.println(F("EVENT ROBOT_V3_MECANUM_READY_STOPPED"));
}
void loop(){
  // Bounded reads prevent serial flooding from starving watchdog/sensors.
  byte budget=80;
  while(Serial.available()&&budget--){
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r'){
      if(overflow)Serial.println(F("0 ERR LINE_TOO_LONG"));
      else if(used){buffer[used]=0;handle(buffer);}
      used=0;overflow=false;
    }else if(!overflow){
      if(used<sizeof(buffer)-1)buffer[used++]=c;else overflow=true;
    }
  }
  unsigned long now=millis();
  for(byte i=0;i<INPUT_COUNT;i++){
    byte v=digitalRead(INPUTS[i]);
    if(v!=raw[i]){raw[i]=v;changed[i]=now;}
    if(now-changed[i]>=120)stable[i]=raw[i];
  }
  updateTof(now);
  now=millis();
  if(tofBlocksDrive(now))motorStop();
  updateSmooth(now);
  bool moving=false;
  for(byte i=0;i<4;i++)if(motors[i])moving=true;
  if(moving&&now-lastDrive>=driveDuration){motorStop();Serial.println(F("EVENT MOTOR_WATCHDOG"));}
  if(!stopped&&now-lastContact>=2000){allStop();Serial.println(F("EVENT LINK_WATCHDOG_STOP"));}
}
