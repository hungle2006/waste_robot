// Standalone bench test ONLY. Reflash robot_control_v3 before AUTO.
#include <Wire.h>
#include <VL53L1X.h>
VL53L1X sensor;
unsigned long sequence=0;
void setup(){
  Serial.begin(115200);Wire.begin();Wire.setClock(100000);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(25000,true);
#endif
  sensor.setTimeout(200);
  if(!sensor.init() || !sensor.setDistanceMode(VL53L1X::Short) ||
     !sensor.setMeasurementTimingBudget(50000)){
    Serial.println(F("VL53 INIT ERROR"));while(1){}
  }
  sensor.setROISize(10,10);sensor.setROICenter(199);sensor.startContinuous(60);
  Serial.println(F("VL53L1X SHORT ROI10x10 CENTER199 BUDGET50ms PERIOD60ms"));
}
void loop(){
  if(!sensor.dataReady())return;
  uint16_t mm=sensor.read(false);
  bool valid=!sensor.timeoutOccurred() && !sensor.last_status &&
             sensor.ranging_data.range_status==VL53L1X::RangeValid && mm>0;
  Serial.print(++sequence);Serial.print(F(" | status="));
  Serial.print((int)sensor.ranging_data.range_status);
  if(!valid){Serial.println(F(" | INVALID"));return;}
  Serial.print(F(" | "));Serial.print(mm);Serial.print(F(" mm | "));
  Serial.print(mm/10.0,1);Serial.print(F(" cm | "));
  if(mm>=250 && mm<=300)Serial.println(F("IN 25..30cm WINDOW"));
  else if(mm<250)Serial.println(F("BELOW 25cm - STOP"));
  else Serial.println(F("FARTHER THAN 30cm"));
}
