#pragma once
#include <stdint.h>
class VL53L1X {
public:
  enum DistanceMode {Short};
  enum RangeStatus {RangeValid=0,Invalid=255};
  struct RangingData {RangeStatus range_status=RangeValid;} ranging_data;
  uint8_t last_status=0;
  uint8_t roiWidth=0,roiHeight=0,roiCenter=0;
  bool present=true,ready=false;
  uint16_t mm=900;
  void setTimeout(uint16_t){}
  bool init(){return present;}
  bool setDistanceMode(DistanceMode){return true;}
  bool setMeasurementTimingBudget(uint32_t){return true;}
  void setROISize(uint8_t w,uint8_t h){roiWidth=w;roiHeight=h;}
  void setROICenter(uint8_t c){roiCenter=c;}
  void startContinuous(uint32_t){}
  bool dataReady(){return ready;}
  uint16_t read(bool){ready=false;return mm;}
  bool timeoutOccurred(){return false;}
};
