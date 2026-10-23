#ifndef WASTE_ROBOT_MECANUM_H
#define WASTE_ROBOT_MECANUM_H
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
#endif
