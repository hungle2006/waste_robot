#ifndef WASTE_ROBOT_SMOOTH_DRIVE_H
#define WASTE_ROBOT_SMOOTH_DRIVE_H
#include <stdint.h>
// Fixed-point smoothstep: zero slope at both ends; no floating point on Mega.
inline bool validSmooth(unsigned long ramp,unsigned long hold){
  return ramp>=40 && ramp<=400 && hold<=400 && 2*ramp+hold<=1000;
}
inline int smoothPwm(int peak,unsigned long elapsed,unsigned long ramp,unsigned long hold){
  if(peak<1 || peak>255 || !validSmooth(ramp,hold) || elapsed>=2*ramp+hold)return 0;
  if(elapsed>=ramp && elapsed<ramp+hold)return peak;
  unsigned long t=elapsed<ramp?elapsed:2*ramp+hold-elapsed;
  uint32_t q=(uint32_t)t*1000UL/ramp;
  uint32_t eased=q*q*(3000UL-2*q)/1000000UL;
  return (int)(((uint32_t)peak*eased+500UL)/1000UL);
}
#endif
