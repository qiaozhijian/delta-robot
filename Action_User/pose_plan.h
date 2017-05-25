#ifndef __INPUT_H
#define __INPUT_H

#include "stdint.h"
#include "stm32f4xx_gpio.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "route_ctr.h"



void BaseCenterUpdate(const float ChainAngle[3],float BaseCenter_t[3]);
void ChainUpdateByBase(const float BaseCenter_t[3], float ChainAngle[3]);
void VelocityUpdate(const float BaseCenter_t[3],const int AngularVel[3],float Velocity[3]);
void AngularVelUpdate(const float BaseCenter_t[3],const double Velocity[3],int AngularVelCode[3]);
float getK(float startPoint,float endPoint);
void GetPartVel(const float startToAim[3],const float startToNow[3],const float K,float partVel[3]);
#endif




