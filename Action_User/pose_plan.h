#ifndef __INPUT_H
#define __INPUT_H

#include "stdint.h"
#include "stm32f4xx_gpio.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "route_ctr.h"



void BaseCenterUpdate(const float ChainAngle[3],pcoordinate_t BaseCenter_t);
void ChainUpdateByBase(const coordinate_t BaseCenter_t, float ChainAngle[3]);
void AngularVelUpdate(const coordinate_t BaseCenter_t,const float ChainAngle[3],const float Velocity[3],float AngularVel[3]);
void VelocityUpdate(const coordinate_t BaseCenter_t,const float ChainAngle[3],const float AngularVel[3],float Velocity[3]);
float getK(coordinate_t startPoint,coordinate_t endPoint);
#endif




