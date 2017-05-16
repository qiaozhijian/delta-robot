#ifndef __INPUT_H
#define __INPUT_H

#include "stdint.h"
#include "stm32f4xx_gpio.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "route_ctr.h"



void BaseCenterUpdate(const double ChainAngle[3],pcoordinate_t BaseCenter_t);
void ChainUpdateByBase(const coordinate_t BaseCenter_t, double ChainAngle[3]);
void AngularVelUpdate(const coordinate_t BaseCenter_t,const double ChainAngle[3],const double Velocity[3],double AngularVel[3]);
void VelocityUpdate(const coordinate_t BaseCenter_t,const double ChainAngle[3],const double AngularVel[3],double Velocity[3]);
double getK(coordinate_t startPoint,coordinate_t endPoint);
#endif




