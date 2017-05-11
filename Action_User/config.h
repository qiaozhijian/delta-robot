/**
  ******************************************************************************
  * @file    
  * @author  Summer Action
  * @version 
  * @date   
  * @brief   This file contains the headers of 
  ******************************************************************************
  * @attention
  *
  *
  * 
  * 
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_H
#define __CONFIG_H


/********define**********/
//#define DATAPOOL

/*跟随模式*/
#define TRACK

/*debug模式是看delta各种参数之间的关系*/
//#define DEGUG

/*这是走到任意一个点*/
//#define TRYPOINT

/*路径设计并跟随*/
//#define ROUTEPLAN

#define MASTERARM             120.f
#define SLAVEARM              220.f
#define R                     182.f
#define r                     45.f
#define SQRT3                 1.73205f
#define BASESTARTPOS          -141.f		//未确定
#define STARTANGLE            70.f
#define RECORDPOINT           2000
#define POOLSIZE              2000
/*
三个轮子；编码器和姿态的对应是固定的
109.41°时是-7，内正外负(旋转正方向)
*/
#define ANGLE(code)  ((float)(109.41f-abs(6+code)/4096.f*360.f))
#define CODE(angle)  ((int)(-7.f-fabs(109.41f-angle)/360.f*4096.f))

//控制最小误差
#define ERROR(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))<c)
//教学最小步调
#define TOUCH(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))>c)

#define ZPLAN        -300
/*******typedef**********/
typedef  struct {
	float x;
	float y;
	float z;
}coordinate_t;

typedef  struct {
	float No1;
	float No2;
	float No3;
}angle_t;

typedef unsigned char     uint8_t;
typedef coordinate_t* pcoordinate_t;

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/




