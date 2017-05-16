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

#define MASTERARM             198.f
#define SLAVEARM              354.f
//#define MASTERARM             150.f
//#define SLAVEARM              220.f
#define R                     182.f
#define r                     45.f
#define SQRT3                 1.73205f
#define BASESTARTPOS          -500.1f		//未确定
#define STARTANGLE            70.f
#define RECORDPOINT           2000
#define POOLSIZE              2000
/*
三个轮子；编码器和姿态的对应是固定的
109.41°时是-7，内正外负(旋转正方向)
*/
#define ANGLE(code)  ((double)(102.64f-abs(0+code)*0.208333f))
#define CODE(angle)  ((int)(-0.f-fabs(102.64f-angle)*4.8f))

//控制最小误差
#define ERROR(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))<c)
//教学最小步调
#define TOUCH(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))>c)

#define ZPLAN        -300
/*******typedef**********/
typedef  struct {
	double x;
	double y;
	double z;
}coordinate_t;

typedef  struct {
	double No1;
	double No2;
	double No3;
}angle_t;

typedef unsigned char     uint8_t;
typedef coordinate_t* pcoordinate_t;

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/




