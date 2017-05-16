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

/*����ģʽ*/
#define TRACK

/*debugģʽ�ǿ�delta���ֲ���֮��Ĺ�ϵ*/
//#define DEGUG

/*�����ߵ�����һ����*/
//#define TRYPOINT

/*·����Ʋ�����*/
//#define ROUTEPLAN

#define MASTERARM             198.f
#define SLAVEARM              354.f
//#define MASTERARM             150.f
//#define SLAVEARM              220.f
#define R                     182.f
#define r                     45.f
#define SQRT3                 1.73205f
#define BASESTARTPOS          -500.1f		//δȷ��
#define STARTANGLE            70.f
#define RECORDPOINT           2000
#define POOLSIZE              2000
/*
�������ӣ�����������̬�Ķ�Ӧ�ǹ̶���
109.41��ʱ��-7�������⸺(��ת������)
*/
#define ANGLE(code)  ((double)(102.64f-abs(0+code)*0.208333f))
#define CODE(angle)  ((int)(-0.f-fabs(102.64f-angle)*4.8f))

//������С���
#define ERROR(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))<c)
//��ѧ��С����
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




