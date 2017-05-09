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
/*����ģʽ*/
//#define TRACK
/*debugģʽ�ǿ�delta���ֲ���֮��Ĺ�ϵ*/
#define DEGUG
/*�����ߵ�����һ����*/
//#define TRYPOINT
/*����֮����·����Ʋ�����*/

#define MASTERARM             120.f
#define SLAVEARM              220.f
#define R                     182.f
#define r                     45.f
#define SQRT3                 1.73205f
#define BASESTARTPOS          -141.f		//δȷ��
#define STARTANGLE            70.f
#define RECORDPOINT           500
/*
�������ӣ�����������̬�Ķ�Ӧ�ǹ̶���
109.41��ʱ��-7�������⸺(��ת������)
*/
#define ANGLE(code)  ((float)(109.41f-abs(6+code)/4096.f*360.f))
#define CODE(angle)  ((int)(-7.f-fabs(109.41f-angle)/360.f*4096.f))

//������С���
#define ERROR(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))<c)
//��ѧ��С����
#define TOUCH(a,b,c)   ((__sqrtf(pow((a.x-b.x),2)+pow((a.y-b.y),2)+pow((a.z-b.z),2)))>c)

#define ZPLAN        -250
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




