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
//#define TRACK

/*debugģʽ�ǿ�delta���ֲ���֮��Ĺ�ϵ*/
//#define DEGUG

/*�����ߵ�����һ����*/
//#define TRYPOINT

/*·����Ʋ�����*/
//#define ROUTEPLAN

#define VELOCITY

#define MASTERARM             198.f
#define SLAVEARM              354.f
//#define MASTERARM             150.f
//#define SLAVEARM              220.f
#define R                     182.f
#define r                     45.f
#define SQRT3                 1.73205f
#define BASESTARTPOS          -200.1f		//δȷ��
#define STARTANGLE            70.f
#define RECORDPOINT           2000
#define POOLSIZE              1
/*
�������ӣ�����������̬�Ķ�Ӧ�ǹ̶���
109.41��ʱ��-7�������⸺(��ת������)
*/
#define ANGLE(code)  ((double)(104.04f-abs(0+code)*0.208333f))
#define CODE(angle)  ((int)(-0.f-fabs(104.04f-angle)*4.8f+0.5f))
#define CODEVEL(angle)  ((int)(angle*1728.f/360.f+0.5f))

//������С���
#define ERROR(a,b,c)   ((__sqrtf(pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2)))<c)
//��ѧ��С����
#define TOUCH(a,b,c)   ((__sqrtf(pow((a[0]-b[0]),2)+pow((a[1]-b[1]),2)+pow((a[2]-b[2]),2)))>c)

/*�������Ķ�������ģ��*/
#define Norm(vector) __sqrtf(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])

#define AIM(Aim1,Aim2,Aim3)  for(uint8_t i=0;i<3;i++){BaseCenterAim[0]=Aim1;BaseCenterAim[1]=Aim2;BaseCenterAim[2]=Aim3;}

#define AIMINPUT(Aim1,Aim2,Aim3)  for(uint8_t i=0;i<3;i++){gBaseCenterAim[aimOrder][0]=Aim1;gBaseCenterAim[aimOrder][1]=Aim2;gBaseCenterAim[aimOrder][2]=Aim3;}

#define ZPLAN        -300
/*******typedef**********/

typedef unsigned char     uint8_t;

#endif

/******************* (C) COPYRIGHT 2015 ACTION *****END OF FILE****/




