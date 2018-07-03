/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "can.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/************************************************************/
/****************驱动器CAN1接口模块****start******************/

static int32_t ElmoVel[3]={0};
int* const pElmoVel=ElmoVel;
static int elmoPos[3]={0};
int* const pElmoPos=elmoPos;
void CAN1_RX0_IRQHandler(void)
{
	static uint8_t buffer[8];
	static uint32_t StdId=0;
union MSG
{
	uint8_t data8[8];
	int data32[2];
	float dataf[2];
}msg;
	CAN_RxMsg(CAN1, &StdId,buffer,8);
	for(uint8_t i = 0; i < 8; i++)
		msg.data8[i] = buffer[i];
 if(msg.data32[0] == 0x00005850)
		{
			if(StdId==0x281)
			{
				elmoPos[0]=msg.data32[1];
			}
			if(StdId==0x282)
			{
				elmoPos[1]=msg.data32[1];
			}
			if(StdId==0x283)
			{
				elmoPos[2]=msg.data32[1];
			}
		}
	if(msg.data32[0] == 0x00005856)
		{
			if(StdId==0x281)
			{
				ElmoVel[0]=msg.data32[1];
			}
			if(StdId==0x282)
			{
				ElmoVel[1]=msg.data32[1];
			}
			if(StdId==0x283)
			{
				ElmoVel[2]=msg.data32[1];
			}
		}
	CAN_ClearFlag(CAN1,CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1,CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1,CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1,CAN_FLAG_LEC);
	
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1,CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1,CAN_FLAG_FOV1);
} 
int* getChainCode(void)
{
return pElmoPos;
}
int* getChainVel(void)
{
return pElmoVel;
}
/****************驱动器CAN1接口模块****end******************/
/************************************************************/

/*************定时器2******start************/
//每1ms调用一次  用于读取编码器的值和计算坐标

//int posx,posy;
static uint16_t timeCount=0;
static uint8_t timeFlag=0;
static uint16_t cpuUsage;
static uint16_t usageCount=0;
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_Update)==SET)
  {	
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		timeCount++;
		usageCount++;
		
		if(timeCount>=1)
		{
			timeCount=0;
			timeFlag=1;
		}
  }	 
}

uint8_t getTimeFlag(void)
{
	static uint8_t lastFlag=0;
	
	uint8_t nowFlag;
	nowFlag=timeFlag;
	
	if(lastFlag)
	{
		cpuUsage=usageCount;
		usageCount=0;
	}
	lastFlag=nowFlag;
	
	if(nowFlag)
	{
		usageCount=0;
		timeFlag=0;
		return 1;
	}
	return 0;
}
//定时器1  左编码器中断
void TIM1_UP_TIM10_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}

//定时器8  右编码器中断
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8, TIM_IT_Update)==SET)    
	{                                                
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
}

/********************************************************/
/*****************普通定时TIM5*****Start*****************/
void TIM5_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM5, TIM_IT_Update)==SET)    
	{              
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3, TIM_IT_Update)==SET)    
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	
	}
}



//定时器4  
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
	{                                  
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}
extern Robot_t gRobot;
#define NUM_NUM	32
void UART5_IRQHandler(void)
{
	static uint8_t ch;
	static union
	{
		uint8_t data[NUM_NUM];
		float ActVal[8];
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;

	if(USART_GetITStatus(UART5, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(UART5,USART_IT_RXNE);
		ch=USART_ReceiveData(UART5);		
		 switch(count)
		 {
			 case 0:
				 if(ch==0x0d||ch=='O')
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else if(ch=='K')
				 {
						count=0;
				 }
				 else
					 count=0;
				 break;
				 
			 case 2:
				posture.data[i]=ch;
			   i++;
			   if(i>=NUM_NUM)
				 {
					 i=0;
					 count++;
				 }
				 break;
				 
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 4:
				 if(ch==0x0d)
				 {
					 gRobot.isUsePPS=1;
					 gRobot.pps.angleX=posture.ActVal[0];
					 gRobot.pps.angleY=posture.ActVal[1];
					 gRobot.pps.angleZ=posture.ActVal[2];
					 gRobot.pps.x=posture.ActVal[3];
					 gRobot.pps.y=posture.ActVal[4];
						break;
				 }
			 default:
				 count=0;
			   break;		 
		 
	 }
 }
	else
	{
		USART_ReceiveData(UART5);
  }
}


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
 
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

