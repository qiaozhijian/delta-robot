#include "stm32f4xx.h"
#include "usart.h"
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "stm32f4xx_it.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "math.h"
#include "pose_plan.h"
#include "custom.h"
#include "route_ctr.h"
#include "stm32f4xx_usart.h"
#include "stdio.h"
#include "arm_math.h"
#include "flash.h"

void init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	  
	TIM_Init(TIM2,999,839,0,0);					//�����ڶ�ʱ10ms	
	
	UART5_Init(115200);		//����������
	
	USART3_Init(115200);
	KeyInit();//��סΪ1
	Flash_Init();
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	#ifndef DEGUG
	#ifndef DATAPOOL
	#define START -200
	elmo_Init();
	Vel_cfg(1,100000,100000);
	Vel_cfg(2,100000,100000);
	Vel_cfg(3,100000,100000);
	//elmo_Disable(0);
	#endif
	#endif
}
int main(void)
{
	init();
	while(1)
	{
	while(getTimeFlag())
	{
	//	USART_OUT(USART3,"DDD");
		RouteControl();
//		RouteOutput();
		
	}
	}
}






























//		static int aaa,bbb,ccc,order=0;
//		if(KeyCatch())
//		{
//		PosCrl(1,0,aaa);
//		PosCrl(2,0,bbb);
//		PosCrl(3,0,ccc);
//		switch(order)
//		{
//			case 0:
//				aaa-=250;
//			order++;
//			break;
//			case 1:
//				aaa+=250;
//			order++;
//			break;
//			case 2:
//				bbb-=250;
//			order++;
//			break;
//			case 3:
//				bbb+=250;
//			order++;
//			break;
//			case 4:
//				ccc-=250;
//			order++;
//			break;
//			case 5:
//				ccc+=250;
//			order=0;
//			break;
//			
//		}
//}



































