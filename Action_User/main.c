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
	  
	TIM_Init(TIM2,999,839,0,0);					//主周期定时10ms	
	
	UART5_Init(115200);		//调试用蓝牙
	
	USART3_Init(115200);
	KeyInit();//按住为1
	Flash_Init();
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8, GPIO_Pin_9);
	#ifndef DEGUG
	#ifndef DATAPOOL
	elmo_Init();
	Pos_cfg(1,5000,5000,100);
	Pos_cfg(2,5000,5000,100);
	Pos_cfg(3,5000,5000,100);
	PosCrl(1,0,-100);
	PosCrl(2,0,-100);
	PosCrl(3,0,-100);
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




































