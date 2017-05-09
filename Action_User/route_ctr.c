#include "route_ctr.h"
#include "stdio.h"

/************variable  define**********/
#ifdef  TRACK
static  int gChainCodeAim[RECORDPOINT][3]={0};
static uint16_t trackOrder=0;
#else
static  int gChainCodeAim[3]={0};
#endif
static  int gChainCode[3] = { 0 ,0 ,0 };

static  float gChainAngle[3] = { STARTANGLE ,STARTANGLE ,STARTANGLE };
static float gChainAngleAim[3]={0};

static coordinate_t gBaseCenter_t = { 0,0,BASESTARTPOS };
//ZPLAN是工作平台，一般不变
static coordinate_t gBaseCenterAim_t = { 0,0,ZPLAN };

static  int gChainVel[3] = { 0, 0 ,0 };
static uint8_t gTeachFinish=0;
//速度单位mm/s  角速度为弧度制
static float gVelocity[3]={ 40,35,14 };
static float gAngularVel[3]={ 0,0,0 };
/************funtction  claim**********/
#ifdef  TRACK
uint8_t	ReachToPoint(uint8_t pointToReach);	
#else
uint8_t ReachToPoint(uint8_t error);
#endif
void ChainCodeAimUpdate(void);
void ParaUpdate(void);
uint8_t TrackByLine(coordinate_t startPoint,coordinate_t endPoint,uint8_t steps,uint8_t error);
void RouteControl(void)
{
	
	#ifdef TRACK
	static uint16_t pointToReach=0;
	/*读取需要的编码器值和速度值*/
	ParaUpdate();	
	if(!gTeachFinish)
	ChainCodeAimUpdate();
	else
	{
		if(pointToReach<trackOrder)
		{
		if(ReachToPoint(pointToReach))
			pointToReach++;
		}
		else
		{
			gTeachFinish=0;
			pointToReach=0;
		}
	}
	#else
	#ifdef DEGUG
	//BaseCenterUpdate(gChainAngle,&gBaseCenter_t);
	ChainUpdateByBase(gBaseCenter_t, gChainAngle);
	AngularVelUpdate(gBaseCenter_t,gChainAngle,gVelocity,gAngularVel);
	VelocityUpdate(gBaseCenter_t,gChainAngle,gAngularVel,gVelocity);
	#else
	#ifdef TRYPOINT
	char* const pInstruct=getPInstruct();
	uint8_t intrctOrder=getIntrctOrder();
	uint16_t instruct[9]={
	*(pInstruct+intrctOrder*15+3),(*(pInstruct+intrctOrder*15+4)-48)*10,*(pInstruct+intrctOrder*15+5)-48,
	*(pInstruct+intrctOrder*15+7),(*(pInstruct+intrctOrder*15+8)-48)*10,*(pInstruct+intrctOrder*15+9)-48,
	(*(pInstruct+intrctOrder*15+11)-48)*100,(*(pInstruct+intrctOrder*15+12)-48)*10,*(pInstruct+intrctOrder*15+13)-48
	};
	ParaUpdate();
	
	if(instruct[0]=='+')
		gBaseCenterAim_t.x=instruct[1]+instruct[2];
	else
		gBaseCenterAim_t.x=-(instruct[1]+instruct[2]);
	
	if(instruct[3]=='+')
		gBaseCenterAim_t.y=instruct[4]+instruct[5];
	else
		gBaseCenterAim_t.y=-(instruct[4]+instruct[5]);
	
		gBaseCenterAim_t.z=-(instruct[6]+instruct[7]+instruct[8]);
	if(*(pInstruct+intrctOrder*15+0)!='G')
	{
		gBaseCenterAim_t.x=0;
		gBaseCenterAim_t.y=0;
		gBaseCenterAim_t.z=-250;
	}
	
	ChainUpdateByBase(gBaseCenterAim_t, gChainAngleAim);
	for(uint8_t i=0;i<3;i++)
	gChainCodeAim[i]=CODE(gChainAngleAim[i]);
	ReachToPoint();
	BaseCenterUpdate(gChainAngle,&gBaseCenter_t);
	#else
	/*读取需要的编码器值和速度值*/
	ParaUpdate();
	static uint8_t mode=0;
	static coordinate_t startPoint={0.f,0.f,-270.f};
	static coordinate_t endPoint={20.f,0.f,-270.f};
	TrackByLine(startPoint,endPoint,6,5);
	
	#endif
	#endif
	#endif

}
void Sprintf(float a,float b,float c,float d,float e,float f)
{
	unsigned char buff[10];
	sprintf((char*)buff,"%f",a);
	USART_OUT(USART3,buff);
	USART_OUT(USART3,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",b);
	USART_OUT(USART3,buff);
	USART_OUT(USART3,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",c);
	USART_OUT(USART3,buff);
	USART_OUT(USART3,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",d);
	USART_OUT(USART3,buff);
	USART_OUT(USART3,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",e);
	USART_OUT(USART3,buff);
	USART_OUT(USART3,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",f);
	USART_OUT(USART3,buff);
	USART_OUT(USART3,(uint8_t *)"\r\n");
}
/*TRACK表示跟随模式 非TRACK为路径规划跟随模式*/
#ifdef TRACK
void ChainCodeAimUpdate(void)
{
	static uint8_t key=0;
	static coordinate_t baseCenter_Old_t={0,0,0};
	static uint8_t keyCatch;
	static float debug=0;
	keyCatch=KeyCatch();
	if(keyCatch&&key==1)
	{
		key=0;
		gTeachFinish=1;
		
	}
	if(keyCatch&&key==0)
	{
		key=1;
		baseCenter_Old_t.x=gBaseCenter_t.x;
		baseCenter_Old_t.y=gBaseCenter_t.y;
		baseCenter_Old_t.z=gBaseCenter_t.z;
	}
	debug=(__sqrtf(pow((baseCenter_Old_t.x-gBaseCenter_t.x),2)+pow((baseCenter_Old_t.y-gBaseCenter_t.y),2)+pow((baseCenter_Old_t.z-gBaseCenter_t.z),2)));
	if(TOUCH(baseCenter_Old_t,gBaseCenter_t,7)&&key==1)
	{
	//Sprintf(baseCenter_Old_t.x,baseCenter_Old_t.y,baseCenter_Old_t.z,gBaseCenter_t.x,gBaseCenter_t.y,gBaseCenter_t.z);
		for(uint8_t i=0;i<3;i++)
		{
		gChainCodeAim[trackOrder][i]=gChainCode[i];
		}
		trackOrder++;
		//进来一次发一次发不完，改成最后一起发
		baseCenter_Old_t.x=gBaseCenter_t.x;
		baseCenter_Old_t.y=gBaseCenter_t.y;
		baseCenter_Old_t.z=gBaseCenter_t.z;
	}
}

uint8_t	ReachToPoint(uint8_t pointToReach)
{
	static int chainAim[3]={0};
	static float error=0.f;
//	static float aaa=1;
//	if(aaa==1)
//	{
//		aaa=0;
//		for(int j=0;j<trackOrder;j++)
//		{
//		for(uint8_t i=0;i<3;i++)
//		{
//		chainAim[i]=gChainCodeAim[j][i];
//		gChainAngleAim[i]=ANGLE(chainAim[i]);
//		}
//		BaseCenterUpdate(gChainAngleAim,&gBaseCenterAim_t);
//	//	USART_OUT(USART3,(uint8_t*)"%d\t%d\r\n",(int)gBaseCenterAim_t.x,(int)gBaseCenterAim_t.y);
//		}
//	}
	for(uint8_t i=0;i<3;i++)
	{
	chainAim[i]=gChainCodeAim[pointToReach][i];
	gChainAngleAim[i]=ANGLE(chainAim[i]);
	}
	PosCrl(1,0,chainAim[0]);
	PosCrl(2,0,chainAim[1]);
	PosCrl(3,0,chainAim[2]);
	BaseCenterUpdate(gChainAngleAim,&gBaseCenterAim_t);
	//error=(__sqrtf(pow((gBaseCenterAim_t.x-gBaseCenter_t.x),2)+pow((gBaseCenterAim_t.y-gBaseCenter_t.y),2)+pow((gBaseCenterAim_t.z-gBaseCenter_t.z),2)));
	//USART_OUT(USART3,(uint8_t*)"%d,%d,%d\tact:%d,%d,%d\r\n",(int)gBaseCenterAim_t.x,(int)gBaseCenterAim_t.y,(int)gBaseCenterAim_t.z,(int)gBaseCenter_t.x,(int)gBaseCenter_t.y,(int)gBaseCenter_t.z);
	//USART_OUT(USART3,(uint8_t*)"%d,%d,%d,%d,%d,%d\r\n",chainAim[0],chainAim[0],chainAim[1],gChainCode[2],gChainCode[1],gChainCode[2]);
	//if((abs(chainAim[0]-gChainCode[0])<10)&&(abs(chainAim[1]-gChainCode[1])<10)&&(abs(chainAim[2]-gChainCode[2])<10))
	if((fabs(gBaseCenterAim_t.x-gBaseCenter_t.x)<5)&&(fabs(gBaseCenterAim_t.y-gBaseCenter_t.y)<5)&&(fabs(gBaseCenterAim_t.z-gBaseCenter_t.z)<5))
	{
		
	//USART_OUT(USART3,(uint8_t*)"%d\t%d\t%d\t%d\r\n",(int)gBaseCenterAim_t.x,(int)gBaseCenterAim_t.y,(int)gBaseCenter_t.x,(int)gBaseCenter_t.y);
		return 1;
	}
	else
		return 0;
}
#else
void ChainCodeAimUpdate(void)
{
	ChainUpdateByBase(gBaseCenterAim_t, gChainAngleAim);

	for(uint8_t i=0;i<3;i++)
	gChainCodeAim[i]=CODE(gChainAngleAim[i]);
}
uint8_t ReachToPoint(uint8_t error)
{
	for(uint8_t i=0;i<3;i++)
	PosCrl(i+1,0,gChainCodeAim[i]);
	/*获得当时坐标编码*/
	BaseCenterUpdate(gChainAngle,&gBaseCenter_t);
	if(ERROR(gBaseCenter_t,gBaseCenterAim_t,error))
	return 1;
	else
	return 0;
}
#endif
uint8_t TrackByLine(coordinate_t startPoint,coordinate_t endPoint,uint8_t steps,uint8_t error)
{
	
	static float paraT=0.f;
	gBaseCenterAim_t.x=paraT*(endPoint.x-startPoint.x)+startPoint.x;
	gBaseCenterAim_t.y=paraT*(endPoint.y-startPoint.y)+startPoint.y;
	gBaseCenterAim_t.z=paraT*(endPoint.z-startPoint.z)+startPoint.z;
	ChainCodeAimUpdate();
	if(ReachToPoint(error))
	{
		paraT+=1.f/steps;
	}
	if(paraT>1)
	{
		paraT=1;
		return 1;
	}
	else
		return 0;

}
uint8_t TrackByCircle(const coordinate_t startPoint,const coordinate_t endPoint,const uint8_t Round,const uint8_t PorNFlag,const uint8_t steps,const uint8_t error)
{
	
	static float paraT=0.f;
	float k=getK(startPoint,endPoint);
	float distance=pow(Round,2)-(pow((startPoint.x-endPoint.x),2)+pow((startPoint.y-endPoint.y),2))/4;
	float sitaStart=0.f,sitaEnd=0.f;
	coordinate_t midPoint={(startPoint.x+endPoint.x)/2,(startPoint.y+endPoint.y)/2,0};
	coordinate_t centerPoint={0.f,0.f,0.f};
	k=-1/k;
	if(PorNFlag==1)
	{
		if(startPoint.x>endPoint.x)
		{
		if(startPoint.y>endPoint.y)
		{
	centerPoint.x=midPoint.x-distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y-distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint.x=midPoint.x+distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y+distance*k/(__sqrtf(pow(k,2)+1));
		}
		}
		else
		{
		if(startPoint.y>endPoint.y)
		{
	centerPoint.x=midPoint.x-distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y-distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint.x=midPoint.x+distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y+distance*k/(__sqrtf(pow(k,2)+1));
		}
	}
	sitaStart=atan2f(startPoint.y-centerPoint.y,startPoint.x-centerPoint.x)*180.f/PI;
	sitaEnd=atan2f(endPoint.y-centerPoint.y,endPoint.x-centerPoint.x)*180.f/PI;
	}
	else
	{
		if(startPoint.x>endPoint.x)
		{
		if(startPoint.y>endPoint.y)
		{
	centerPoint.x=midPoint.x+distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y+distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint.x=midPoint.x-distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y-distance*k/(__sqrtf(pow(k,2)+1));
		}
		}
		else
		{
		if(startPoint.y>endPoint.y)
		{
	centerPoint.x=midPoint.x+distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y+distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint.x=midPoint.x-distance/(__sqrtf(pow(k,2)+1));
	centerPoint.y=midPoint.y-distance*k/(__sqrtf(pow(k,2)+1));
		}
	}
	sitaStart=atan2f(startPoint.y-centerPoint.y,startPoint.x-centerPoint.x)*180.f/PI;
	sitaEnd=atan2f(endPoint.y-centerPoint.y,endPoint.x-centerPoint.x)*180.f/PI;	
	}
	gBaseCenterAim_t.x=paraT*(endPoint.x-startPoint.x)+startPoint.x;
	gBaseCenterAim_t.y=paraT*(endPoint.y-startPoint.y)+startPoint.y;
	gBaseCenterAim_t.z=paraT*(endPoint.z-startPoint.z)+startPoint.z;
	ChainCodeAimUpdate();
	if(ReachToPoint(error))
	{
		paraT+=1.f/steps;
	}
	if(paraT>1)
	{
		paraT=1;
		return 1;
	}
	else
		return 0;

}


uint8_t getTeachFinish(void)
{
return gTeachFinish;
}

/*基础参数更新*/
void ParaUpdate(void)
{
static  int chainCodeLast[3] = { 0 ,0 ,0 }; 
	
/*读编码器的绝对位置的值*/
	  ReadActualPos(1);
		ReadActualPos(2);
		ReadActualPos(3);
		
/*从中断文件里提出编码器的值*/
		int*p=getChainCode();
		gChainCode[0]=*p;
		gChainCode[1]=*(p+1);
		gChainCode[2]=*(p+2);

/*更新电机的速度向内为正*/
	for(uint8_t i=0;i<3;i++)
	gChainVel[i]=gChainCode[i]-chainCodeLast[i];
	for(uint8_t i=0;i<3;i++)
	chainCodeLast[i]=gChainCode[i];
	
/*通过编码器的值得知此时的角度*/
	gChainAngle[0]=ANGLE(gChainCode[0]);
	gChainAngle[1]=ANGLE(gChainCode[1]);
	gChainAngle[2]=ANGLE(gChainCode[2]);
	
/*更新此时坐标*/	
	BaseCenterUpdate(gChainAngle,&gBaseCenter_t);

}

/*扫描行程开关*/
uint8_t KeyCatch(void)
{
	static uint8_t status=0;
	static uint8_t statusOld=0;
	status=GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0);
	if(statusOld==0&&status==1)
	{
		statusOld=status;
		return 1;
	}
	else
	{
		statusOld=status;
		return 0;
	}	
}

