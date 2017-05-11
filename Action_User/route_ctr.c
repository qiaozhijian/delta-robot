#include "route_ctr.h"
#include "stdio.h"
#include "flash.h"
/************global variable  define**********/
static coordinate_t gBaseCenter_t = { 0,0,BASESTARTPOS };
static  int gChainCode[3] = { 0 ,0 ,0 };
static  float gChainAngle[3] = { STARTANGLE ,STARTANGLE ,STARTANGLE };
//static  int gChainVel[3] = { 0, 0 ,0 };

/**********function claim************/
void Sprintf(float a,float b,float c,float d,float e,float f);

#ifdef TRACK
/************global variable  define**********/
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[RECORDPOINT][3]={0};
static uint16_t trackOrder=0;
static uint8_t gTeachFinish=0;
static coordinate_t gBaseCenterAim_t = { 0,0,ZPLAN };
/**********function claim************/
void ChainCodeAimUpdate(void);
uint8_t	ReachToPoint(uint8_t pointToReach);	
void ParaUpdate(void);


void RouteControl(void)
{
	ParaUpdate();
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
}


void ChainCodeAimUpdate(void)
{
	static uint8_t key=0;
	static coordinate_t baseCenter_Old_t={0,0,0};
	static uint8_t keyCatch;
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
	if(TOUCH(baseCenter_Old_t,gBaseCenter_t,6)&&key==1)
	{
	//Sprintf(baseCenter_Old_t.x,baseCenter_Old_t.y,baseCenter_Old_t.z,gBaseCenter_t.x,gBaseCenter_t.y,gBaseCenter_t.z);
		for(uint8_t i=0;i<3;i++)
		{
		gChainCodeAim[trackOrder][i]=gChainCode[i];
		}
		USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",gChainCode[0],gChainCode[1],gChainCode[2]);
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
	for(uint8_t i=0;i<3;i++)
	{
	chainAim[i]=gChainCodeAim[pointToReach][i];
	gChainAngleAim[i]=ANGLE(chainAim[i]);
	}
	PosCrl(1,0,chainAim[0]);
	PosCrl(2,0,chainAim[1]);
	PosCrl(3,0,chainAim[2]);
	BaseCenterUpdate(gChainAngleAim,&gBaseCenterAim_t);
	//if((abs(chainAim[0]-gChainCode[0])<10)&&(abs(chainAim[1]-gChainCode[1])<10)&&(abs(chainAim[2]-gChainCode[2])<10))
	if((fabs(gBaseCenterAim_t.x-gBaseCenter_t.x)<3)&&(fabs(gBaseCenterAim_t.y-gBaseCenter_t.y)<3)&&(fabs(gBaseCenterAim_t.z-gBaseCenter_t.z)<3))
	{
		USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",chainAim[0],chainAim[1],chainAim[2]);
		return 1;
	}
	else
		return 0;
}

#endif


#ifdef DEGUG
static  float gAngularVel[3] = { 0, 0 ,0 };
static  float gVelocity[3] = { 0, 0 ,0 };
void RouteControl(void)
{
	//BaseCenterUpdate(gChainAngle,&gBaseCenter_t);
	ChainUpdateByBase(gBaseCenter_t, gChainAngle);
	AngularVelUpdate(gBaseCenter_t,gChainAngle,gVelocity,gAngularVel);
	VelocityUpdate(gBaseCenter_t,gChainAngle,gAngularVel,gVelocity);
}
#endif


#ifdef TRYPOINT
/************global variable  define**********/

static coordinate_t gBaseCenterAim_t = { 0,0,ZPLAN };
static coordinate_t gRBaseCenterAim_t = { 0,0,ZPLAN };
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[3]={0};
static float preValue[3]={0};
/**********function claim************/
void ParaUpdate(void);
uint8_t ReachToPoint(uint8_t error);
void RouteControl(void)
{
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
	for(int i=0;i<3;i++)
	{
	gRBaseCenterAim_t.x=gBaseCenterAim_t.x+preValue[0];
	gRBaseCenterAim_t.y=gBaseCenterAim_t.y+preValue[1];
	gRBaseCenterAim_t.z=gBaseCenterAim_t.z+preValue[2];
	}
	ChainUpdateByBase(gRBaseCenterAim_t, gChainAngleAim);
	for(uint8_t i=0;i<3;i++)
	gChainCodeAim[i]=CODE(gChainAngleAim[i]);
	ReachToPoint(3);
	BaseCenterUpdate(gChainAngle,&gBaseCenter_t);
}

uint8_t ReachToPoint(uint8_t error)
{
	static int CodeLast[3]={0};
	static int countForAdd=0;
	
	for(uint8_t i=0;i<3;i++)
	PosCrl(i+1,0,gChainCodeAim[i]);
	/*获得当时坐标编码*/
	BaseCenterUpdate(gChainAngle,&gBaseCenter_t);
	if(ERROR(gBaseCenter_t,gBaseCenterAim_t,error))
	{
	for(uint8_t i=0;i<3;i++)
		CodeLast[i]=gChainCode[i];
//	for(int i=0;i<3;i++)
//	{
//		preValue[i]=0;
//	}
	return 1;
	}
	else
	{
	if((CodeLast[0]==gChainCode[0])&&(CodeLast[1]==gChainCode[1])&&(CodeLast[2]==gChainCode[2]))
		countForAdd++;
	else
		countForAdd=0;
	if(countForAdd>100)
	{
	countForAdd=0;
	if(gBaseCenterAim_t.x>gBaseCenter_t.x)
		preValue[0]++;
	else
		preValue[0]--;
	if(gBaseCenterAim_t.y>gBaseCenter_t.y)
		preValue[1]++;
	else
		preValue[1]--;
	if(gBaseCenterAim_t.z>gBaseCenter_t.z)
		preValue[2]++;
	else
		preValue[2]--;
	for(int i=0;i<3;i++)
	{
	if(fabs(preValue[i])>10)
		preValue[i]=0;
	}
	}
	for(int i=0;i<3;i++)
	CodeLast[i]=gChainCode[i];
	return 0;
	}
}
#endif


#ifdef ROUTEPLAN
/************global variable  define**********/
static coordinate_t gBaseCenterAim_t = { 0,0,ZPLAN };
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[3]={0};
/**********function claim************/
void ParaUpdate(void);
uint8_t TrackByLine(coordinate_t startPoint,coordinate_t endPoint,uint8_t steps,uint8_t error);
void RouteControl(void)
{
	ParaUpdate();
	static uint8_t mode=0;
	static coordinate_t startPoint={0.f,0.f,-270.f};
	static coordinate_t endPoint={20.f,0.f,-270.f};
	TrackByLine(startPoint,endPoint,6,5);

}
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
#endif


#ifdef DATAPOOL
/************global variable  define**********/
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[RECORDPOINT][3]={0};
static uint16_t trackOrder[10]={0};
static uint32_t countNum=0;
void test(void)
{
//int aaa[POOLSIZE]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//static int a=1;
//int *result=GetResultArr();

//if(a==0)
//{
//for(int i=0;i<POOLSIZE;i++)
//aaa[i]=1;
//for(int i=0;i<POOLSIZE;i++)
//*(result+i)=aaa[i];
//Flash_Write(GetFlashArr(),POOLSIZE*4);
//	a=1;
//}
//if(a==1)
//{
//	a=2;
//result=GetResultArr();
//for(int i=0;i<4;i++)
//	USART_OUT(UART5,(uint8_t*)"%d\r\n",*(result+i+POOLSIZE-4)+1);
//}
/*超出内存报警*/


static int aaa[2]={0};

if((aaa[0]!=*getPInstruct())||(aaa[1]!=*(getPInstruct()+1)))
{
	aaa[0]=*getPInstruct();
	aaa[1]=*(getPInstruct()+1);
	USART_OUT(UART5,(uint8_t*)"%s\r\n","input one successfully");
}
	


}
/**********function claim************/
void ParaUpdate(void);
uint8_t DataInput(void);

void RouteControl(void)
{
		//ParaUpdate();
		//DataInput();
	test();
}

uint8_t DataInput(void)
{
  static uint8_t key=0;
	static coordinate_t baseCenter_Old_t={0,0,0};
	static uint8_t keyCatch;
	static uint8_t indexForPool=0;
	static int *result;
	
	result=GetResultArr();
	keyCatch=KeyCatch();
	/*一次演示结束*/
	if(keyCatch&&key==1)
	{
		indexForPool++;
		key=0;
		Flash_Write(GetFlashArr(),4000);
	}
	/*一次演示开始*/
	if(keyCatch&&key==0)
	{
		key=1;
		baseCenter_Old_t.x=gBaseCenter_t.x;
		baseCenter_Old_t.y=gBaseCenter_t.y;
		baseCenter_Old_t.z=gBaseCenter_t.z;
	}
	/*把trackorder写成一个数组，输出每个trackorder，两次按键结束如果trackorder==0，则写flash*/
	if(TOUCH(baseCenter_Old_t,gBaseCenter_t,7)&&key==1)
	{
		for(uint8_t i=0;i<3;i++)
		{
		gChainCodeAim[trackOrder[indexForPool]][i]=gChainCode[i];
		*(result+trackOrder[indexForPool]*3+i)=gChainCodeAim[trackOrder[indexForPool]][i];
		}
		trackOrder[indexForPool]++;
		//进来一次发一次发不完，改成最后一起发
		baseCenter_Old_t.x=gBaseCenter_t.x;
		baseCenter_Old_t.y=gBaseCenter_t.y;
		baseCenter_Old_t.z=gBaseCenter_t.z;
	}
	
	return 1;
}
#endif






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





/*基础参数更新*/
void ParaUpdate(void)
{
//static  int chainCodeLast[3] = { 0 ,0 ,0 }; 
	
/*读编码器的绝对位置的值*/
	  ReadActualPos(1);
		ReadActualPos(2);
		ReadActualPos(3);
		
/*从中断文件里提出编码器的值*/
		int*p=getChainCode();
		gChainCode[0]=*p;
		gChainCode[1]=*(p+1);
		gChainCode[2]=*(p+2);

///*更新电机的速度向内为正*/
//	for(uint8_t i=0;i<3;i++)
//	gChainVel[i]=gChainCode[i]-chainCodeLast[i];
//	for(uint8_t i=0;i<3;i++)
//	chainCodeLast[i]=gChainCode[i];
	
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

