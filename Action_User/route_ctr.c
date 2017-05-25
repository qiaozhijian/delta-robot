#include "route_ctr.h"
#include "stdio.h"
#include "flash.h"
/************global variable  define**********/
static float gBaseCenter[3] = { 0,0,-317.61f };
static  int gChainCode[3] = { 0 ,0 ,0 };
static  float gChainAngle[3] = { 45.f ,45.f ,45.f };
static  int gChainVel[3] = { 0, 0 ,0 };

/**********function claim************/
void Sprintf(float a,float b,float c,float d,float e,float f);
#ifdef VELOCITY
void ParaUpdate(void);
uint8_t KeyCatch(void);
uint8_t ReachToPoint(float baseCenterAim[3],double velocity);
static int aimOrder=0;
static float gBaseCenterAim[55][3] = { 
40,140,-350,
40,140,-317.6,
10,110,-317.6,
10,110,-350,
30,130,-350,
30,130,-317.6,
30,50,-317.6,
30,50,-350,
70,140,-350,
70,140,-317.6,
50,110,-317.6,
50,110,-350,
57,130,-350,
57,130,-317.6,
110,125,-317.6,
110,125,-350,
110,125,-317.6,
100,110,-317.6,
100,110,-350,
85,115,-350,
85,115,-317.6,
85,50,-317.6,
85,50,-350,
85,50,-317.6,
60,60,-317.6,
70,90,-350,
70,90,-317.6,
60,80,-317.6,
60,80,-350,
100,90,-350,
100,90,-317.6,
110,80,-317.6,
110,80,-350,	
0,0,-500.0
};
void RouteControl(void)
{
	static uint8_t step=0;
	static uint8_t status=1;
	static uint8_t key=0;
  uint8_t keyCatch;
  static float BaseCenterAim[3] = { 0 };
	ParaUpdate();
	if(status==0)
	{
	keyCatch=KeyCatch();
	if(keyCatch)
	{
		switch(key)
		{
			case 0:
				elmo_Disable(0);
				AIMINPUT(gBaseCenter[0],gBaseCenter[1],gBaseCenter[2]);
				USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",(int)gBaseCenter[0],(int)gBaseCenter[1],(int)gBaseCenter[2]);
				aimOrder++;
				break;
			case 1:
				break;
			case 2:
				break;
		}
	
	}
	}
	else
	{
	switch(step)
	{
		case 0:	
			AIM(-(gBaseCenterAim[aimOrder][0]-125),gBaseCenterAim[aimOrder][1]-85,gBaseCenterAim[aimOrder][2]);
			if(ReachToPoint(BaseCenterAim,70)==1)
				aimOrder++;
			break;
	
	//	Sprintf(gAngularVel[0],gAngularVel[1],gAngularVel[2],gChainCode[0]*1.0f,gChainCode[1],gChainCode[2]);
	//USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",(int)fabs(gAngularVel[0]),(int)fabs(gAngularVel[1]),(int)fabs(gAngularVel[2]));
	}
	//VelocityUpdate(gBaseCenter,gAngularVel,gVelocity);
}
}
#define PARAK   3.f
uint8_t ReachToPoint(float baseCenterAim[3],double velocity)
{
	static float startToAim[3]={0};
	static float startPoint[3]={0};
  float startToNow[3]={0};
	float partVel[3]={0};
	double Velocity[3]={0};
	static uint8_t flag=0;
	static float Distance=0.f;
	//elmo_Disable(0);
	if(flag==0)
	{
		flag=1;
		for(uint8_t i=0;i<3;i++)
		startToAim[i]=(baseCenterAim[i]-gBaseCenter[i]);
		float norm=Norm(startToAim);
		Distance=Norm(startToAim);
		for(uint8_t i=0;i<3;i++)
		startToAim[i]=startToAim[i]*velocity/norm;
		for(uint8_t i=0;i<3;i++)
		startPoint[i]=gBaseCenter[i];
	}
	
	for(uint8_t i=0;i<3;i++)
	startToNow[i]=(gBaseCenter[i]-startPoint[i]);	
	
	GetPartVel(startToAim,startToNow,PARAK,partVel);
	
	for(uint8_t i=0;i<3;i++)
	{
		//if((gBaseCenter[2]-baseCenterAim[2])*(startPoint[2]-baseCenterAim[2])>0)
	Velocity[i]=partVel[i]+startToAim[i];
//		else
//	Velocity[i]=partVel[i]-startToAim[i];
			
	}
	/*�ٶ�ֵ�᲻�ϸı䣬Norm��ֵ����֮�ı䣬����noom����*/
	static float nooom=0;
	nooom=Norm(Velocity);
	for(uint8_t i=0;i<3;i++)
	Velocity[i]=Velocity[i]*velocity/nooom;
	int AngularVel[3]={0};
	AngularVelUpdate(gBaseCenter,Velocity,AngularVel);
	static float noom=0;
	noom=Norm(AngularVel);
	if(noom>150)
	for(uint8_t i=0;i<3;i++)
	AngularVel[i]=AngularVel[i]*150/noom;
	/*�۲����ٶ�Ϊ��������
	����ٶ�Ϊ��������ת��*/
		VelCrl(1,AngularVel[0]);
		VelCrl(2,AngularVel[1]);
		VelCrl(3,AngularVel[2]);

	if(ERROR(gBaseCenter,baseCenterAim,1)||(Norm(startToNow)>Distance))
	{
		flag=0;
		VelCrl(1,0);
		VelCrl(2,0);
		VelCrl(3,0);	
	USART_OUT(UART5,(uint8_t*)"s\r\n");
	return 1;
	}
	else
	{
	USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",(int)gBaseCenter[0],(int)gBaseCenter[1],(int)gBaseCenter[2]);
	return 0;
	}
}
#endif
#ifdef TRACK
/************global variable  define**********/
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[RECORDPOINT][3]={0};
static uint32_t trackOrder=0;
static uint8_t gTeachFinish=0;
static float gBaseCenterAim = { 0,0,ZPLAN };
/**********function claim************/
void ChainCodeAimUpdate(void);
uint8_t	ReachToPoint(uint32_t pointToReach);	
void ParaUpdate(void);


void RouteControl(void)
{
	static uint32_t pointToReach=0;
	/*��ȡ��Ҫ�ı�����ֵ���ٶ�ֵ*/
	ParaUpdate();	
//	if(!gTeachFinish)
//	ChainCodeAimUpdate();
//	else
//	{
//		if(pointToReach<=trackOrder)
//		{
//		if(ReachToPoint(pointToReach))
//			pointToReach++;
//		}
//		else
//		{
//			gTeachFinish=0;
//			pointToReach=0;
//		}
//	}
}


void ChainCodeAimUpdate(void)
{
	static uint8_t key=0;
	static float baseCenter_Old_t={0,0,0};
	static uint8_t keyCatch;
	keyCatch=KeyCatch();
	if(keyCatch)
	{
	switch(key)
	{
		case 0:
			elmo_Disable(0);
		key=1;
			break;
		case 1:
		key=2;
		trackOrder=0;
		gTeachFinish=0;
		
		baseCenter_Old_t[0]=gBaseCenter[0];
		baseCenter_Old_t[1]=gBaseCenter[1];
		baseCenter_Old_t[2]=gBaseCenter[2];
			break;
		case 2:
		key=3;
		keyCatch=0;
		elmo_Enable(1);
		elmo_Enable(2);
		elmo_Enable(3);
		gTeachFinish=1;
			break;
		case 3:
			key=0;
			PosCrl(1,0,-100);
			PosCrl(2,0,-100);
			PosCrl(3,0,-100);
			break;
	}
}
	if(TOUCH(baseCenter_Old_t,gBaseCenter,2)&&key==2)
	{
	//Sprintf(baseCenter_Old_t[0],baseCenter_Old_t[1],baseCenter_Old_t[2],gBaseCenter[0],gBaseCenter[1],gBaseCenter[2]);
		for(uint8_t i=0;i<3;i++)
		{
		gChainCodeAim[trackOrder][i]=gChainCode[i];
		}
		USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",gChainCode[0],gChainCode[1],gChainCode[2]);
		trackOrder++;
		//����һ�η�һ�η����꣬�ĳ����һ��
		baseCenter_Old_t[0]=gBaseCenter[0];
		baseCenter_Old_t[1]=gBaseCenter[1];
		baseCenter_Old_t[2]=gBaseCenter[2];
	}
}

uint8_t	ReachToPoint(uint32_t pointToReach)
{
	static int chainAim[3]={0};
	if(pointToReach==1)
	{
	Pos_cfg(1,1000000,1000000,10);
	Pos_cfg(2,1000000,1000000,10);
	Pos_cfg(3,1000000,1000000,10);
	}
	for(uint8_t i=0;i<3;i++)
	{
	chainAim[i]=gChainCodeAim[pointToReach][i];
	gChainAngleAim[i]=ANGLE(chainAim[i]);
	}
	PosCrl(1,0,chainAim[0]);
	PosCrl(2,0,chainAim[1]);
	PosCrl(3,0,chainAim[2]);
	BaseCenterUpdate(gChainAngleAim,&gBaseCenterAim);
	//if((abs(chainAim[0]-gChainCode[0])<10)&&(abs(chainAim[1]-gChainCode[1])<10)&&(abs(chainAim[2]-gChainCode[2])<10))
	if((fabs(gBaseCenterAim[0]-gBaseCenter[0])<0.5f)&&(fabs(gBaseCenterAim[1]-gBaseCenter[1])<0.5f)&&(fabs(gBaseCenterAim[2]-gBaseCenter[2])<0.5f))
	{
		//USART_OUT(UART5,(uint8_t*)"%d\t%d\t%d\r\n",chainAim[0],chainAim[1],chainAim[2]);
		return 1;
	}
	else
		return 0;
}

#endif


#ifdef DEGUG
static  int gAngularVel[3] = { 0, 0 ,0 };
static  double gVelocity[3] = { 0, 0 ,50 };
void	ParaUpdate(void);	
void RouteControl(void)
{
	
	ParaUpdate();	
	ChainUpdateByBase(gBaseCenter, gChainAngle);
	BaseCenterUpdate(gChainAngle,gBaseCenter);
	AngularVelUpdate(gBaseCenter,gVelocity,gAngularVel);
//	VelocityUpdate(gBaseCenter,gAngularVel,gVelocity);
}
#endif


#ifdef TRYPOINT
/************global variable  define**********/

static float gBaseCenterAim = { 0,0,ZPLAN };
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[3]={0};
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
		gBaseCenterAim[0]=instruct[1]+instruct[2];
	else
		gBaseCenterAim[0]=-(instruct[1]+instruct[2]);
	
	if(instruct[3]=='+')
		gBaseCenterAim[1]=instruct[4]+instruct[5];
	else
		gBaseCenterAim[1]=-(instruct[4]+instruct[5]);
	
		gBaseCenterAim[2]=-(instruct[6]+instruct[7]+instruct[8]);
	if(*(pInstruct+intrctOrder*15+0)!='G')
	{
		gBaseCenterAim[0]=0;
		gBaseCenterAim[1]=0;
		gBaseCenterAim[2]=-400;
	}
	ChainUpdateByBase(gBaseCenterAim, gChainAngleAim);
	for(uint8_t i=0;i<3;i++)
	gChainCodeAim[i]=CODE(gChainAngleAim[i]);
	ReachToPoint(1);
	BaseCenterUpdate(gChainAngle,&gBaseCenter);
}

uint8_t ReachToPoint(uint8_t error)
{
	for(uint8_t i=0;i<3;i++)
	PosCrl(i+1,0,gChainCodeAim[i]);
	/*��õ�ʱ�������*/
	BaseCenterUpdate(gChainAngle,&gBaseCenter);
	if(ERROR(gBaseCenter,gBaseCenterAim,error))
	{
	Sprintf(gBaseCenterAim[0],gBaseCenterAim[1],gBaseCenterAim[2],gBaseCenter[0],gBaseCenter[1],gBaseCenter[2]);
	return 1;
	}
	else
	return 0;
}
#endif


#ifdef ROUTEPLAN
/************global variable  define**********/
static float gBaseCenterAim = { 0,0,ZPLAN };
static float gChainAngleAim[3]={0};
static  int gChainCodeAim[3]={0};
/**********function claim************/
void ParaUpdate(void);
uint8_t TrackByLine(float startPoint,float endPoint,uint8_t steps,uint8_t error);
void RouteControl(void)
{
	ParaUpdate();
	static uint8_t mode=0;
	static float startPoint={0.f,0.f,-270.f};
	static float endPoint={20.f,0.f,-270.f};
	float gAngularVel[3] = { 0, 0 ,0 };
	static float gVelocity[3] = { 50, 0 ,0 };
	
	
	
	
	TrackByLine(startPoint,endPoint,6,5);

}
void ChainCodeAimUpdate(void)
{
	ChainUpdateByBase(gBaseCenterAim, gChainAngleAim);

	for(uint8_t i=0;i<3;i++)
	gChainCodeAim[i]=CODE(gChainAngleAim[i]);
}
uint8_t ReachToPoint(uint8_t error)
{
	for(uint8_t i=0;i<3;i++)
	PosCrl(i+1,0,gChainCodeAim[i]);
	/*��õ�ʱ�������*/
	BaseCenterUpdate(gChainAngle,&gBaseCenter);
	if(ERROR(gBaseCenter,gBaseCenterAim,error))
	return 1;
	else
	return 0;
}
uint8_t TrackByLine(float startPoint,float endPoint,uint8_t steps,uint8_t error)
{
	
	static float paraT=0.f;
	gBaseCenterAim[0]=paraT*(endPoint[0]-startPoint[0])+startPoint[0];
	gBaseCenterAim[1]=paraT*(endPoint[1]-startPoint[1])+startPoint[1];
	gBaseCenterAim[2]=paraT*(endPoint[2]-startPoint[2])+startPoint[2];
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
uint8_t TrackByCircle(const float startPoint,const float endPoint,const uint8_t Round,const uint8_t PorNFlag,const uint8_t steps,const uint8_t error)
{
	
	static float paraT=0.f;
	float k=getK(startPoint,endPoint);
	float distance=pow(Round,2)-(pow((startPoint[0]-endPoint[0]),2)+pow((startPoint[1]-endPoint[1]),2))/4;
	float sitaStart=0.f,sitaEnd=0.f;
	float midPoint={(startPoint[0]+endPoint[0])/2,(startPoint[1]+endPoint[1])/2,0};
	float centerPoint={0.f,0.f,0.f};
	k=-1/k;
	if(PorNFlag==1)
	{
		if(startPoint[0]>endPoint[0])
		{
		if(startPoint[1]>endPoint[1])
		{
	centerPoint[0]=midPoint[0]-distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]-distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint[0]=midPoint[0]+distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]+distance*k/(__sqrtf(pow(k,2)+1));
		}
		}
		else
		{
		if(startPoint[1]>endPoint[1])
		{
	centerPoint[0]=midPoint[0]-distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]-distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint[0]=midPoint[0]+distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]+distance*k/(__sqrtf(pow(k,2)+1));
		}
	}
	sitaStart=atan2f(startPoint[1]-centerPoint[1],startPoint[0]-centerPoint[0])*180.f/PI;
	sitaEnd=atan2f(endPoint[1]-centerPoint[1],endPoint[0]-centerPoint[0])*180.f/PI;
	}
	else
	{
		if(startPoint[0]>endPoint[0])
		{
		if(startPoint[1]>endPoint[1])
		{
	centerPoint[0]=midPoint[0]+distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]+distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint[0]=midPoint[0]-distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]-distance*k/(__sqrtf(pow(k,2)+1));
		}
		}
		else
		{
		if(startPoint[1]>endPoint[1])
		{
	centerPoint[0]=midPoint[0]+distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]+distance*k/(__sqrtf(pow(k,2)+1));
		}
		else
		{
	centerPoint[0]=midPoint[0]-distance/(__sqrtf(pow(k,2)+1));
	centerPoint[1]=midPoint[1]-distance*k/(__sqrtf(pow(k,2)+1));
		}
	}
	sitaStart=atan2f(startPoint[1]-centerPoint[1],startPoint[0]-centerPoint[0])*180.f/PI;
	sitaEnd=atan2f(endPoint[1]-centerPoint[1],endPoint[0]-centerPoint[0])*180.f/PI;	
	}
	gBaseCenterAim[0]=paraT*(endPoint[0]-startPoint[0])+startPoint[0];
	gBaseCenterAim[1]=paraT*(endPoint[1]-startPoint[1])+startPoint[1];
	gBaseCenterAim[2]=paraT*(endPoint[2]-startPoint[2])+startPoint[2];
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
/*�����ڴ汨��*/


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
	static float baseCenter_Old_t={0,0,0};
	static uint8_t keyCatch;
	static uint8_t indexForPool=0;
	static int *result;
	
	result=GetResultArr();
	keyCatch=KeyCatch();
	/*һ����ʾ����*/
	if(keyCatch&&key==1)
	{
		indexForPool++;
		key=0;
		Flash_Write(GetFlashArr(),4000);
	}
	/*һ����ʾ��ʼ*/
	if(keyCatch&&key==0)
	{
		key=1;
		baseCenter_Old_t[0]=gBaseCenter[0];
		baseCenter_Old_t[1]=gBaseCenter[1];
		baseCenter_Old_t[2]=gBaseCenter[2];
	}
	/*��trackorderд��һ�����飬���ÿ��trackorder�����ΰ����������trackorder==0����дflash*/
	if(TOUCH(baseCenter_Old_t,gBaseCenter,7)&&key==1)
	{
		for(uint8_t i=0;i<3;i++)
		{
		gChainCodeAim[trackOrder[indexForPool]][i]=gChainCode[i];
		*(result+trackOrder[indexForPool]*3+i)=gChainCodeAim[trackOrder[indexForPool]][i];
		}
		trackOrder[indexForPool]++;
		//����һ�η�һ�η����꣬�ĳ����һ��
		baseCenter_Old_t[0]=gBaseCenter[0];
		baseCenter_Old_t[1]=gBaseCenter[1];
		baseCenter_Old_t[2]=gBaseCenter[2];
	}
	
	return 1;
}
#endif






void Sprintf(float a,float b,float c,float d,float e,float f)
{
	unsigned char buff[10];
	sprintf((char*)buff,"%f",a);
	USART_OUT(UART5,buff);
	USART_OUT(UART5,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",b);
	USART_OUT(UART5,buff);
	USART_OUT(UART5,(uint8_t *)"\t");
	sprintf((char*)buff,"%f",c);
	USART_OUT(UART5,buff);
//	USART_OUT(UART5,(uint8_t *)"\t");
//	sprintf((char*)buff,"%f\r\n",d);
//	USART_OUT(UART5,buff);
//	USART_OUT(UART5,(uint8_t *)"\t");
//	sprintf((char*)buff,"%f",e);
//	USART_OUT(UART5,buff);
//	USART_OUT(UART5,(uint8_t *)"\t");
//	sprintf((char*)buff,"%f",f);
//	USART_OUT(UART5,buff);
	USART_OUT(UART5,(uint8_t *)"\r\n");
}





/*������������*/
void ParaUpdate(void)
{
static  int chainCodeLast[3] = { 0 ,0 ,0 }; 
	
/*���������ľ���λ�õ�ֵ*/
	  ReadActualPos(1);
		ReadActualPos(2);
		ReadActualPos(3);
		
/*���ж��ļ��������������ֵ*/
		int*p=getChainCode();
		gChainCode[0]=*p;
		gChainCode[1]=*(p+1);
		gChainCode[2]=*(p+2);

///*���µ�����ٶ�����Ϊ��*/
	for(uint8_t i=0;i<3;i++)
	gChainVel[i]=gChainCode[i]-chainCodeLast[i];
	for(uint8_t i=0;i<3;i++)
	chainCodeLast[i]=gChainCode[i];
	
/*ͨ����������ֵ��֪��ʱ�ĽǶ�*/
	gChainAngle[0]=ANGLE(gChainCode[0]);
	gChainAngle[1]=ANGLE(gChainCode[1]);
	gChainAngle[2]=ANGLE(gChainCode[2]);
	
/*���´�ʱ����*/	
	BaseCenterUpdate(gChainAngle,gBaseCenter);

}





/*ɨ���г̿���*/
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

