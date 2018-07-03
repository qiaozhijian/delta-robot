#include "route_ctr.h"
#include "stdio.h"
#include "flash.h"
/************global variable  define**********/
static float gBaseCenter[3] = { 0,0,-400.61f };
static  int gChainCode[3] = { 0 ,0 ,0 };
static  float gChainAngle[3] = { 45.f ,45.f ,45.f };
static  int gChainVel[3] = { 0, 0 ,0 };

/**********function claim************/
void Sprintf(float a,float b,float c,float d,float e,float f);


void ParaUpdate(void);
uint8_t KeyCatch(void);
uint8_t ReachToPoint(float baseCenterAim[3],double velocity);
static int aimOrder=0;
static float gBaseCenterAim[][3] = { 
/**��**/
39.901,85.113,-448.0,
39.901,85.113,-447.5,
38.295,80.296,-447.5,
36.444,75.099,-447.5,
33.392,68.237,-447.5,
29.823,61.677,-447.5,
26.323,56.024,-447.5,
22.732,51.108,-447.5,
26.323,56.024,-454,
	
38.287,80.295,-448.0,
38.287,80.295,-447.5,
54.244,54.687,-447.5,
38.287,80.295,-454,

29.843,52.679,-448.0,
29.843,52.679,-448.5,
50.859,52.079,-448.5,
//50.559,52.379,-454.5,
//50.459,54.379,-448.0,
//50.459,52.379,-448.0,
//50.459,52.379,-448.0,
49.059,33.819,-448.0,
49.099,31.134,-448.0,
49.066,29.230,-448.0,
47.368,28.026,-448.0,
45.571,27.947,-448.0,
39.418,28.966,-448.0,
41.571,29.047,-454,

28.860,52.410,-448.0,
28.860,52.410,-448.5,
30.910,13.155,-448.5,
31.910,10.037,-448.5,
33.236,8.090,-448.5,
35.158,9.352,-448.5,
44.905,9.352,-448.5,
47.227,9.352,-448.5,
48.975,9.352,-448.5,
54.278,11.188,-448.5,
55.255,13.676,-448.5,
55.581,16.638,-448.5,
55.818,20.221,-448.5,
55.966,23.005,-448.5,
54.966,26.322,-448.5,
55.966,24.005,-454,


60.443,75.434,-448.0,
60.443,75.434,-448.5,
60.443,24.203,-447.5,
61.443,50.434,-454,

72.895,84.087,-448.0,
72.895,84.087,-448.5,
72.948,16.750,-447.5,
72.889,13.018,-446.5,
72.119,9.672,-446.5,
70.253,7.095,-446.5,
67.054,6.177,-446.5,
62.184,12.177,-446.5,
62.184,12.177,-484,

/**��**/
99.180,84.991,-448.0,
99.180,84.991,-448.5,
102.956,72.196,-447.5,
99.180,84.991,-454,

88.118,72.152,-448.0,
88.118,72.152,-448.5,
116.781,72.181,-447.5,
100,72.152,-454,

92.451,67.169,-448.0,
92.451,67.169,-448.5,
96.963,56.155,-447.5,
92.451,67.169,-454,

109.893,67.285,-448.0,
109.893,67.285,-448.5,
104.155,52.031,-447.5,
109.893,67.285,-454,

86.691,51.989,-448.0,
86.691,51.989,-448.5,
115.199,52.049,-447.5,
100.691,51.989,-454,

86.091,36.158,-448.0,
86.091,36.158,-448.5,
114.442,36.158,-447.5,
100.091,36.158,-454,

101.203,48.508,-448.0,
101.203,48.508,-448.5,
101.186,13.456,-447.5,
100.070,9.458,-447.5,
98.738,6.459,-447.5,
97.072,5.793,-447.5,
92.274,10.859,-447.5,
92.274,10.859,-451.5,

94.084,28.061,-448.0,
94.084,28.061,-448.5,
86.421,12.934,-448.5,
94.084,28.061,-454,

106.679,27.994,-448.0,
106.679,27.994,-448.5,
113.609,15.800,-448.5,
106.679,27.994,-454,

139.831,78.883,-448.0,
139.831,78.883,-448.5,
134.545,77.194,-448.5,
127.703,75.995,-448.5,
120.151,75.106,-448.5,
120.151,75.106,-454,

120.077,75.208,-448.0,
120.077,75.208,-448.5,
120.112,49.367,-448,
120.167,43.395,-448,
119.890,35.379,-446,
119.564,29.752,-446,
118.972,23.503,-446,
117.863,17.785,-446,
116.263,11.965,-446,
112.709,3.657,-446,
116.263,11.965,-454,

120.109,49.000,-448.0,
120.109,49.000,-448.5,
143.805,50.440,-447.5,
130.109,49.000,-454,

133.396,49.437,-448.0,
133.396,49.437,-448.5,
133.398,3.160,-447.5,
133.396,10.437,-454,
133.396,10.437,-470,
0.0,0.0,0.0
};
void WriteChuangXin(void)
{
	static uint8_t step=0;
	static uint8_t status=1;
	static uint8_t key=0;
  uint8_t keyCatch;
  static float BaseCenterAim[3] = { 0 };
	ParaUpdate();
	
	switch(step)
	{
		case 0:	
			if(gBaseCenterAim[aimOrder][2]>=-5)
				aimOrder=0;
			if(gBaseCenterAim[aimOrder][2]>=-5)
			{
				BaseCenterAim[0]=0.0;
				BaseCenterAim[1]=0.0;
				BaseCenterAim[2]=-480;
			}
			else
			{
				/*��������ƫ��*/
				float a,b=0.f;
				float x,y=0.f;
				float xita=3.f/180.f*3.14f;
				a=-gBaseCenterAim[aimOrder][0]+80;
				b=gBaseCenterAim[aimOrder][1]-30;
				x=a*cos(xita)+b*sin(xita);
				y=-a*sin(xita)+b*cos(xita);
				for(uint8_t i=0;i<3;i++)
				{
					BaseCenterAim[0]=x;
					BaseCenterAim[1]=y;
					BaseCenterAim[2]=gBaseCenterAim[aimOrder][2]+3;
				}
			}

			if(ReachToPoint(BaseCenterAim,200)==1&&gBaseCenterAim[aimOrder][2]!=-530){
				aimOrder++;
			}
			break;
	}
		USART_OUT_F(aimOrder);
	for(int i=0;i<3;i++)
		USART_OUT_F(gBaseCenter[i]);
	for(int i=0;i<3;i++)
		USART_OUT_F(gBaseCenterAim[aimOrder][i]);
	USART_Enter();
}
#define PARAK   1.2f
uint8_t ReachToPoint(float baseCenterAim[3],double velocity)
{
	static float startToAim[3]={0};
	static float startPoint[3]={0};
  float startToNow[3]={0};
	float partVel[3]={0};
	double Velocity[3]={0};
	static uint8_t flag=0;
	static float Distance=0.f;
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
	/*Ŀǰ�㵽��ʼ��ľ���*/
	for(uint8_t i=0;i<3;i++)
		startToNow[i]=(gBaseCenter[i]-startPoint[i]);	
	
	/*��⿿��Ŀ��������ƫ����*/
	GetPartVel(startToAim,startToNow,PARAK,partVel);
	
	/*����Ŀ��������ƫ�������ϳ����㵽Ŀ��������*/
	for(uint8_t i=0;i<3;i++)
	{
		//if((gBaseCenter[2]-baseCenterAim[2])*(startPoint[2]-baseCenterAim[2])>0)
	Velocity[i]=partVel[i]+startToAim[i];
//		else
//	Velocity[i]=partVel[i]-startToAim[i];
			
	}
	/*���iΪ1��Խ�����յ��ٶ�Խ��*/
//	velocity*=1-Norm(startToNow)/Distance;
	
	/*�ٶ�ֵ�᲻�ϸı䣬Norm��ֵ����֮�ı䣬����noom����*/
	static float nooom=0;
	nooom=Norm(Velocity);
	/*ʹ�ٶ���Զ���ֲ���*/
	for(uint8_t i=0;i<3;i++)
		Velocity[i]=Velocity[i]*velocity/nooom;
	/*�õ���Ӧ�Ľ��ٶȣ���λ������*/
	int AngularVel[3]={0};
	AngularVelUpdate(gBaseCenter,Velocity,AngularVel);
	static float noom=0;
	noom=Norm(AngularVel);
	/*�����ٶ�������ֵ*/
	if(noom>150)
	{
	for(uint8_t i=0;i<3;i++)
		AngularVel[i]=AngularVel[i]*150/noom;
	}
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
	return 1;
	}
	else
	{
		return 0;
	}
}




static  int gAngularVel[3] = { 0, 0 ,0 };
static  double gVelocity[3] = { 0, 0 ,70 };

void DebugMode(void)
{
	
	//ParaUpdate();	
	//elmo_Disable(0);
//	ChainUpdateByBase(gBaseCenter, gChainAngle);
	//BaseCenterUpdate(gChainAngle,gBaseCenter);
	AngularVelUpdate(gBaseCenter,gVelocity,gAngularVel);
//	VelocityUpdate(gBaseCenter,gAngularVel,gVelocity);
}





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







