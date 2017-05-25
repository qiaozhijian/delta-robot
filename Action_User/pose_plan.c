#include "pose_plan.h"
#include "stdio.h"
/************variable  define**********/
const float gAxisAngle[3] = { 0.f,120.f/180.f*PI,240.f/180.f*PI };

/*ͨ����������ĽǶ�ȥ���¹ؽ�����*/
void LinkCoordinateUpdate(const float ChainAngle[3],float pLinkCoordinate[3][3])
{
	
	for (uint8_t i = 0; i < 3; i++)
	{
		pLinkCoordinate[i][0] = (R - r + MASTERARM*arm_cos_f32(ChainAngle[i] / 180.f*PI))*arm_cos_f32(i*120.f / 180.f*PI);
		pLinkCoordinate[i][1] = (R - r + MASTERARM*arm_cos_f32(ChainAngle[i] / 180.f*PI))*arm_sin_f32(i*120.f / 180.f*PI);
		pLinkCoordinate[i][2] = -MASTERARM*arm_sin_f32(ChainAngle[i] / 180.f*PI);
	}
}

/*�������Ĳ��*/
void VectorCross(const float vector1[3], const float vector2[3], float vectorResult[3])
{

	vectorResult[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	vectorResult[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
	vectorResult[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];

}
/*���˵ĵ�λ����*/
void UnitVector(const float vector1[3], const float vector2[3], float unitVector[3])
{
	float vector[3]={0};
	float length = 0.f;
  VectorCross(vector1, vector2,vector);
	length = Norm(vector);
	for (uint8_t i = 0; i < 3; i++)
	{
		unitVector[i] = vector[i] / length;
	}
}
/*���˵ĵ�λ����*/
void UnitVector3(const float vector1[3], const float vector2[3], const float vector3[3],float unitVector[3])
{
	float vectorTemp1[3]={0};
	float vectorTemp2[3]={0};
	float length = 0.f;
	VectorCross(vector1, vector2,vectorTemp1);
	VectorCross(vectorTemp1, vector3,vectorTemp2);
	length = Norm(vectorTemp2);
	for (uint8_t i = 0; i < 3; i++)
	{
		unitVector[i] = vectorTemp2[i] / length;
	}
}

float Distance(	const float startToAim[3],const float startToNow[3])
{
	float temp[3]={0};
	float result=0.f;
	VectorCross(startToAim, startToNow,temp);
	result=Norm(temp);
	result=result/Norm(startToAim);
	return result;
}

void GetPartVel(const float startToAim[3],const float startToNow[3],const float K,float partVel[3])
{
	/*��ǰ�����ĳ���*/
	float dStartToNow=Norm(startToNow);
	/*��ʼ�㵽Ŀ�������ľ���*/
	float dVertical=Distance(	startToAim,startToNow);
	/*Ŀ�������ĳ���*/
	float dStartToAim=Norm(startToAim);
	/*��ǰ����Ŀ�������ϵ�ͶӰ��*/
	float verticalPoint[3]={0};
	
	for(uint8_t i=0;i<3;i++)
	verticalPoint[i]=__sqrtf(pow(dStartToNow,2)-pow(dVertical,2))/dStartToAim*startToAim[i];
	
	for(uint8_t i=0;i<3;i++)
	partVel[i]=K*(verticalPoint[i]-startToNow[i]);
	
//	double noom=Norm(partVel);
//	if(noom>dStartToAim)
//	{
//		for(uint8_t i=0;i<3;i++)
//	partVel[i]=partVel[i]/noom*dStartToAim;
//	}
}
/*��������һ������*/
void VectorSolveBY2Pouint8_t(const float pLinkCoordinate1[3],const float pLinkCoordinate2[3], float vector[3])
{
	vector[0] = pLinkCoordinate2[0] - pLinkCoordinate1[0];
	vector[1] = pLinkCoordinate2[1] - pLinkCoordinate1[1];
	vector[2] = pLinkCoordinate2[2] - pLinkCoordinate1[2];
}
/*������������*/
void BaseCenterUpdate(const float ChainAngle[3],float BaseCenter_t[3])
{
	float LinkCoordinate[3][3]={0};
	LinkCoordinateUpdate(ChainAngle,LinkCoordinate);
	float H = 0.f;
	float S = 0.f;

	float vectorE1E2[3] = { 0.f };
	float vectorE2E3[3] = { 0.f };
	float vectorE3E1[3] = { 0.f };
	float vectorGF[3] = { 0.f };
	float vectorFP[3] = { 0.f };

	float LenE1E2 = 0.f;
	float LenE2E3 = 0.f;
	float LenE3E1 = 0.f;
	float LenFE = 0.f;
	float LenGF = 0.f;
	float LenFP = 0.f;

	VectorSolveBY2Pouint8_t(LinkCoordinate[0], LinkCoordinate[1], vectorE1E2);
	VectorSolveBY2Pouint8_t(LinkCoordinate[1], LinkCoordinate[2], vectorE2E3);
	VectorSolveBY2Pouint8_t(LinkCoordinate[2], LinkCoordinate[0], vectorE3E1);
	
	LenE1E2 = Norm(vectorE1E2);
	LenE2E3 = Norm(vectorE2E3);
	LenE3E1 = Norm(vectorE3E1);
	
	H = (LenE1E2 + LenE2E3 + LenE3E1) / 2.f;
	S = __sqrtf(H*(H - LenE1E2)*(H - LenE2E3)*(H - LenE3E1));
	LenFE = LenE1E2*LenE2E3*LenE3E1 / 4.f / S;
	LenGF = __sqrtf(pow(LenFE, 2) - pow(LenE1E2, 2) / 4.f);
	LenFP = __sqrtf(pow(SLAVEARM, 2) - pow(LenFE, 2));

//	UnitVector(vectorE2E3, vectorE3E1, vectorGF);
//	UnitVector(vectorGF, vectorE1E2, vectorGF);
	UnitVector3(vectorE2E3, vectorE3E1, vectorE1E2,vectorGF);
	/*�˴�FP������ȡ���ں����������*/
	UnitVector(vectorE1E2, vectorE2E3, vectorFP);

	BaseCenter_t[0] = (LinkCoordinate[0][0] +  LinkCoordinate[1][0])*0.5f + LenGF*vectorGF[0] - vectorFP[0] * LenFP;
	BaseCenter_t[1] = (LinkCoordinate[0][1] +  LinkCoordinate[1][1])*0.5f + LenGF*vectorGF[1] - vectorFP[1] * LenFP;
	BaseCenter_t[2] = (LinkCoordinate[0][2] +  LinkCoordinate[1][2])*0.5f + LenGF*vectorGF[2] - vectorFP[2] * LenFP;

}
/*ͨ�����귴���µ���Ƕ�*/
void ChainUpdateByBase(const float BaseCenter_t[3], float ChainAngle[3])
{
	float paraR[3] = { 0.f };
	float paraS[3] = { 0.f };
	float paraT[3] = { 0.f };

	for (uint8_t i = 0; i < 3; i++)
	{
		paraR[i] = -2 * MASTERARM *(arm_cos_f32(gAxisAngle[i])*((r - R)*arm_cos_f32(gAxisAngle[i]) + BaseCenter_t[0]) + \
		 arm_sin_f32(gAxisAngle[i])*(BaseCenter_t[1] + (r - R)*arm_sin_f32(gAxisAngle[i])));

		paraS[i] = 2 * MASTERARM*BaseCenter_t[2];

		paraT[i] = pow((BaseCenter_t[0] + (r - R)*arm_cos_f32(gAxisAngle[i])), 2) + \
			pow((BaseCenter_t[1] + (r - R)*arm_sin_f32(gAxisAngle[i])), 2) + pow(BaseCenter_t[2], 2) + \
			pow(MASTERARM, 2) - pow(SLAVEARM, 2);
	}

	for (uint8_t i = 0; i < 3; i++)
	{
		/*������ϵ��Ϊ0ʱ*/
		if (fabs(paraT[i] - paraR[i]) < 0.01)
			ChainAngle[i] = 2 * atan2f(-paraR[i]-paraT[i],2*paraS[i]);
		else
			ChainAngle[i] = 2 * atan2f(-paraS[i]-__sqrtf(pow(paraR[i],2)+pow(paraS[i],2)-pow(paraT[i],2)), paraT[i] - paraR[i]);
		
	}

	
	for (uint8_t i = 0; i < 3; i++)
	{
		ChainAngle[i]=ChainAngle[i]/PI*180.f;
		if (ChainAngle[i] > 0.5f*360.f)
			ChainAngle[i] -= 180.f * 2;
		if (ChainAngle[i] < -0.5f*360.f)
			ChainAngle[i] += 180.f * 2;
//		if (ChainAngle[i] > 0.66f*180.f)
//			ChainAngle[i] = 0.66f*180.f;
//		if (ChainAngle[i] < 180.f / 6.f)
//			ChainAngle[i] = 180.f / 6.f;
	}
}
/*ͨ��������ٶȸ��������ٶ�*/
void VelocityUpdate(const float BaseCenter_t[3],const int AngularVel[3],float Velocity[3])
{
		float ChainAngle[3]={0};
		ChainUpdateByBase(BaseCenter_t,ChainAngle);
	  float ChainRad[3]={ChainAngle[0]/180.f*PI,ChainAngle[1]/180.f*PI,ChainAngle[2]/180.f*PI};
	  arm_matrix_instance_f32 jacobiA;
		arm_matrix_instance_f32 jacobiAInverse;
		arm_matrix_instance_f32 jacobiB;
		arm_matrix_instance_f32 Jacobi;
		arm_matrix_instance_f32 negativeMat;
		arm_matrix_instance_f32 velocity;
		arm_matrix_instance_f32 angularVel;
	
		float jacobiAArray[9]={0};
		for(uint8_t i=0;i<3;i++)
		{
		jacobiAArray[i*3]=BaseCenter_t[0]-arm_cos_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
		jacobiAArray[i*3+1]=BaseCenter_t[1]-arm_sin_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
		jacobiAArray[i*3+2]=BaseCenter_t[2]+MASTERARM*arm_sin_f32(ChainRad[i]);
		}
		float jacobiAInverseArray[9]={0};
		
		float jacobiBArray[9]={0};
		for(uint8_t i=0;i<3;i++)
		{
		jacobiBArray[i*4]=MASTERARM*arm_sin_f32(ChainRad[i])*(BaseCenter_t[0]*arm_cos_f32(gAxisAngle[i])+BaseCenter_t[1]*arm_sin_f32(gAxisAngle[i])/
			-R+r)+BaseCenter_t[2]*MASTERARM*arm_cos_f32(ChainRad[i]);
		}
		float JacobiArray[9]={0};
		float negativeMatArray[9]={
		-1,0,0,
		0,-1,0,
		0,0,-1
		};
		float velocityArray[3]={0};
		float angularVelArray[3]={ANGLE(AngularVel[0])/180.f*PI,ANGLE(AngularVel[1])/180.f*PI,ANGLE(AngularVel[2])/180.f*PI};
		
		arm_mat_init_f32(&Jacobi,3,3,JacobiArray);
		arm_mat_init_f32(&jacobiA,3,3,jacobiAArray);
		arm_mat_init_f32(&jacobiAInverse,3,3,jacobiAInverseArray);
		arm_mat_init_f32(&jacobiB,3,3,jacobiBArray);
		arm_mat_init_f32(&negativeMat,3,3,negativeMatArray);
		arm_mat_init_f32(&velocity,3,1,velocityArray);
		arm_mat_init_f32(&angularVel,3,1,angularVelArray);		
		
		arm_mat_inverse_f32(&jacobiA,&jacobiAInverse);
	  arm_mat_mult_f32(&jacobiAInverse,&jacobiB,&Jacobi);
		arm_mat_mult_f32(&negativeMat,&Jacobi,&Jacobi);
		arm_mat_mult_f32(&Jacobi,&angularVel,&velocity);
		
		for(uint8_t i=0;i<3;i++)
		{
		Velocity[i]=*(velocity.pData+i);
		}
		
}
void CrossInverse(double matrixB[9]);
void CrossMultiply(double matrixB[9],double matrixA[9],double matrix[9]);
void tripeMultiplyOne(double matrixB[9],double matrixA[3],double matrix[3]);
/*ͨ�������ٶȸ��µ�����ٶ�*/
void AngularVelUpdate(const float BaseCenter_t[3],const double Velocity[3],int AngularVelCode[3])
{
		double AngularVel[3]={0};
		float ChainAngle[3]={0};
		ChainUpdateByBase(BaseCenter_t,ChainAngle);
	  double ChainRad[3]={ChainAngle[0]/180*3.1415926,ChainAngle[1]/180*3.1415926,ChainAngle[2]/180*3.1415926};
		
		double jacobiAArray[9]={0};
		for(uint8_t i=0;i<3;i++)
		{
		jacobiAArray[i*3]=BaseCenter_t[0]-arm_cos_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
		jacobiAArray[i*3+1]=BaseCenter_t[1]-arm_sin_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
		jacobiAArray[i*3+2]=BaseCenter_t[2]+MASTERARM*arm_sin_f32(ChainRad[i]);
		}
		
		double jacobiBArray[9]={0};
		for(uint8_t i=0;i<3;i++)
		{
		jacobiBArray[i*4]=MASTERARM*arm_sin_f32(ChainRad[i])*(BaseCenter_t[0]*arm_cos_f32(gAxisAngle[i])+BaseCenter_t[1]*arm_sin_f32(gAxisAngle[i])-R+r)+BaseCenter_t[2]*MASTERARM*arm_cos_f32(ChainRad[i]);
		}
		double JacobiArray[9]={0};
		double velocityArray[3]={Velocity[0],Velocity[1],Velocity[2]};
		double angularVelArray[3]={0};
		
		CrossInverse(jacobiBArray);
		CrossMultiply(jacobiBArray,jacobiAArray,JacobiArray);
		tripeMultiplyOne(JacobiArray,velocityArray,angularVelArray);
		
		for(uint8_t i=0;i<3;i++)
		{
		AngularVel[i]=angularVelArray[i]/PI*180.f;
		}
		
		for(uint8_t i=0;i<3;i++)
		{
		AngularVelCode[i]=CODEVEL(AngularVel[i]);
		}
}

void CrossInverse(double matrixB[9])
{
		for(uint8_t i=0;i<3;i++)
		{
		matrixB[i*4]=1/matrixB[i*4];
		}
}

void CrossMultiply(double matrixB[9],double matrixA[9],double matrix[9])
{
for(uint8_t i=0;i<3;i++)
	for(uint8_t j=0;j<3;j++)
	matrix[i*3+j]=-1*matrixA[i*3+j]*matrixB[4*i];

}

void tripeMultiplyOne(double matrixB[9],double matrixA[3],double matrix[3])
{
for(uint8_t i=0;i<3;i++)
	for(uint8_t j=0;j<3;j++)
	matrix[i]+=matrixB[i*3+j]*matrixA[j];

}
