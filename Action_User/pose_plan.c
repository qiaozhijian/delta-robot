#include "pose_plan.h"
#include "stdio.h"
/************variable  define**********/
const double gAxisAngle[3] = { 0.f,120.f/180.f*PI,240.f/180.f*PI };

/*通过三个电机的角度去更新关节坐标*/
void LinkCoordinateUpdate(const double ChainAngle[3],pcoordinate_t pLinkCoordinate)
{
	
	for (uint8_t i = 0; i < 3; i++)
	{
		pLinkCoordinate[i].x = (R - r + MASTERARM*arm_cos_f32(ChainAngle[i] / 180.f*PI))*arm_cos_f32(i*120.f / 180.f*PI);
		pLinkCoordinate[i].y = (R - r + MASTERARM*arm_cos_f32(ChainAngle[i] / 180.f*PI))*arm_sin_f32(i*120.f / 180.f*PI);
		pLinkCoordinate[i].z = -MASTERARM*arm_sin_f32(ChainAngle[i] / 180.f*PI);
	}
}
/*求向量的二范数（模）*/
double Norm(const double vector[3])
{
	return __sqrtf(vector[0]*100 * vector[0] + vector[1]*100 * vector[1] + vector[2]*100 * vector[2])/10;
}
/*求向量的叉乘*/
void VectorCross(const double vector1[3], const double vector2[3], double vectorResult[3])
{

	vectorResult[0] = vector1[1] * vector2[2] - vector1[2] * vector2[1];
	vectorResult[1] = vector1[2] * vector2[0] - vector1[0] * vector2[2];
	vectorResult[2] = vector1[0] * vector2[1] - vector1[1] * vector2[0];

}
/*求叉乘的单位向量*/
void UnitVector(const double vector1[3], const double vector2[3], double unitVector[3])
{
	double vector[3]={0};
	double length = 0.f;
  VectorCross(vector1, vector2,vector);
	length = Norm(vector);
	for (uint8_t i = 0; i < 3; i++)
	{
		unitVector[i] = vector[i] / length;
	}
}
/*求叉乘的单位向量*/
void UnitVector3(const double vector1[3], const double vector2[3], const double vector3[3],double unitVector[3])
{
	double vectorTemp1[3]={0};
	double vectorTemp2[3]={0};
	double length = 0.f;
	VectorCross(vector1, vector2,vectorTemp1);
	VectorCross(vectorTemp1, vector3,vectorTemp2);
	length = Norm(vectorTemp2);
	for (uint8_t i = 0; i < 3; i++)
	{
		unitVector[i] = vectorTemp2[i] / length;
	}
}
/*由两点求一个向量*/
void VectorSolveBY2Pouint8_t(const pcoordinate_t pLinkCoordinate1,const pcoordinate_t pLinkCoordinate2, double vector[3])
{
	vector[0] = pLinkCoordinate2->x - pLinkCoordinate1->x;
	vector[1] = pLinkCoordinate2->y - pLinkCoordinate1->y;
	vector[2] = pLinkCoordinate2->z - pLinkCoordinate1->z;
}
/*更新中心坐标*/
void BaseCenterUpdate(const double ChainAngle[3],pcoordinate_t pBaseCenter_t)
{
	coordinate_t LinkCoordinate[3]={0};
	LinkCoordinateUpdate(ChainAngle,LinkCoordinate);
	double H = 0.f;
	double S = 0.f;

	double vectorE1E2[3] = { 0.f };
	double vectorE2E3[3] = { 0.f };
	double vectorE3E1[3] = { 0.f };
	double vectorGF[3] = { 0.f };
	double vectorFP[3] = { 0.f };

	double LenE1E2 = 0.f;
	double LenE2E3 = 0.f;
	double LenE3E1 = 0.f;
	double LenFE = 0.f;
	double LenGF = 0.f;
	double LenFP = 0.f;

	VectorSolveBY2Pouint8_t(LinkCoordinate, LinkCoordinate + 1, vectorE1E2);
	VectorSolveBY2Pouint8_t(LinkCoordinate + 1, LinkCoordinate + 2, vectorE2E3);
	VectorSolveBY2Pouint8_t(LinkCoordinate + 2, LinkCoordinate, vectorE3E1);
	
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
	/*此处FP向量的取反在后续过程完成*/
	UnitVector(vectorE1E2, vectorE2E3, vectorFP);

	pBaseCenter_t->x = (LinkCoordinate->x + (LinkCoordinate + 1)->x)*0.5f + LenGF*vectorGF[0] - vectorFP[0] * LenFP;
	pBaseCenter_t->y = (LinkCoordinate->y + (LinkCoordinate + 1)->y)*0.5f + LenGF*vectorGF[1] - vectorFP[1] * LenFP;
	pBaseCenter_t->z = (LinkCoordinate->z + (LinkCoordinate + 1)->z)*0.5f + LenGF*vectorGF[2] - vectorFP[2] * LenFP;

}
/*通过坐标反更新电机角度*/
void ChainUpdateByBase(const coordinate_t BaseCenter_t, double ChainAngle[3])
{
	double paraR[3] = { 0.f };
	double paraS[3] = { 0.f };
	double paraT[3] = { 0.f };

	for (uint8_t i = 0; i < 3; i++)
	{
		paraR[i] = -2 * MASTERARM *(arm_cos_f32(gAxisAngle[i])*((r - R)*arm_cos_f32(gAxisAngle[i]) + BaseCenter_t.x) + \
		 arm_sin_f32(gAxisAngle[i])*(BaseCenter_t.y + (r - R)*arm_sin_f32(gAxisAngle[i])));

		paraS[i] = 2 * MASTERARM*BaseCenter_t.z;

		paraT[i] = pow((BaseCenter_t.x + (r - R)*arm_cos_f32(gAxisAngle[i])), 2) + \
			pow((BaseCenter_t.y + (r - R)*arm_sin_f32(gAxisAngle[i])), 2) + pow(BaseCenter_t.z, 2) + \
			pow(MASTERARM, 2) - pow(SLAVEARM, 2);
	}

	for (uint8_t i = 0; i < 3; i++)
	{
		/*二次项系数为0时*/
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
		if (ChainAngle[i] > 0.66f*180.f)
			ChainAngle[i] = 0.66f*180.f;
		if (ChainAngle[i] < 180.f / 6.f)
			ChainAngle[i] = 180.f / 6.f;
	}
}
/*通过电机角速度更新坐标速度*/
//void VelocityUpdate(const coordinate_t BaseCenter_t,const double ChainAngle[3],const double AngularVel[3],double Velocity[3])
//{
//	  double ChainRad[3]={ChainAngle[0]/180.f*PI,ChainAngle[1]/180.f*PI,ChainAngle[2]/180.f*PI};
//	  arm_matrix_instance_f32 jacobiA;
//		arm_matrix_instance_f32 jacobiAInverse;
//		arm_matrix_instance_f32 jacobiB;
//		arm_matrix_instance_f32 Jacobi;
//		arm_matrix_instance_f32 negativeMat;
//		arm_matrix_instance_f32 velocity;
//		arm_matrix_instance_f32 angularVel;
//	
//		double jacobiAArray[9]={0};
//		for(uint8_t i=0;i<3;i++)
//		{
//		jacobiAArray[i*3]=BaseCenter_t.x-arm_cos_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
//		jacobiAArray[i*3+1]=BaseCenter_t.y-arm_sin_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
//		jacobiAArray[i*3+2]=BaseCenter_t.z+MASTERARM*arm_sin_f32(ChainRad[i]);
//		}
//		double jacobiAInverseArray[9]={0};
//		
//		double jacobiBArray[9]={0};
//		for(uint8_t i=0;i<3;i++)
//		{
//		jacobiBArray[i*4]=MASTERARM*arm_sin_f32(ChainRad[i])*(BaseCenter_t.x*arm_cos_f32(gAxisAngle[i])+BaseCenter_t.y*arm_sin_f32(gAxisAngle[i])/
//			-R+r)+BaseCenter_t.z*MASTERARM*arm_cos_f32(ChainRad[i]);
//		}
//		double JacobiArray[9]={0};
//		double negativeMatArray[9]={
//		-1,0,0,
//		0,-1,0,
//		0,0,-1
//		};
//		double velocityArray[3]={0};
//		double angularVelArray[3]={AngularVel[0]/180.f*PI,AngularVel[1]/180.f*PI,AngularVel[2]/180.f*PI};
//		
//		arm_mat_init_f32(&Jacobi,3,3,JacobiArray);
//		arm_mat_init_f32(&jacobiA,3,3,jacobiAArray);
//		arm_mat_init_f32(&jacobiAInverse,3,3,jacobiAInverseArray);
//		arm_mat_init_f32(&jacobiB,3,3,jacobiBArray);
//		arm_mat_init_f32(&negativeMat,3,3,negativeMatArray);
//		arm_mat_init_f32(&velocity,3,1,velocityArray);
//		arm_mat_init_f32(&angularVel,3,1,angularVelArray);		
//		
//		arm_mat_inverse_f32(&jacobiA,&jacobiAInverse);
//	  arm_mat_mult_f32(&jacobiAInverse,&jacobiB,&Jacobi);
//		arm_mat_mult_f32(&negativeMat,&Jacobi,&Jacobi);
//		arm_mat_mult_f32(&Jacobi,&angularVel,&velocity);
//		
//		for(uint8_t i=0;i<3;i++)
//		{
//		Velocity[i]=*(velocity.pData+i);
//		}
//		
//}
/*通过坐标速度更新电机角速度*/
//void AngularVelUpdate(const coordinate_t BaseCenter_t,const double ChainAngle[3],const double Velocity[3],double AngularVel[3])
//{
//	  double ChainRad[3]={ChainAngle[0]/180.f*PI,ChainAngle[1]/180.f*PI,ChainAngle[2]/180.f*PI};
//	  arm_matrix_instance_f32 jacobiA;
//		arm_matrix_instance_f32 jacobiB;
//		arm_matrix_instance_f32 jacobiBInverse;
//		arm_matrix_instance_f32 Jacobi;
//		arm_matrix_instance_f32 negativeMat;
//		arm_matrix_instance_f32 velocity;
//		arm_matrix_instance_f32 angularVel;
//		
//		double jacobiAArray[9]={0};
//		for(uint8_t i=0;i<3;i++)
//		{
//		jacobiAArray[i*3]=BaseCenter_t.x-arm_cos_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
//		jacobiAArray[i*3+1]=BaseCenter_t.y-arm_sin_f32(gAxisAngle[i])*(R-r+MASTERARM*arm_cos_f32(ChainRad[i]));
//		jacobiAArray[i*3+2]=BaseCenter_t.z+MASTERARM*arm_sin_f32(ChainRad[i]);
//		}
//		double jacobiBInverseArray[9]={0};
//		
//		double jacobiBArray[9]={0};
//		for(uint8_t i=0;i<3;i++)
//		{
//		jacobiBArray[i*4]=MASTERARM*arm_sin_f32(ChainRad[i])*(BaseCenter_t.x*arm_cos_f32(gAxisAngle[i])+BaseCenter_t.y*arm_sin_f32(gAxisAngle[i])/
//			-R+r)+BaseCenter_t.z*MASTERARM*arm_cos_f32(ChainRad[i]);
//		}
//		double JacobiArray[9]={0};
//		double negativeMatArray[9]={
//		-1,0,0,
//		0,-1,0,
//		0,0,-1
//		};
//		double velocityArray[3]={Velocity[0],Velocity[1],Velocity[2]};
//		double angularVelArray[3]={0};

//		arm_mat_init_f32(&Jacobi,3,3,JacobiArray);
//		arm_mat_init_f32(&jacobiA,3,3,jacobiAArray);
//		arm_mat_init_f32(&jacobiBInverse,3,3,jacobiBInverseArray);
//		arm_mat_init_f32(&jacobiB,3,3,jacobiBArray);
//		arm_mat_init_f32(&negativeMat,3,3,negativeMatArray);
//		arm_mat_init_f32(&velocity,3,1,velocityArray);
//		arm_mat_init_f32(&angularVel,3,1,angularVelArray);		
//		
//		arm_mat_inverse_f32(&jacobiB,&jacobiBInverse);
//	  arm_mat_mult_f32(&jacobiBInverse,&jacobiA,&Jacobi);
//		arm_mat_mult_f32(&negativeMat,&Jacobi,&Jacobi);
//		arm_mat_mult_f32(&Jacobi,&velocity,&angularVel);
//		
//		for(uint8_t i=0;i<3;i++)
//		{
//		AngularVel[i]=*(angularVel.pData+i);
//		AngularVel[i]=AngularVel[i]/PI*180.f;
//		}
//}

double getK(coordinate_t startPoint,coordinate_t endPoint)
{
	double k=(startPoint.y-endPoint.y)/(startPoint.x-endPoint.x);
	return k;
}



