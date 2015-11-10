/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
*/
/* Control.c file
编写者：小马  (Camel) 、祥、Nieyong
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.PID参数初始化
2.控制函数

------------------------------------
*/
#include <stm32f10x.h>
#include "control.h"
#include "moto.h"
#include "math.h"
#include "sys_fun.h"
#include "mpu6050.h"
#include "imu.h"
#include "extern_variable.h"
#include "led.h"
#include "stmflash.h"
#include "ReceiveData.h"
#include "DMP.h"
#include "Battery.h"
#include "stdio.h"
#include "BT.h"
#include "Altitude.h"
#include "SysConfig.h"

extern uint32_t micros(void);

uint8_t offLandFlag=0;


volatile unsigned char motorLock=1;

int16_t Motor[4]={0};   //定义电机PWM数组，分别对应M1-M4
float rollSp =0,pitchSp =0;		//根据动力分配重新计算得到的期望roll pitch
float Thro=0,Roll=0,Pitch=0,Yaw=0;


//----PID结构体实例化----
PID_Typedef pitch_angle_PID;	//pitch角度环的PID
PID_Typedef pitch_rate_PID;		//pitch角速率环的PID

PID_Typedef roll_angle_PID;   //roll角度环的PID
PID_Typedef roll_rate_PID;    //roll角速率环的PID

PID_Typedef yaw_angle_PID;    //yaw角度环的PID 
PID_Typedef yaw_rate_PID;     //yaw角速率环的PID

PID_Typedef	alt_PID;
PID_Typedef alt_vel_PID;

float gyroxGloble = 0;
float gyroyGloble = 0;


S_FLOAT_XYZ DIF_ACC;		//实际去期望相差的加速度
S_FLOAT_XYZ EXP_ANGLE;	//期望角度	
S_FLOAT_XYZ DIF_ANGLE;	//实际与期望相差的角度	

uint32_t ctrlPrd=0;
uint8_t headFreeMode=0;
float headHold=0;

//#define CONSTRAIN(x,min,max)  {if(x<min) x=min; if(x>max) x=max;}

//-----------位置式PID-----------
static void PID_Postion_Cal(PID_Typedef * PID,float target,float measure,int32_t dertT)
{
 float termI=0;
 float dt= dertT/1000000.0;
	//-----------位置式PID-----------
	//误差=期望值-测量值
	PID->Error=target-measure;
	
	PID->Deriv= (PID->Error-PID->PreError)/dt;
	
	PID->Output=(PID->P * PID->Error) + (PID->I * PID->Integ) + (PID->D * PID->Deriv);    //PID:比例环节+积分环节+微分环节
	
	PID->PreError=PID->Error;
	//仅用于角度环和角速度环的

	if(FLY_ENABLE)
	{
			if(fabs(PID->Output) < Thro )		              //比油门还大时不积分
			{
				termI=(PID->Integ) + (PID->Error) * dt;     //积分环节
				if(termI > - PID->iLimit && termI < PID->iLimit && PID->Output > - PID->iLimit && PID->Output < PID->iLimit)       //在-300~300时才进行积分环节
						PID->Integ=termI;
			}
	}
	else
			PID->Integ= 0;
}

void SetHeadFree(uint8_t on)
{
	if(on==1)
	{
		headHold=imu.yaw;
		headFreeMode=1;
	}
	else
		headFreeMode=0;
}


	
//run after get rc cmd
void CtrlAttiAng(void)
{
		static float yawHold=0;
		static uint32_t tPrev=0;
		float yawRateTarget=0;
		float angTarget[3]={0};
		float dt=0,t=0;
		t=micros();
		dt=(tPrev>0)?(t-tPrev):0;
		tPrev=t;
    altCtrlMode=MANUAL;
		angTarget[ROLL]=(float)(RC_DATA.ROOL);
		angTarget[PITCH]=(float)(RC_DATA.PITCH);
 
		PID_Postion_Cal(&pitch_angle_PID,angTarget[PITCH],imu.pitch,dt);
		PID_Postion_Cal(&roll_angle_PID,angTarget[ROLL],imu.roll,dt);	 
}




//run in 200Hz or 400Hz loop 
void CtrlAttiRate(void)
{
 	float yawRateTarget=0;
	static uint32_t tPrev=0; 

	float dt=0,t=0;
	t=micros();
	dt=(tPrev>0)?(t-tPrev):0;
	tPrev=t;
		
		yawRateTarget=-(float)RC_DATA.YAW;
		//注意，原来的pid参数，对应的是 ad值,故转之
		#ifdef IMU_SW
		PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,imu.gyro[PITCH]*180.0f/M_PI_F,dt);	
		PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,imu.gyro[ROLL]*180.0f/M_PI_F,dt);//gyroxGloble
		PID_Postion_Cal(&yaw_rate_PID,yawRateTarget,imu.gyro[YAW]*180.0f/M_PI_F,dt);//DMP_DATA.GYROz
	  #else
		//原参数对应于 DMP的直接输出gyro , 是deg.  且原DMP之后的处理运算是错误的
		PID_Postion_Cal(&pitch_rate_PID,pitch_angle_PID.Output,imu.gyro[PITCH]*DMP_GYRO_SCALE,0);	
		PID_Postion_Cal(&roll_rate_PID,roll_angle_PID.Output,imu.gyro[ROLL]*DMP_GYRO_SCALE,0);//gyroxGloble
		PID_Postion_Cal(&yaw_rate_PID,yawRateTarget,imu.gyro[YAW]*DMP_GYRO_SCALE,0);          //DMP_DATA.GYROz
		#endif
		
		Pitch = pitch_rate_PID.Output;
    Roll  = roll_rate_PID.Output;
    Yaw   = yaw_rate_PID.Output; 
}

//----------to be tested--//
//cut deadband, move linear
float dbScaleLinear(float x, float x_end, float deadband)
{
	if (x > deadband) {
		return (x - deadband) / (x_end - deadband);

	} else if (x < -deadband) {
		return (x + deadband) / (x_end - deadband);

	} else {
		return 0.0f;
	}
}

//alti ctrl , output thrustZSp ,  ned frame , thrust force on z axis. 



float thrInit;

#define ALT_FEED_FORWARD  		0.5f
#define THR_MAX								1.0f		//max thrust
#define TILT_MAX 					(Angle_Max * M_PI_F / 180.0 )
const float ALT_CTRL_Z_DB = 0.1f;	//
float spZMoveRate;

uint8_t altCtrlMode;					//normal=0  CLIMB rate, normal .  tobe tested
float hoverThrust=0;
uint8_t zIntReset=1;	//integral reset at first . when change manual mode to climb rate mode
float thrustZInt=0, thrustZSp=0;
float thrustXYSp[2]={0,0};	//roll pitch
uint8_t recAltFlag=0;
float holdAlt=0;
uint8_t satZ=0,satXY=0;	//是否过饱和


#define ALT_LIMIT							2.0f		//限高 3.5
uint8_t isAltLimit=0;
float altLand;

 

#define ANG_COR_COEF 50.0f



//函数名：CtrlMotor()
//输入：无
//输出: 4个电机的PWM输出
//描述：输出PWM，控制电机，本函数会被主循环中100Hz循环调用
void CtrlMotor(void)
{
		float  cosTilt = imu.accb[2] / ONE_G;
			DIF_ACC.Z =  imu.accb[2] - ONE_G;
			Thro = RC_DATA.THROTTLE;
		
		//将输出值融合到四个电机 
		Motor[0] = (int16_t)(Thro - Pitch + Roll + Yaw );    //M3  
		Motor[1] = (int16_t)(Thro - Pitch - Roll - Yaw );    //M1
		Motor[2] = (int16_t)(Thro + Pitch - Roll + Yaw );    //M4 
		Motor[3] = (int16_t)(Thro + Pitch + Roll - Yaw );    //M2
	
   	if((FLY_ENABLE!=0)) {
			MotorPwmFlash(Motor[0],Motor[1],Motor[2],Motor[3]);

		}			
		else                  
			MotorPwmFlash(0,0,0,0); 
			
}
