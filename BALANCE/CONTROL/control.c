#include "control.h"	
int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Flag_Target;


int EXTI9_5_IRQHandler(void)
{
	//清除LINE5上的中断标志位
	EXTI->PR=1<<5;
	Flag_Target=!Flag_Target;
	//获取观测值（5ms）
	if(Flag_Target==1)
	{
		Get_Angle();
		return 0;	
	}
	Encoder_Left=-Read_Encoder(2);
	Encoder_Right=Read_Encoder(4);
	Get_Angle();
	//直立环
	Balance_Pwm=balance(Angle_Balance,Gyro_Balance);
	//速度环
	Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);
	//计算PWM
	Moto1=Balance_Pwm-Velocity_Pwm;
	Moto2=Balance_Pwm-Velocity_Pwm;
	//Xianfu_Pwm();
	if(Turn_Off(Angle_Balance)==0)
		Set_Pwm(Moto1,Moto2);	
	return 0;		
}


int balance(float Angle,float Gyro)
{
	float Bias,kp=600,kd=2.4;
	int balance;
	Bias=Angle-ZHONGZHI;
	balance=kp*Bias+Gyro*kd;
	return balance;
}


int velocity(int encoder_left,int encoder_right)
{
	static float Velocity,Encoder_Least,Encoder,Encoder_Integral,kp=120,ki=0.6;
	Encoder_Least=(Encoder_Left+Encoder_Right)-0;
	//一阶低通滤波器
	Encoder*=0.8;
	Encoder+=Encoder_Least*0.2;
	Encoder_Integral+=Encoder;
	Velocity=Encoder*kp+Encoder_Integral*ki;
	return Velocity;
}































/*
int EXTI9_5_IRQHandler(void) 
{    
		EXTI->PR = 1<<5;                                                      //清除LINE5上的中断标志位   
		Flag_Target = !Flag_Target;
		if(Flag_Target == 1)                                                  //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		{
			Get_Angle();                                                      	//===更新姿态	
			return 0;	                                               
		}                                                                   	//10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据	
		Encoder_Left = -Read_Encoder(2);                                      //===读取编码器的值，保证输出极性一致
		Encoder_Right = Read_Encoder(4);                                      //===读取编码器的值
		Get_Angle();                                                        	//===更新姿态	                                                            
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);                   //===平衡PID控制	
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);                 //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
		Moto1 = Balance_Pwm-Velocity_Pwm;                                     //===计算左轮电机最终PWM
		Moto2 = Balance_Pwm-Velocity_Pwm;                                     //===计算右轮电机最终PWM
		Xianfu_Pwm();                                                       	//===PWM限幅
		if(Turn_Off(Angle_Balance) == 0)                                      //===如果不存在异常
			Set_Pwm(Moto1, Moto2);                                             	//===赋值给PWM寄存器    	
		return 0;	 	
} 
int balance(float Angle,float Gyro)
{  
   float Bias, kp = 600, kd = 2.4;
	 int balance;
	 Bias = Angle-ZHONGZHI;       	//===求出平衡的角度中值 和机械相关
	 balance = kp*Bias + Gyro*kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Encoder_Integral,kp=120,ki=0.6;
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
		return Velocity;
}
*/

/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
		if(moto1<0)			AIN2=1,			AIN1=0;
		else 	          AIN2=0,			AIN1=1;
		PWMA=myabs(moto1);
		if(moto2<0)	BIN1=0,			BIN2=1;
		else        BIN1=1,			BIN2=0;
		PWMB=myabs(moto2);	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle)
{
	    u8 temp;
			if(angle<-40||angle>40)
			{	                                                 //===倾角大于40度关闭电机
      temp=1;                                            //===Flag_Stop置1关闭电机
			AIN1=0;                                            //===可自行增加主板温度过高时关闭电机
			AIN2=0;
			BIN1=0;
			BIN2=0;
      }
			else
      temp=0;
      return temp;			
}
	
/**************************************************************************
函数功能：获取角度
返回  值：无
**************************************************************************/
void Get_Angle(void)
{ 
		Read_DMP();                      //===读取加速度、角速度、倾角
		Angle_Balance=Pitch;             //===更新平衡倾角
		Gyro_Balance=gyro[1];            //===更新平衡角速度
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}



