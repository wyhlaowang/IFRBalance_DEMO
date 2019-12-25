#include "control.h"	
int Balance_Pwm,Velocity_Pwm,Turn_Pwm,Flag_Target;


int EXTI9_5_IRQHandler(void)
{
	//���LINE5�ϵ��жϱ�־λ
	EXTI->PR=1<<5;
	Flag_Target=!Flag_Target;
	//��ȡ�۲�ֵ��5ms��
	if(Flag_Target==1)
	{
		Get_Angle();
		return 0;	
	}
	Encoder_Left=-Read_Encoder(2);
	Encoder_Right=Read_Encoder(4);
	Get_Angle();
	//ֱ����
	Balance_Pwm=balance(Angle_Balance,Gyro_Balance);
	//�ٶȻ�
	Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);
	//����PWM
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
	//һ�׵�ͨ�˲���
	Encoder*=0.8;
	Encoder+=Encoder_Least*0.2;
	Encoder_Integral+=Encoder;
	Velocity=Encoder*kp+Encoder_Integral*ki;
	return Velocity;
}































/*
int EXTI9_5_IRQHandler(void) 
{    
		EXTI->PR = 1<<5;                                                      //���LINE5�ϵ��жϱ�־λ   
		Flag_Target = !Flag_Target;
		if(Flag_Target == 1)                                                  //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ�����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		{
			Get_Angle();                                                      	//===������̬	
			return 0;	                                               
		}                                                                   	//10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������	
		Encoder_Left = -Read_Encoder(2);                                      //===��ȡ��������ֵ����֤�������һ��
		Encoder_Right = Read_Encoder(4);                                      //===��ȡ��������ֵ
		Get_Angle();                                                        	//===������̬	                                                            
		Balance_Pwm = balance(Angle_Balance, Gyro_Balance);                   //===ƽ��PID����	
		Velocity_Pwm = velocity(Encoder_Left, Encoder_Right);                 //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
		Moto1 = Balance_Pwm-Velocity_Pwm;                                     //===�������ֵ������PWM
		Moto2 = Balance_Pwm-Velocity_Pwm;                                     //===�������ֵ������PWM
		Xianfu_Pwm();                                                       	//===PWM�޷�
		if(Turn_Off(Angle_Balance) == 0)                                      //===����������쳣
			Set_Pwm(Moto1, Moto2);                                             	//===��ֵ��PWM�Ĵ���    	
		return 0;	 	
} 
int balance(float Angle,float Gyro)
{  
   float Bias, kp = 600, kd = 2.4;
	 int balance;
	 Bias = Angle-ZHONGZHI;       	//===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balance = kp*Bias + Gyro*kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder,Encoder_Integral,kp=120,ki=0.6;
		Encoder_Least =(Encoder_Left+Encoder_Right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
		Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
		Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
		Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===�ٶȿ���	
		return Velocity;
}
*/

/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ���������PWM������PWM
����  ֵ����
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
�������ܣ�����PWM��ֵ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM������7200 ������6900
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
}

/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������Ǻ͵�ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off(float angle)
{
	    u8 temp;
			if(angle<-40||angle>40)
			{	                                                 //===��Ǵ���40�ȹرյ��
      temp=1;                                            //===Flag_Stop��1�رյ��
			AIN1=0;                                            //===���������������¶ȹ���ʱ�رյ��
			AIN2=0;
			BIN1=0;
			BIN2=0;
      }
			else
      temp=0;
      return temp;			
}
	
/**************************************************************************
�������ܣ���ȡ�Ƕ�
����  ֵ����
**************************************************************************/
void Get_Angle(void)
{ 
		Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             //===����ƽ�����
		Gyro_Balance=gyro[1];            //===����ƽ����ٶ�
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}



