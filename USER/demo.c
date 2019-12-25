#include "sys.h"
//���ұ��������������
int Encoder_Left,Encoder_Right;
//���PWM����
int Moto1,Moto2;
//ƽ�����,ƽ��������
float Angle_Balance,Gyro_Balance;


void init(void)
{
	//=====ϵͳʱ������
	Stm32_Clock_Init(9);
	//=====��ʱ��ʼ��
	delay_init(72);
	//=====��ʼ������1
	uart_init(72,128000);
	//=====��ʼ��PWM,10KHZ�������������
	MiniBalance_PWM_Init(7199,0);
	//=====��ʼ��������2
	Encoder_Init_TIM2();
	//=====��ʼ��������4
	Encoder_Init_TIM4();
	//=====ģ��IIC��ʼ��
	IIC_Init();
	//=====MPU6050��ʼ��
	MPU6050_initialize();
	//=====��ʼ��DMP
	DMP_Init();
	//=====MPU60505ms��ʱ�жϳ�ʼ��
	EXTI_Init();		
}

int main(void)
{
	init();
	while(1);
}
