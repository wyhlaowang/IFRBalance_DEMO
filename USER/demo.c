#include "sys.h"
//左右编码器的脉冲计数
int Encoder_Left,Encoder_Right;
//电机PWM变量
int Moto1,Moto2;
//平衡倾角,平衡陀螺仪
float Angle_Balance,Gyro_Balance;


void init(void)
{
	//=====系统时钟设置
	Stm32_Clock_Init(9);
	//=====延时初始化
	delay_init(72);
	//=====初始化串口1
	uart_init(72,128000);
	//=====初始化PWM,10KHZ，用于驱动电机
	MiniBalance_PWM_Init(7199,0);
	//=====初始化编码器2
	Encoder_Init_TIM2();
	//=====初始化编码器4
	Encoder_Init_TIM4();
	//=====模拟IIC初始化
	IIC_Init();
	//=====MPU6050初始化
	MPU6050_initialize();
	//=====初始化DMP
	DMP_Init();
	//=====MPU60505ms定时中断初始化
	EXTI_Init();		
}

int main(void)
{
	init();
	while(1);
}
