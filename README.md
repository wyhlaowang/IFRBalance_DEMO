# 控制逻辑

- 声明观测量
```
//左右编码器的脉冲计数
int Encoder_Left,Encoder_Right;
//电机PWM变量
int Moto1,Moto2;
//平衡倾角,平衡陀螺仪
float Angle_Balance,Gyro_Balance;
```
- 初始化外设
```
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
```
- 进入主函数
```
int main(void)
{
	init();
	while(1);
}
```
- 外部中断控制
```
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
	Xianfu_Pwm();
	if(Turn_Off(Angle_Balance)==0)
		Set_Pwm(Moto1,Moto2);	
	return 0;		
}
```
- 直立环PID控制器
```
int balance(float Angle,float Gyro)
{
	float Bias,kp=600,kd=2.4;
	int balance;
	Bias=Angle-ZHONGZHI;
	balance=kp*Bias+Gyro*kd;
	return balance;
}
```
- 速度环PID控制器
```
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
```
