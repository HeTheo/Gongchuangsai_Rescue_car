#include "sys.h"
#include "delay.h"
#include "key.h"
#include "Timer.h"
#include "Serial.h"
#include "Gyro.h"
#include "PWM.h"
#include "Motor.h"
#include "Encoder.h"
#include "PID.h"
#include "Motion.h"
#include "Wireless.h"
#include "Comm.h"
#include "Servo.h"
#include "App.h"

float Target_LeftSpeed = 250;
float Target_RightSpeed = 250;
float BaseSpeed = 250;
float Target_Yaw = 0;
float Actual_LeftSpeed, Actual_RightSpeed;
float Left_Out, Right_Out;
void Back_Test(float dist);
float Yaw_actual = 0;
float Yaw_speed = 0;


int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	
	TIM5_Init();
	Serial_Init();
	Gyro_Init();
	Motor_Init();
	Encoder_Init();
	Motion_Init();
	Wireless_Init();
	Servo_Init();
	
	delay_ms(2000);
	Comm_Init();
	
//	Motion_SetTurnSpeed(20, 1);
//	Motion_SetTurnAngle(90);
//	Motion_SetMode(MOTION_TURN);
//	while (Motion_GetMode() != MOTION_STOP) delay_ms(10);
//	Motion_SetTurnSpeed(20, 1);
//	Motion_SetTurnAngle(90);
//	Motion_SetMode(MOTION_TURN);
//	while (Motion_GetMode() != MOTION_STOP) delay_ms(10);
		
	//Motion_SetMode(MOTION_TURN);

//	/* 1. 直行 1.5 秒 */
//	Motion_SetMode(MOTION_STRAIGHT);
//	delay_ms(2000);
//	Motion_SetMode(MOTION_STOP);
//	delay_ms(500);
//	
//	/* 2. 左转 90°（转到自动停） */
//	Motion_SetTurnSpeed(45, 1);
//	Motion_SetTurnAngle(90);                    // 90° 会自动停
//	Motion_SetMode(MOTION_TURN);
//	while (Motion_GetMode() != MOTION_STOP)     // 等它转完
//	delay_ms(10);
//	delay_ms(500);
	
	/* 3. 后退 1 秒 */
//	Motion_SetMode(MOTION_BACK);
//	delay_ms(8000);
//	Motion_SetMode(MOTION_STOP);
//	delay_ms(500);
//	
//	/* 4. 右转 90° */
//	Motion_SetTurnSpeed(20, -1);
//	Motion_SetTurnAngle(90);
//	Motion_SetMode(MOTION_TURN);
//	while (Motion_GetMode() != MOTION_STOP)
//		delay_ms(10);
//	delay_ms(500);
//	
//	/* 5. 模拟视觉追踪（目标在左 tx=100）2 秒 */
//	Motion_SetTrackTarget(100, 240);
//	Motion_SetMode(MOTION_VISUAL_TRACK);
//	delay_ms(2000);
//	Motion_SetMode(MOTION_STOP);
//	delay_ms(500);
//	
//	/* 6. 一直转 3 秒（模拟搜索，手动停） */
//	Motion_SetTurnSpeed(20, 1);
//	Motion_SetTurnAngle(0);                     // 0 = 一直转
//	Motion_SetMode(MOTION_TURN);
//	delay_ms(5000);
//	Motion_SetMode(MOTION_STOP);
//	delay_ms(500);
//	
//	/* 7. 舵机：抓一下 + 松一下 + 摄像头低头抬头 */
//	Servo_Grab();  delay_ms(500);
//	Servo_Open();  delay_ms(500);
//	Servo_SetCam(1); delay_ms(500);   // 低头
//	Servo_SetCam(0); delay_ms(500);   // 抬头

//	Servo_SetAngle(SERVO_GRIPPER, 60);
//	Servo_SetAngle(SERVO_CAMERA, 80);
	
	while(1)
	{
		
//		/* 1. 直行 1.5 秒 */
//		Motion_SetMode(MOTION_STRAIGHT);
//		delay_ms(1500);
//		Motion_SetMode(MOTION_STOP);
//		delay_ms(500);
//		
//		/* 3. 后退 1 秒 */
//		Motion_SetMode(MOTION_BACK);
//		delay_ms(1500);
//		Motion_SetMode(MOTION_STOP);
//		delay_ms(500);
		
		
//		Actual_LeftSpeed = Encoder_GetLeft();
//		Actual_RightSpeed = Encoder_GetRight();
//		Serial_Printf("%f,%f\r\n", Actual_LeftSpeed, Actual_RightSpeed);
//		
//		Yaw_actual = Yaw();
//		Yaw_speed = GyroZ();
//		Serial_Printf("%f,%f\r\n", Yaw_actual, Yaw_speed);

		Comm_Poll();                    // 把缓冲区的字节喂给解析器

		Comm_Frame_t f;
		if (Comm_GetFrame(&f))          // 有新帧才处理
		{
			Cmd_Dispatch(&f);           // 分发（下面写）
		}
		
		App_Poll();
	
		if (!Comm_Valid())              // 通信超时没收到帧 → 停
        {
            Motion_SetMode(MOTION_STOP);
        }
//		
	}
	
}

void TIM5_IRQHandler(void)
{
	static uint16_t Count;		//定义静态变量，默认初始化为0，函数结束后，仍保留值和存储空间
	
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //溢出中断
	{
		Count ++;
		g_MsTick++;              /* 全局毫秒时基 +1 */
		

		//中断1ms触发一次，计次分频至40ms读取一次编码器增量值
		if(Count >= 20)
		{
			Count = 0;
			Motion_Control();  // 中断里判断模式、算目标、驱动电机
			
		}
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //清除中断标志位
}




