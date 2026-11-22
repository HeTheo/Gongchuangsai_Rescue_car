#ifndef __SERVO_H
#define __SERVO_H

#include "sys.h"

/* ---------------- ͨ������ ---------------- */
#define SERVO_CH1   1        /* PE5 = TIM9_CH1 */
#define SERVO_CH2   2        /* PE6 = TIM9_CH2 */

/* ---------------- ������䣨�����ʵ�ʽ��߸ģ� ---------------- */
#define SERVO_GRIPPER   SERVO_CH1   /* ��צ */
#define SERVO_CAMERA    SERVO_CH2   /* ����ͷ */

/* ---------------- Ԥ��Ƕȣ��ϰ�궨������ ---------------- */
#define GRIPPER_UP_DEG       155
#define GRIPPER_OPEN_DEG     120
#define GRIPPER_CLOSE_DEG    87
#define CAM_UP_DEG           73
#define CAM_NEAR_DEG         55      /* �� ����������ǰ�� + ����Ŀ��Ⱥ */
#define CAM_DOWN_DEG         35

/* ---------------- �ӿ� ---------------- */
void Servo_Init(void);
void Servo_SetAngle(uint8_t ch, float angle);
void Servo_SetPulse(uint8_t ch, uint16_t us);

void Servo_Grab(void);
void Servo_Drop(void);
void Servo_Open(void);
void Servo_SetCam(uint8_t up_down);

#endif

