/*
 * l298n.c
 *
 *  Created on: Mar 23, 2025
 *      Author: zhang
 */
#include "driver.h"
#include "tim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
// 常量定义
#define CENTER 2048
#define CENTER_DEADZONE 200
#define EDGE_DEADZONE 20
#define PWM_MIN 595 // 电机启动值 (1%)
#define PWM_MAX 665 // 最大PWM值 (100%)
#define PWM_RANGE (PWM_MAX - PWM_MIN)

float throttle_percent=0; // 油门百分比

/*
 *电机控制结构体
 * 用于设置PWM和方向控制
 */
typedef struct
{
	GPIO_TypeDef *in1_port;
	uint16_t in1_pin;
	GPIO_TypeDef *in2_port;
	uint16_t in2_pin;
	TIM_HandleTypeDef *pwm_tim;
	uint32_t pwm_in1_channel;
	uint32_t pwm_in2_channel;
} MotorController;

// 电机实例化
MotorController left_motor = {.in1_port = GPIOA, .in1_pin = GPIO_PIN_0, .in2_port = GPIOA, .in2_pin = GPIO_PIN_1, .pwm_tim = &htim2, .pwm_in1_channel = TIM_CHANNEL_1, .pwm_in2_channel = TIM_CHANNEL_2};

MotorController right_motor = {.in1_port = GPIOA, .in1_pin = GPIO_PIN_2, .in2_port = GPIOA, .in2_pin = GPIO_PIN_3, .pwm_tim = &htim2, .pwm_in1_channel = TIM_CHANNEL_3, .pwm_in2_channel = TIM_CHANNEL_4};

/*
 *	应用死区处理
 * @param raw 原始输入值
 * @param processed 处理后的值
 */
static void apply_deadzone(int raw, int *processed)
{
	*processed = raw;

	// 中心死区
	if (abs(raw - CENTER) <= CENTER_DEADZONE)
	{
		*processed = CENTER;
	}
	// 边缘死区
	else if (raw <= EDGE_DEADZONE)
	{
		*processed = 0;
	}
	else if (raw >= 4096 - EDGE_DEADZONE)
	{
		*processed = 4096;
	}
}

/* 速度转PWM值
 * @param speed 速度值 (-1.0到1.0)
 * @return PWM值 (0到100%)
 */
static int speed_to_pwm(float speed)
{
	float abs_speed = fabsf(speed);
	if (abs_speed <= 0)
		return 0;
	return (int)round(PWM_MIN + ((abs_speed - 1) / 99) * PWM_RANGE);
}

/* 电机控制函数
 *	@param motor 电机控制结构体
 *	@param speed 速度值 (-1.0到1.0)
 *   @return 无
 */
static void motor_control(MotorController *motor, float speed)
{
	// 计算PWM值
	int pwm = speed_to_pwm(speed);

	if (throttle_percent > 0)
	{
		if (speed >= 0)
		{
			/* code */
			__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_in1_channel, 0);
			__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_in2_channel, pwm);
		}
		else if (speed <= 0)
		{
			/* code */
			__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_in1_channel, pwm);
			__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_in2_channel, 0);
		}
	}
	else
	{
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_in1_channel, 0);
		__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_in2_channel, 0);
	}
}

// 主控制函数
void update_motion_control(int *input_array)
{
	int x_processed, y_processed;  // 处理后的输入值
	float x_norm, y_norm;		   // 归一化后的输入值
	float left_speed, right_speed; // 左右电机速度

	// 1. 应用死区处理
	apply_deadzone(input_array[0], &x_processed);
	apply_deadzone(input_array[1], &y_processed);

	// 2. 归一化处理 (-1.0到1.0范围)
	x_norm = (x_processed - CENTER) / (float)(CENTER - EDGE_DEADZONE);
	y_norm = (y_processed - CENTER) / (float)(CENTER - EDGE_DEADZONE);

	// 限制在[-1, 1]范围内
	x_norm = fmaxf(-1.0f, fminf(1.0f, x_norm));
	y_norm = fmaxf(-1.0f, fminf(1.0f, y_norm));

	// 3. 计算油门百分比 (0-100%)
	if (input_array[2] > CENTER && input_array[2] < (4096 - EDGE_DEADZONE))
	{
		throttle_percent = ((input_array[2] - 2048) / (float)2048) * 99 + 1; // 范围 在1到100之间
	}
	else if (input_array[2] >= (4096 - EDGE_DEADZONE))
	{
		throttle_percent = 100;
	}
	else if (input_array[2] <= CENTER)
	{
		throttle_percent = 0;
	}
	
	// 限制在[0, 100]范围内
	throttle_percent = fmaxf(0.0f, fminf(100.0f, throttle_percent));

	// 4. 差速驱动计算
	left_speed = (y_norm + x_norm) * throttle_percent / 100.0f;	 // 左电机速度
	right_speed = (y_norm - x_norm) * throttle_percent / 100.0f; // 右电机速度

	// 限幅处理 (确保在-100%到100%之间)
	left_speed = fmaxf(-1.0f, fminf(1.0f, left_speed));
	right_speed = fmaxf(-1.0f, fminf(1.0f, right_speed));

	// 5. 控制电机
	motor_control(&left_motor, left_speed);
	motor_control(&right_motor, right_speed);
}

// 停止电机
void quiescent(void)
{
	motor_control(&left_motor, 0);
	motor_control(&right_motor, 0);
}
