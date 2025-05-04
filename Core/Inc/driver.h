/*
 * l298n.h
 *
 *  Created on: Mar 23, 2025
 *      Author: zhang
 */

#ifndef INC_L298N_H_
#define INC_L298N_H_

#include "main.h"

void quiescent(void);
static void apply_deadzone(int raw, int *processed);
static int speed_to_pwm(float speed) ;

void update_motion_control(int *input_array);

#endif /* INC_L298N_H_ */
