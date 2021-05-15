/*
 * servo.h
 *
 *  Created on: 18 Mar 2021
 *      Author: elvin
 */

#ifndef INC_DRIVERS_SERVO_H_
#define INC_DRIVERS_SERVO_H_

#include "stm32f4xx_hal.h"

typedef struct {
    uint8_t angle;
    TIM_HandleTypeDef *timer;
    volatile uint32_t *timerval;

    uint8_t enabled;
} Servo;


void servo_init(Servo *servo, TIM_HandleTypeDef *tim, volatile uint32_t *timerval);
void servo_writeangle(Servo *servo, uint8_t angle);
void servo_writemicros(Servo *servo, uint32_t pulse);
void servo_disable(Servo *servo);

#endif /* INC_DRIVERS_SERVO_H_ */
