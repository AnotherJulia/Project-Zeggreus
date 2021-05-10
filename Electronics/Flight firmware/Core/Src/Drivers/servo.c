#include "servo.h"

void servo_init(Servo *servo, TIM_HandleTypeDef *tim, volatile uint32_t *timerval) {
    servo->timer = tim;
    servo->timerval = timerval;
}


void servo_writeangle(Servo *servo, uint8_t angle) {
    // value between 0 and 180
    uint32_t newtimerval = 1000 + (angle * 1000 / 180);
    *servo->timerval = newtimerval;
}

void servo_writemicros(Servo *servo, uint32_t pulse) {
    *servo->timerval = pulse;
}

void servo_disable(Servo *servo) {
    *servo->timerval = 0;
}
