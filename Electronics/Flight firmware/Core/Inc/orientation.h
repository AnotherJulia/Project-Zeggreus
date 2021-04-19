/*
 * orientation.h
 *
 *  Created on: Mar 19, 2021
 *      Author: elvin
 */

#ifndef INC_DRIVERS_ORIENTATION_H_
#define INC_DRIVERS_ORIENTATION_H_

#include "Quaternion.h"

typedef struct {
    Quaternion orientationQuat;
    Quaternion orientationQuatConj;
    Quaternion incrementalRotation;
    float gyroVec[3];
    float accBodyVec[3];
    float accWorldVec[3];
    float vertical[3];
    Quaternion gyroQuat;
    Quaternion accQuat;
    float eulerZYX[3];
    Quaternion horQuat;
} Orientation;

void cross_product(float a[3], float b[3], float output[3]);
void vector_sum(float a[3], float b[3], float output[3]);

void orientation_init(Orientation *ori);

void orientation_setGyro(Orientation *ori, float gyro[3]);
void orientation_setAcc(Orientation *ori, float acc[3]);
void orientation_update(Orientation *ori, float dt);

#endif /* INC_DRIVERS_ORIENTATION_H_ */
