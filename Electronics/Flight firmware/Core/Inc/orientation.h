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
    double gyroVec[3];
    double accBodyVec[3];
    double accWorldVec[3];
    double vertical[3];
    Quaternion gyroQuat;
    Quaternion accQuat;
    double eulerZYX[3];
    Quaternion horQuat;
} Orientation;

void cross_product(double a[3], double b[3], double output[3]);
void vector_sum(double a[3], double b[3], double output[3]);

void orientation_init(Orientation *ori);

void orientation_setGyro(Orientation *ori, double gyro[3]);
void orientation_setAcc(Orientation *ori, double acc[3]);
void orientation_update(Orientation *ori, double dt);

#endif /* INC_DRIVERS_ORIENTATION_H_ */
