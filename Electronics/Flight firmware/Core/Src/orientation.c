/*
 * orientation.c
 *
 *  Created on: Mar 19, 2021
 *      Author: elvin
 */


#include "orientation.h"
#include "constants.h"

void cross_product(float a[3], float b[3], float output[3]) {
    output[0] = a[1] * b[2] - a[2]*b[1];
    output[1] = a[2] * b[0] - a[0]*b[2];
    output[2] = a[0] * b[1] - a[1]*b[0];
}

void vector_sum(float a[3], float b[3], float output[3]) {
    output[0] = a[0] + b[0];
    output[1] = a[1] + b[1];
    output[2] = a[2] + b[2];
}

float vector_lenSquared(float a[3]) {
    return a[0]*a[0] + a[1]*a[1] + a[2]*a[2];
}

void orientation_init(Orientation *ori) {
    Quaternion_setIdentity(&ori->orientationQuat);
    Quaternion_setIdentity(&ori->accQuat);
    Quaternion_setIdentity(&ori->gyroQuat);

    // Point down (-z axis)
    ori->vertical[0] = 0;
    ori->vertical[1] = 0;
    ori->vertical[2] = 1;

    ori->gyroVec[0] = 0;
    ori->gyroVec[1] = 0;
    ori->gyroVec[2] = 0;
}

void orientation_setGyro(Orientation *ori, float gyro[3]) {
    ori->gyroVec[0] = gyro[0];
    ori->gyroVec[1] = gyro[1];
    ori->gyroVec[2] = gyro[2];
}

void orientation_setAcc(Orientation *ori, float acc[3]) {
    ori->accBodyVec[0] = acc[0];
    ori->accBodyVec[1] = acc[1];
    ori->accBodyVec[2] = acc[2];
}

// based on https://github.com/daPhoosa/SimpleIMU-6/blob/master/SimpleIMU-6.ino
void orientation_update(Orientation *ori, float dt) {

    //Quaternion_set(0,ori->gyroVec[0],ori->gyroVec[1],ori->gyroVec[2],&ori->gyroQuat);
    //Quaternion_set(0,ori->accBodyVec[0],ori->accBodyVec[1],ori->accBodyVec[2],&ori->accQuat);

    Quaternion_rotate(&ori->orientationQuat, ori->accBodyVec, ori->accWorldVec);
    float correctionWorld[3];
    cross_product(ori->accWorldVec, ori->vertical, correctionWorld);

    float correctionBody[3];
    Quaternion_conjugate(&ori->orientationQuat, &ori->orientationQuatConj);
    Quaternion_rotate(&ori->orientationQuatConj, correctionWorld, correctionBody);

    correctionBody[0] = correctionBody[0] * 0.1;
    correctionBody[1] = correctionBody[1] * 0.1;
    correctionBody[2] = correctionBody[2] * 0.1;

    float GsSquared = vector_lenSquared(ori->accBodyVec) / (standardGravity * standardGravity);
    if (GsSquared > 0.81 && GsSquared < 1.21) {
        vector_sum(ori->gyroVec, correctionBody, ori->gyroVec);
    }
    Quaternion_fromRate(ori->gyroVec, dt, &ori->incrementalRotation);

    Quaternion tempQuat;

    Quaternion_multiply(&ori->orientationQuat, &ori->incrementalRotation, &tempQuat);
    Quaternion_copy(&tempQuat, &ori->orientationQuat);
    //Quaternion_lockY(&ori->orientationQuat, &ori->horQuat);
    Quaternion_toEulerZYX(&ori->orientationQuat, ori->eulerZYX);
    //ori->eulerZYX[2] = 0;
    //Quaternion_fromEulerZYX(ori->eulerZYX, &ori->horQuat);
}
