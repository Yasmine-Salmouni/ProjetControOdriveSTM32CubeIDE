#include "../Inc/MotorComputations.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

MotorComputations::MotorComputations(float torqueConstant)
    : torqueConstant(torqueConstant) {}

float MotorComputations::computeTorqueFromCurrent(float current) const {
    return current * torqueConstant;
}

float MotorComputations::computeCurrentFromTorque(float torque) const {
    return torque / torqueConstant;
}

float MotorComputations::computeOmega(float cadence_rpm) const {
    return cadence_rpm * 2.0f * M_PI / 60.0f;  // conversion tr/min → rad/s
}

float MotorComputations::computePower(float torque, float cadence_rpm) const {
    float omega = computeOmega(cadence_rpm);
    return torque * omega;
}

void MotorComputations::setTorqueConstant(float value) {
    torqueConstant = value;

}

