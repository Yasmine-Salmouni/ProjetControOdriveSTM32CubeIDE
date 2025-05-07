/*
 * MotorController.cpp
 *
 *  Created on: May 05 2025
 *      Author: Yasmine Salmouni
 */

#include <main.hpp>
#include "../Inc/VESCInterface.hpp"
#include "../Inc/ScreenDisplay.hpp"
#include "../Inc/MotorComputations.hpp"

#include <stdio.h>
#include <cmath>

MotorController::MotorController(UART_HandleTypeDef* controlUart, UART_HandleTypeDef* screenUart, float torquecst)
    : control_uart(controlUart),
      screen_uart(screenUart),
      direction(DirectionMode::FORWARD),
      controlMode(ControlMode::CADENCE),
      instruction(0.0f),
      linearGain(0.05f),
      torqueConstant(torquecst),
      lastAppliedCurrent(0.0f),
      ramp(6.0f),
      computations(torquecst)
{
    screen = new ScreenDisplay(screen_uart);
    vesc = new VESCInterface(control_uart);
}

//______________________________________________________________________________________

void MotorController::setLinearGain(float gain)
{
    linearGain = gain;
    if (screen) screen->sendValue("n0", linearGain, "%.2f");
}

float MotorController::getGain() {
    if (screen) screen->sendValue("n0", linearGain, "%.2f");
    return linearGain;
}
//____________________________________________________________________________________________

void MotorController::setrampRate(float rampRate)
{
   ramp = rampRate;
}

float MotorController::getRampRate() const {
    return ramp;
}
//______________________________________________________________________________

void MotorController::setDirection(DirectionMode dir) {
    direction = dir;
    if (screen) screen->sendText("t0", (dir == DirectionMode::REVERSE) ? "REVERSE" : "FORWARD");
}

DirectionMode MotorController::getDirection() const {
    if (screen) screen->sendText("t0", (direction == DirectionMode::REVERSE) ? "REVERSE" : "FORWARD");
    return direction;
}
//________________________________________________________________________________________________

void MotorController::setControlMode(ControlMode mode) {
    controlMode = mode;
    if (screen) {
        const char* modeName = "UNKNOWN";
        switch (mode) {
            case ControlMode::CADENCE: modeName = "Cadence"; break;
            case ControlMode::TORQUE: modeName = "Torque"; break;
            case ControlMode::POWER_CONCENTRIC: modeName = "Power Concentric"; break;
            case ControlMode::POWER_ECCENTRIC: modeName = "Power Eccentric"; break;
            case ControlMode::LINEAR: modeName = "Linear"; break;
        }
        screen->sendText("t0", modeName);
    }
}

ControlMode MotorController::getControlMode() {
    if (screen) {
        const char* modeName = "UNKNOWN";
        switch (controlMode) {
            case ControlMode::CADENCE: modeName = "Cadence"; break;
            case ControlMode::TORQUE: modeName = "Torque"; break;
            case ControlMode::POWER_CONCENTRIC: modeName = "Power Concentric"; break;
            case ControlMode::POWER_ECCENTRIC: modeName = "Power Eccentric"; break;
            case ControlMode::LINEAR: modeName = "Linear"; break;
        }
        screen->sendText("t0", modeName);
    }
    return controlMode;
}

//______________________________________________________________________________________

void MotorController::setInstruction(float value) {
    instruction = value;
    if (screen) screen->sendValue("n0", instruction, "%.2f");
    if (controlMode == ControlMode::LINEAR) return;
    switch (controlMode) {
        case ControlMode::CADENCE:
            setCadence(value);
            break;
        case ControlMode::TORQUE:
            setTorque(value);
            break;
        case ControlMode::POWER_CONCENTRIC:
            setPowerConcentric(value);
            break;
        case ControlMode::POWER_ECCENTRIC:
            setPowerEccentric(value);
            break;
        default:
            break;
    }
}

float MotorController::getInstructionValue() const {
    if (screen) screen->sendValue("n0", instruction, "%.2f");
    return instruction;
}

//_______________________________________________________________________________________________

void MotorController::setCadence(float rpm, float rampRate)
{
    float value = applyDirection(rpm);
    vesc->setRPM(value);
    if (screen) screen->sendValue("n0", rpm, "%.1f");
}

float MotorController::getCadence()
{
   float rpmValue = vesc->getRPM();
   if (screen) screen->sendValue("n0", rpmValue, "%.1f");
   return rpmValue;
}

//_________________________________________________________________________________________________

void MotorController::setTorque(float torque, float rampRate)
{
    float effectiveTorque = applyDirection(torque);
    float current = computations.computeCurrentFromTorque(effectiveTorque);
    vesc->setCurrent(current);
    if (screen) screen->sendValue("n0", torque, "%.2f");
}

float MotorController::getTorque() {
   float current = vesc->getCurrent();
   float torque = computations.computeTorqueFromCurrent(current);
   float final_torque = applyDirection(torque);
   if (screen) screen->sendValue("n0", final_torque, "%.2f");
   return final_torque;
}

//_____________________________________________________________________________

void MotorController::setPowerConcentric(float power, float rampRate)
{
   float cadence = getCadence();
   if (fabs(cadence) < 1.0f) {
       if (screen) screen->sendText("t0", "Cadence trop basse");
       vesc->setCurrent(0.0f);
       return;
   }

   float omega = computations.computeOmega(cadence);
   float torque = power / omega;
   float effectiveTorque = applyDirection(torque);
   float current = computations.computeCurrentFromTorque(effectiveTorque);
   lastAppliedCurrent = current;
   vesc->setCurrent(current);
   if (screen) screen->sendValue("n0", power, "%.1f");
}

void MotorController::setPowerEccentric(float power, float rampRate)
{
   float cadence = getCadence();
   if (fabs(cadence) < 1.0f) {
       if (screen) screen->sendText("t0", "Cadence trop basse");
       vesc->setCurrent(0.0f);
       return;
   }

   float omega = computations.computeOmega(cadence);
   float torque = -power / omega;
   float effectiveTorque = applyDirection(torque);
   float current = computations.computeCurrentFromTorque(effectiveTorque);
   lastAppliedCurrent = current;
   vesc->setCurrent(current);
   if (screen) screen->sendValue("n0", power, "%.1f");
}

float MotorController::getPower() {
    float torque = getTorque();
    float cadence = getCadence();
    float omega = computations.computeOmega(cadence);
    float power = computations.computePower(torque, omega);
    if (screen) screen->sendValue("n0", power, "%.1f");
    return power;
}

//_______________________________________________________________________________

void MotorController::setLinear(float gain, float cadence) {
    linearGain = gain;
    float torque = linearGain * cadence;
    float value = applyDirection(torque);
    float current = computations.computeCurrentFromTorque(value);
    lastAppliedCurrent = current;
    vesc->setCurrent(current);
    if (screen) {
        screen->sendValue("n0", torque, "%.2f");
        screen->sendValue("n0", gain, "%.2f");
    }
}

void MotorController::update(float measured_cadence) {
    if (controlMode == ControlMode::LINEAR) {
        setLinear(linearGain, measured_cadence);
    }
}

//_________________________________________________________________________________

void MotorController::stop(float rampRate)
{
   float current = lastAppliedCurrent;
   const float timeStepMs = 50.0f;
   const float timeStepS = timeStepMs / 1000.0f;
   const float maxStep = rampRate * timeStepS;

   while (fabs(current) > 0.05f) {
       if (current > 0) {
           current -= maxStep;
           if (current < 0) current = 0.0f;
       } else {
           current += maxStep;
           if (current > 0) current = 0.0f;
       }
       vesc->setCurrent(current);
       HAL_Delay(static_cast<uint32_t>(timeStepMs));
   }
   vesc->setCurrent(0.0f);
   instruction = 0.0f;
   lastAppliedCurrent = 0.0f;
   if (screen) screen->sendValue("n0", 0.0f, "%.1f");
}

//_________________________________________________________________________

float MotorController::applyDirection(float value) {
    return (direction == DirectionMode::REVERSE) ? -value : value;
}

//_________________________________________________________________________________

void MotorController::setTorqueConstant(float torquecst)
{
   torqueConstant = torquecst;
   computations.setTorqueConstant(torquecst);
   if (screen) screen->sendValue("n0", torqueConstant, "%.4f");
}

float MotorController::getTorqueConstant() const {
    return torqueConstant;
}

void MotorController::calibrateTorqueConstant() {
    const float testCurrent = 5.0f;
    vesc->setCurrent(testCurrent);
    HAL_Delay(1000);

    float measuredTorque = getTorque();

    vesc->setCurrent(0.0f);
    HAL_Delay(100);

    if (measuredTorque <= 0.0f) {
        if (screen) screen->sendText("t0", "Erreur calibration");
        return;
    }

    float newKt = measuredTorque / testCurrent;

    if (newKt > 0.01f && newKt < 1.0f) {
        setTorqueConstant(newKt);
        if (screen) screen->sendText("t0", "Calibration OK");
    } else {
        if (screen) screen->sendText("t0", "Erreur calibration");
    }
}

//_________________________________________________________________________________

float MotorController::getDutyCycle()
{
    float duty = vesc->getDutyCycle();
    if (screen) screen->sendValue("n0", duty * 100.0f, "%.1f");
    return duty;
}
