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
//_____________________________________________________________________________________________
void MotorController::setTorqueConstant(float torquecst)
{
   torqueConstant = torquecst;
   computations.setTorqueConstant(torquecst);
}

//permet de tester setTorqueConstant et calibrateTorqueConstant
float MotorController::getTorqueConstant() const { 
	//screen->sendValue("n0", torqueConstant, "%.4f");
    return torqueConstant;
}

void MotorController::calibrateTorqueConstant() {
    const float testCurrent = 5.0f;
    vesc->setCurrent(testCurrent);

    HAL_Delay(1000);

    float measuredTorque = getTorque();
    screen->sendValue("Mtorque", measuredTorque, "%.4f");

    vesc->setCurrent(0.0f);
    HAL_Delay(100);

    /*if (measuredTorque <= 0.0f) {
        screen->sendText("t0", "Erreur calibration");
        return;
    }*/

    float newKt = measuredTorque / testCurrent;

    /*if (newKt<0)
    {
    	newKt= -newKt;
    }*/
    setTorqueConstant(newKt);

    /*if (newKt > 0.01f && newKt < 1.0f) {
        setTorqueConstant(newKt);
        screen->sendText("t0", "Calibration OK");
    } else {
        screen->sendText("t0", "Erreur calibration");
    }*/
    screen->sendText("calib_stat", "Calibration OK");
}


//______________________________________________________________________________________

void MotorController::setLinearGain(float gain)
{
    linearGain = gain;
    //if (screen) screen->sendValue("n0", linearGain, "%.2f");
}

//permet de tester setLinearGain et utilisée dans l'application
float MotorController::getGain() {
    //if (screen) screen->sendValue("n0", linearGain, "%.2f");
    return linearGain;
}
//____________________________________________________________________________________________

void MotorController::setrampRate(float rampRate)
{
   ramp = rampRate;
}

//permet de tester setrampRate
float MotorController::getRampRate() const {
    //if (screen) screen->sendValue("n0", ramp, "%.2f");
    return ramp;
}
//______________________________________________________________________________

void MotorController::setDirection(DirectionMode dir) {
    direction = dir;
    //if (screen) screen->sendText("t0", (dir == DirectionMode::REVERSE) ? "REVERSE" : "FORWARD");
}

DirectionMode MotorController::getDirection() const {
    //if (screen) screen->sendText("t0", (direction == DirectionMode::REVERSE) ? "REVERSE" : "FORWARD");
    return direction;
}
//________________________________________________________________________________________________

void MotorController::setControlMode(ControlMode mode) {
    controlMode = mode;
    /*if (screen) {
        const char* modeName = "UNKNOWN";
        switch (mode) {
            case ControlMode::CADENCE: modeName = "Cadence"; break;
            case ControlMode::TORQUE: modeName = "Torque"; break;
            case ControlMode::POWER_CONCENTRIC: modeName = "Power Concentric"; break;
            case ControlMode::POWER_ECCENTRIC: modeName = "Power Eccentric"; break;
            case ControlMode::LINEAR: modeName = "Linear"; break;
        }
        screen->sendText("t0", modeName);
    }*/
}

ControlMode MotorController::getControlMode() {
    /*if (screen) {
        const char* modeName = "UNKNOWN";
        switch (controlMode) {
            case ControlMode::CADENCE: modeName = "Cadence"; break;
            case ControlMode::TORQUE: modeName = "Torque"; break;
            case ControlMode::POWER_CONCENTRIC: modeName = "Power Concentric"; break;
            case ControlMode::POWER_ECCENTRIC: modeName = "Power Eccentric"; break;
            case ControlMode::LINEAR: modeName = "Linear"; break;
        }
        screen->sendText("t0", modeName);
    }*/
    return controlMode;
}

//______________________________________________________________________________________

void MotorController::setInstruction(float value) {
    instruction = value;

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

//permet de tester setInstruction
float MotorController::getInstructionValue() const {
    //if (screen) screen->sendValue("n0", instruction, "%.2f");
    return instruction;
}

//_______________________________________________________________________________________________

void MotorController::setCadence(float rpm)
{
    float value = applyDirection(rpm);
    vesc->setRPM(value);
    //if (screen) screen->sendValue("n0", rpm, "%.1f");
}

float MotorController::getCadence()
{
   float rpmValue = vesc->getRPM();
   //if (screen) screen->sendValue("n0", rpmValue, "%.1f");
   return rpmValue;
}

//_________________________________________________________________________________________________

void MotorController::setTorque(float torque)
{
    float effectiveTorque = applyDirection(torque);
    float current = computations.computeCurrentFromTorque(effectiveTorque);
    vesc->setCurrent(current);

}

//permet de tester setTorque
void MotorController::setCurrent(float current) {
   vesc->setCurrent(current);
}

//permet de tester setCurrent
float MotorController::getCurrent() {
   float current = vesc->getCurrent();
   //screen->sendValue("n0", current, "%.4f");
   return current;
}

float MotorController::getTorque() {
   float current = vesc->getCurrent();
   float torque = computations.computeTorqueFromCurrent(current);
   float final_torque = applyDirection(torque);
   //if (screen) screen->sendValue("n0", final_torque, "%.4f");
   return final_torque;
}

//_____________________________________________________________________________

void MotorController::setPowerConcentric(float power)
{
   float cadence = getCadence();
   //screen->sendValue("n0", cadence, "%.1f");

   if (fabs(cadence) < 1.0f) {
       //if (screen) screen->sendText("t0", "Cadence trop basse");
       vesc->setCurrent(0.0f);
       return;
   }

   float omega = computations.computeOmega(cadence);
   float torque = power / omega;
   float effectiveTorque = applyDirection(torque);
   float current = computations.computeCurrentFromTorque(effectiveTorque);
   lastAppliedCurrent = current;
   vesc->setCurrent(current);
   //screen->sendValue("n0", current, "%.1f");
}

void MotorController::setPowerEccentric(float power)
{
   float cadence = getCadence();
   if (fabs(cadence) < 1.0f) {
       //if (screen) screen->sendText("t0", "Cadence trop basse");
       vesc->setCurrent(0.0f);
       return;
   }

   float omega = computations.computeOmega(cadence);
   float torque = -power / omega;
   float effectiveTorque = applyDirection(torque);
   float current = computations.computeCurrentFromTorque(effectiveTorque);
   lastAppliedCurrent = current;
   vesc->setCurrent(current);
   //if (screen) screen->sendValue("n0", power, "%.1f");
}

float MotorController::getPower() {
    float torque = getTorque();
    float cadence = getCadence();
    float omega = computations.computeOmega(cadence);
    float power = computations.computePower(torque, omega);
    //if (screen) screen->sendValue("n0", power, "%.1f");
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
    /*if (screen) {
        screen->sendValue("n0", torque, "%.2f");
        screen->sendValue("n0", gain, "%.2f");
    }*/
}

void MotorController::update(float measured_cadence) {
    if (controlMode == ControlMode::LINEAR) {
        setLinear(linearGain, measured_cadence);
    }
}

//_________________________________________________________________________________

void MotorController::stop()
{
   float rampRate=ramp;
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
   //if (screen) screen->sendValue("n0", 0.0f, "%.1f");
}
//_________________________________________________________________________
// Gestion de la batterie

float MotorController::getVoltage() const
{
    float voltage = vesc->getVoltage();
    screen->sendValue("voltage", voltage, "%.4f");

    // Estimation du pourcentage de charge (entre 42.0V et 54.6 V)
    const float minV = 40.3f; // le moteur s’arrête complètement
    const float maxV = 54.6f; //tension maximale en pleine charge pour batterie Li-ion 13S
    uint8_t percent = 0;

    if (voltage >= maxV) {
        percent = 100;
    } else if (voltage <= minV) {
        percent = 0;
    } else {
        percent = static_cast<uint8_t>(((voltage - minV) / (maxV - minV)) * 100.0f);
    }

    // Alerte selon le niveau
    if (voltage < 40.3f) // le moteur s’arrête complètement
    {
        screen->sendText("batterie", "Recharge immediate");
    }
    else if (voltage < 44.2f) // le VESC commence à réduire la puissance
    {
        screen->sendText("batterie", "Batterie faible");
    }
    else
    {
        char text[64];
        snprintf(text, sizeof(text), " %.1f V | %d%%", voltage, percent);
        screen->sendText("batterie", text);
    }
    return voltage;
}

//_________________________________________________________________________

float MotorController::applyDirection(float value) {
    return (direction == DirectionMode::REVERSE) ? -value : value;
}

//_________________________________________________________________________________

float MotorController::getDutyCycle()
{
    float duty = vesc->getDutyCycle();
    //if (screen) screen->sendValue("n0", duty * 100.0f, "%.1f");
    return duty;
}

//________________________________________________________________________________

void MotorController::updateScreen() {
    if (!screen) return;  // Sécurité : écran non initialisé

    float LinearGain = getGain();
    DirectionMode direction = getDirection();
    ControlMode mode = getControlMode();
    float rpm     = getCadence();
    float torque  = getTorque();
    float power   = getPower();
    float dutyCycle = getDutyCycle();
    float voltage = getVoltage();

    // Affichage à l’écran
    //screen->showWelcome();
    screen->showGain(LinearGain);
    screen->showDirection(direction);
    screen->showMode(mode);
    screen->showCadence(rpm);
    screen->showTorque(torque);
    screen->showPower(power);
    screen->showDutyCycle(dutyCycle);
    screen->showAll(rpm, torque, power);
}

//________________________________________________________________________________
void MotorController::updateFromScreen()
{
    if (!screen) return;  // Sécurité : écran non initialisé

    DirectionMode selectedDirection = screen->getDirection();
    setDirection(selectedDirection);

    ControlMode selectedMode = screen->getMode();
    setControlMode(selectedMode);

    switch (controlMode)
    {
        case ControlMode::CADENCE:
        {
            float rpm = screen->getUserCadence();
            setInstruction(rpm);
            break;
        }

        case ControlMode::TORQUE:
        {
            float torque = screen->getUserTorque();
            setInstruction(torque);
            break;
        }

        case ControlMode::POWER_CONCENTRIC:
        case ControlMode::POWER_ECCENTRIC:
        {
            float power = screen->getUserPower();
            setInstruction(power);
            break;
        }

        case ControlMode::LINEAR:
        {
            float gain = screen->getUserLinearGain();
            setLinearGain(gain);
            break;
        }

        default:
            break;
    }
    if (screen->getStop())
    {
        stop(3.0f);  // Stop progressif avec rampRate = 3 A/s (à adapter si besoin)
    }

    if (screen->getCalibrateRequest())
    {
        calibrateTorqueConstant();
    }
}

