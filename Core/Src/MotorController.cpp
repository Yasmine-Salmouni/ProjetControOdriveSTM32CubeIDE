/*
 * MotorController.cpp
 *
 *  Created on: Apr 14, 2025
 *      Author: Yasmine Salmouni
 */

 #include "../Inc/MotorController.hpp"
 #include "../Inc/main.h"

#include "../Inc/ScreenDisplay.hpp"
#include "../Inc/VESCInterface.hpp"
#include "../Inc/MotorComputations.hpp"

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
 //Par défaut le moteur est en modes forward et cadence avec une vitesse nulle

 void MotorController::setDirection(DirectionMode dir) {
     direction = dir;
 }

 void MotorController::setControlMode(ControlMode mode) {
     controlMode = mode;
 }

 void MotorController::setInstruction(float value) {
     instruction = value;
     if (controlMode == ControlMode::LINEAR) return;  // linear se gère dynamiquement
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

 void MotorController::setLinearGain(float gain)
 {
     linearGain = gain;
 }

 void MotorController::setTorqueConstant(float torquecst)
 {
    torqueConstant = torquecst;
    computations.setTorqueConstant(torquecst);

 }

 void MotorController::setrampRate(float rampRate)
 {
    ramp = rampRate;
 }

 void MotorController::setCadence(float rpm, float rampRate) //Implémenter une version avec rampRate
 {
     float value = applyDirection(rpm);
     //sendCommand("v 0 %.2f\n", value);
     vesc->setRPM(value);
 }

 void MotorController::setTorque(float torque, float rampRate) //Implementer une version avec rampRate
 {
     float effectiveTorque = applyDirection(torque);
     float current = computations.computeCurrentFromTorque(effectiveTorque);
     vesc->setCurrent(current);
 }

 float MotorController::getCadence()
 {
    float rpmValue = vesc->getRPM();  // Lire la vitesse moteur (en tr/min)

    if (rpmValue < 0.0f) {
        // Affichage erreur si lecture échouée
        screen->showError("Erreur: réception cadence");
        return -1.0f;
    }

    return rpmValue;  // Retourne directement la cadence (RPM)
 }

 float MotorController::getTorque() {
    float current = vesc->getCurrent();  // Récupère le courant réel du moteur

    if (current < 0.0f) {
        screen->showError("Erreur: réception courant");
        return -1.0f;  // Erreur de lecture
    }

    float torque = computations.computeTorqueFromCurrent(current);
    return applyDirection(torque);  // Respecte le sens FORWARD/REVERSE
}

float MotorController::getDutyCycle()
{
    float duty = vesc->getDutyCycle();  // Lecture via VESCInterface

    if (duty < -1.1f || duty > 1.1f) {  // Valeur hors plage → erreur
        screen->showError("Erreur: Duty invalide");
        return -2.0f;
    }

    if (duty > 0.95f)
    {
        screen->sendText("t0", "ALERTE: Duty élevé !");
    }

    return duty;
}

float MotorController::getPower() {
    float torque = getTorque();

    if (torque < 0.0f) {
        screen->showError("Erreur: couple invalide");
        return -1.0f;
    }

    float cadence = getCadence();  // tr/min

    if (cadence < 0.0f) {
        screen->showError("Erreur: réception cadence");
        return -1.0f;
    }

    // Conversion cadence → vitesse angulaire ω (rad/s)
    float omega = computations.computeOmega(cadence);

    // Puissance mécanique P = τ × ω
    float power = computations.computePower(torque, omega);

    return power;  // En watts signé
}

ControlMode MotorController::getControlMode() {
    return controlMode;
}

float MotorController::getGain() {
    return linearGain;
}

 void MotorController::setPowerConcentric(float power, float rampRate)
 {
    float cadence = getCadence();  // Lecture de la vitesse réelle

    // Vérifie si getCadence() a échoué (renvoie une valeur d’erreur)
    if (cadence < 0.0f)
    {
        screen->showError("Erreur: réception cadence");
        return;
    }

    // Sécurité : éviter division par zéro ou valeurs trop basses
    if (cadence < 1.0f)
    {
        cadence = 1.0f;
    }

    // Conversion cadence (tr/min) → vitesse angulaire ω (rad/s)
    float omega = computations.computeOmega(cadence);

    // Calcul du couple réel à appliquer : τ = P / ω
    float torque = power / omega;

    // Appliquer la direction (FORWARD ou REVERSE)
    float effectiveTorque = applyDirection(torque);

    // Conversion couple → courant moteur : I = τ / Kt
    float current = computations.computeCurrentFromTorque(effectiveTorque);
    lastAppliedCurrent = current;

    // Envoi de la commande au VESC
    vesc->setCurrent(current);
 }


 void MotorController::setPowerEccentric(float power, float rampRate)
 {
    float cadence = getCadence();  // Lecture de la vitesse réelle

    // Vérifie si la lecture a échoué
    if (cadence < 0.0f)
    {
        screen->showError("Erreur: réception cadence");
        return;
    }

    // Sécurité : éviter division par zéro
    if (cadence < 1.0f)
    {
        cadence = 1.0f;
    }

    // Conversion cadence → vitesse angulaire ω (rad/s)
    float omega = computations.computeOmega(cadence);

    // Calcul du couple nécessaire (négatif pour excentrique)
    float torque = -power / omega;

    // Applique la direction choisie (FORWARD ou REVERSE)
    float effectiveTorque = applyDirection(torque);

    // Conversion couple → courant moteur : I = τ / Kt
    float current = computations.computeCurrentFromTorque(effectiveTorque);
    lastAppliedCurrent = current;

    // Envoi au VESC
    vesc->setCurrent(current);
 }


 void MotorController::setLinear(float gain, float cadence) {
     linearGain = gain;
     float torque = linearGain * cadence;
     float value = applyDirection(torque);
     // Conversion couple → courant : I = τ / Kt
     float current = computations.computeCurrentFromTorque(value);
     lastAppliedCurrent = current;

     // Envoi au VESC
     vesc->setCurrent(current);
 }

 void MotorController::update(float measured_cadence) {
     if (controlMode == ControlMode::LINEAR) {
         setLinear(linearGain, measured_cadence);
     }
 }

 void MotorController::stop(float rampRate)
 {
    // Lire le courant actuel
    float current = lastAppliedCurrent;  // À maintenir dans ta classe
    const float timeStepMs = 50.0f;      // Intervalle entre chaque pas (50 ms)
    const float timeStepS = timeStepMs / 1000.0f; //conversion en secondes
    const float maxStep = rampRate * timeStepS; //On calcule combien on doit diminuer le courant à chaque pas

    while (fabs(current) > 0.05f)
    {  // Tant qu'on n'est pas (quasiment) à 0
        if (current > 0) //Si le courant est positif, on le réduit vers zéro
        {
            current -= maxStep;
            if (current < 0) current = 0.0f;
        }
        else //Si le courant est négatif (ex : freinage), on l’augmente vers zéro

        {
            current += maxStep;
            if (current > 0) current = 0.0f;
        }

        vesc->setCurrent(current);
        HAL_Delay(static_cast<uint32_t>(timeStepMs));
    }

    // Finalise à zéro pour s'assurer que c'est bien arrêté
    vesc->setCurrent(0.0f);
    instruction = 0.0f;
    lastAppliedCurrent = 0.0f;
}

 float MotorController::applyDirection(float value) {
     return (direction == DirectionMode::REVERSE) ? -value : value;
 }

 /*if (direction == DirectionMode::REVERSE)
     return -value;
 else
     return value;*/


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

void MotorController::updateScreen() {
    if (!screen) return;  // Sécurité : écran non initialisé

    float rpm     = getCadence();
    float torque  = getTorque();
    float power   = getPower();
    float dutyCycle = getDutyCycle();
    ControlMode mode = getControlMode();
    float LinearGain = getGain();
    DirectionMode direction = screen->getDirection();

    // Affichage à l’écran
    //screen->showWelcome();
    screen->showCadence(rpm);
    screen->showTorque(torque);
    screen->showPower(power);
    screen->showDutyCycle(dutyCycle);
    screen->showMode(mode);
    screen->showGain(LinearGain);
    screen->showDirection(direction);
}

void MotorController::calibrateTorqueConstant() {
    const float testCurrent = 5.0f;  // Appliquer 5 A
    vesc->setCurrent(testCurrent);

    HAL_Delay(1000);  // Attente pour stabilisation (1 sec)

    float measuredTorque = getTorque();

    vesc->setCurrent(0.0f);  // Sécurité : stop après mesure

    if (measuredTorque <= 0.0f) {
        screen->showError("Erreur: pas de couple");
        return;
    }

    float newKt = measuredTorque / testCurrent; //simple calcule a partir des valeurs mesurées

    if (newKt > 0.01f && newKt < 1.0f) { //documentation
        screen->showCalibrationStatus(true);  // ✅ calibration OK
        setTorqueConstant(newKt);
    } else {
        screen->showCalibrationStatus(false); // ❌ calibration échouée
    }
}

/*On donne 1.95A au moteur, il fournit 0.39 Nm,
ce qui devient 15 Nm au pédalier via le réducteur.
On n'a pas à tenir compte de la réduction dans le code*/

ScreenDisplay* MotorController::getscreen()
{
    return screen;
}
