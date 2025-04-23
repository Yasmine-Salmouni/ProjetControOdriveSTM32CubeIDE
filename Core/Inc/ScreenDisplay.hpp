/*
 * ScreenDisplay.hpp
 *
 *  Created on: Apr 14, 2025
 *      Author: Yasmine Salmouni
 */

 #pragma once
 using namespace std;

 #include <cstring>
 #include <cstdio>
 #include <cstdint>

 #include "stm32f4xx_hal.h"
 #include "MotorController.hpp"
 class MotorController;

 /**
  * @brief Classe pour gérer la communication avec un écran Nextion via UART
  * Adaptée pour un écran Enhanced NX4832K035
  */

 class ScreenDisplay {
 public:

     ScreenDisplay(UART_HandleTypeDef* EcranUart);

     // Affichage des valeurs dynamiques
     void showCadence(float rpm);
     void showTorque(float torque);
     void showPower(float power);
     void showMode(const char* modeName);
     void showMode(ControlMode mode);
     void showGain(float LinearGain);
     void showDutyCycle(float duty);
     void showDirection(DirectionMode dir);

     void showCalibrationStatus(bool success);

     // Affichage de messages statiques
     void showError(const char* message);
     void showWelcome(); //utiliser dans le main
     void clearScreen(); //utiliser dans le main

     int32_t readInt32();
     float getUserCadence();
     float getUserPower();
     float getUserTorque();
     ControlMode getMode();
     float getUserLinearGain();
     bool getStop();
     bool getCalibrateRequest();
     DirectionMode getDirection();
     float getRampRate();

     void sendText(const char* component, const char* message);

 private:
     UART_HandleTypeDef* ecran_uart;

     // Méthodes internes d'envoi
     void sendCommand(const char* cmd);
     void sendValue(const char* component, float value, const char* format = "%.1f");

 };



