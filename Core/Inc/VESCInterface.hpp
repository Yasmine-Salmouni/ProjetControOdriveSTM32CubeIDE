#pragma once
using namespace std;

#include <cstdint>
#include <cstring>

#include "stm32f4xx_hal.h"
//#include "ScreenDisplay.hpp"
class ScreenDisplay;


class VESCInterface {
public:
    explicit VESCInterface(UART_HandleTypeDef* ControlUart); // explicit empêche les conversions automatiques (implicites) qui peuvent créer des comportements imprévus.

    void setCurrent(float current);
    void setRPM(int32_t rpm);
    bool getValues();
    float getRPM();
    float getCurrent();
    float getDutyCycle();


private:
    UART_HandleTypeDef* control_uart;
    UART_HandleTypeDef* screen_uart;

    // Buffers
    uint8_t txBuffer[64]; // zone mémoire utilisée pour préparer les paquets à envoyer au VESC
    uint8_t rxBuffer[128]; //zone mémoire pour stocker la réponse reçue depuis le VESC

    // Extracted values
    float rpm; //Vitesse de rotation du moteur exprimée en tours par minute (RPM). Elle est renvoyée par le VESC via COMM_GET_VALUES.
    float inputCurrent; //C’est le courant consommé par le VESC lui-même, depuis la source d’alimentation. Elle est renvoyée par le VESC via COMM_GET_VALUES.
    float dutyCycle; //Cycle de travail PWM appliqué au moteur. (Le VESC gère lui-même le PWM interne pour contrôler le moteur.)

    ScreenDisplay* screen;

    void sendPacket(uint8_t* data, uint16_t len);
    bool receivePacket(uint8_t* buffer, uint16_t& len, uint32_t timeout = 100);
    uint16_t crc16(const uint8_t* data, uint16_t len);
};
