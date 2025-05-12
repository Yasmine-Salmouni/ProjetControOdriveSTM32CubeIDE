#pragma once
using namespace std;

#include <cstdint>
#include <cstring>

#include "stm32f4xx_hal.h"
//#include "ScreenDisplay.hpp"
class ScreenDisplay;

#include "main.hpp"   // pour HAL_UART_Transmit, HAL_UART_Receive, etc.
#include <stdint.h>
#include <string.h>

// Identifiants de commande VESC (COMM_PACKET_ID)
#define COMM_GET_VALUES   4
#define COMM_SET_CURRENT  6
#define COMM_SET_RPM      8

// Structure pour stocker les valeurs reçues
typedef struct {
    float rpm;
    float current;
    float dutyCycle;
    float voltage;
    // ... (autres champs éventuellement)
} VESCValues;

class VESCInterface {
private:
    UART_HandleTypeDef *uart;   // handle UART utilisé pour communiquer
    VESCValues values;          // dernières valeurs reçues

    // Méthode interne pour envoyer un paquet et attendre une réponse (pour GET_VALUES)
    int getValues() {
        // Construire la requête COMM_GET_VALUES
        uint8_t txBuf[6];  // 6 octets: start, len, id, CRC16(2B), stop
        uint16_t crc;
        txBuf[0] = 0x02;       // Start byte (short packet)
        txBuf[1] = 0x01;       // Length = 1 (payload size)
        txBuf[2] = COMM_GET_VALUES;  // Payload: command ID
        crc = crc16(&txBuf[2], 1);   // CRC16 sur 1 octet (0x04)
        txBuf[3] = (crc >> 8) & 0xFF;  // CRC high byte
        txBuf[4] = crc & 0xFF;        // CRC low byte
        txBuf[5] = 0x03;       // End byte

        // Envoyer la requête
        if (HAL_UART_Transmit(uart, txBuf, sizeof(txBuf), 100) != HAL_OK) {
            return 0;  // échec envoi
        }

        // Lire l'octet de départ de la réponse
        uint8_t startByte;
        if (HAL_UART_Receive(uart, &startByte, 1, 100) != HAL_OK) {
            return 0;  // timeout ou pas de réponse
        }
        if (startByte != 0x02 && startByte != 0x03) {
            return 0;  // octet de start invalide
        }

        // Lire la longueur
        uint16_t payloadLen = 0;
        if (startByte == 0x02) {
            // paquet court -> 1 octet de longueur
            uint8_t lenByte;
            if (HAL_UART_Receive(uart, &lenByte, 1, 100) != HAL_OK) return 0;
            payloadLen = lenByte;
        } else {
            // paquet long -> 2 octets de longueur
            uint8_t lenBytes[2];
            if (HAL_UART_Receive(uart, lenBytes, 2, 100) != HAL_OK) return 0;
            payloadLen = ((uint16_t)lenBytes[0] << 8) | lenBytes[1];
        }

        // Lire tout le payload de longueur connue
        uint8_t rxPayload[256];  // buffer assez grand (adapter si besoin)
        if (payloadLen > sizeof(rxPayload)) {
            // Si la longueur annoncée dépasse le buffer prévu, on évite le dépassement
            // On pourrait lire et jeter les octets, mais on retourne erreur par sûreté
            for (uint16_t i = 0; i < payloadLen + 3; ++i) {
                uint8_t dump;
                HAL_UART_Receive(uart, &dump, 1, 10); // flush
            }
            return 0;
        }
        if (HAL_UART_Receive(uart, rxPayload, payloadLen, 100) != HAL_OK) {
            return 0;  // échec lecture payload
        }

        // Lire CRC16 (2 octets) et l'octet de fin
        uint8_t crcBytes[2];
        uint8_t endByte;
        if (HAL_UART_Receive(uart, crcBytes, 2, 100) != HAL_OK) return 0;
        if (HAL_UART_Receive(uart, &endByte, 1, 100) != HAL_OK) return 0;
        if (endByte != 0x03) {
            return 0;  // mauvais octet de fin
        }

        // Vérifier le CRC
        uint16_t crcReceived = ((uint16_t)crcBytes[0] << 8) | crcBytes[1];
        uint16_t crcCalc = crc16(rxPayload, payloadLen);
        if (crcReceived != crcCalc) {
            return 0;  // CRC invalide
        }

        // Vérifier l'ID de commande dans le payload
        if (payloadLen < 1 || rxPayload[0] != COMM_GET_VALUES) {
            return 0;  // pas le bon paquet de réponse
        }

        // Extraire les valeurs nécessaires du payload
        // En fonction du mapping connu de COMM_GET_VALUES:
        // rxPayload[0] = 0x04 (ID)
        // rxPayload[1-2] = temp_fet (int16)
        // rxPayload[3-4] = temp_motor (int16)
        // rxPayload[5-8] = avg_motor_current (int32, échelle x100)
        // rxPayload[21-22] = duty_now (int16, échelle x1000)
        // rxPayload[23-26] = rpm (int32)
        if (payloadLen >= 27) {
            // Courant moteur (float en A)
            int32_t current_int = (rxPayload[5] << 24) | (rxPayload[6] << 16) |
                                   (rxPayload[7] << 8) | rxPayload[8];
            values.current = current_int / 100.0f;  // échelle 100 -> A

            // Duty cycle (ratio)
            int16_t duty_int = (int16_t)((rxPayload[21] << 8) | rxPayload[22]);
            values.dutyCycle = duty_int / 1000.0f;   // échelle 1000 -> [0,1] (ou 0-100%)

            // RPM
            int32_t rpm_int = (rxPayload[23] << 24) | (rxPayload[24] << 16) |
                               (rxPayload[25] << 8) | rxPayload[26];
            values.rpm = (float)rpm_int;

            // Voltage (float en V)
            uint16_t rawV = ((uint16_t)rxPayload[27] << 8) | rxPayload[28];
            values.voltage = rawV / 10.0f;  // division par 10 pour avoir la tension en volts


        } else {
            return 0;  // payload trop court pour contenir les valeurs attendues
        }

        return 1;  // succès
    }

public:
    VESCInterface(UART_HandleTypeDef *huart) : uart(huart) {
        memset(&values, 0, sizeof(values));
    }

    uint16_t crc16(const uint8_t *data, uint16_t len) {
        uint16_t crc = 0x0000;
        for (uint16_t i = 0; i < len; ++i) {
            crc ^= (uint16_t)data[i] << 8;
            for (uint8_t bit = 0; bit < 8; ++bit) {
                if (crc & 0x8000) {
                    crc <<= 1;
                    crc ^= 0x1021;
                } else {
                    crc <<= 1;
                }
            }
        }
        return crc;
    }


    void setCurrent(float current) {
        // Préparer la valeur en mA
        int32_t milliamp = (int32_t)(current * 1000.0f);
        // Construire le paquet à envoyer
        uint8_t buf[11]; // start(1)+len(1)+id(1)+data(4)+crc(2)+end(1) = 10 octets, 11 par sécurité
        uint16_t crc;
        buf[0] = 0x02;            // Start (short packet)
        buf[1] = 0x05;            // Length = 5 (1+4 bytes payload)
        buf[2] = COMM_SET_CURRENT;
        buf[3] = (milliamp >> 24) & 0xFF;
        buf[4] = (milliamp >> 16) & 0xFF;
        buf[5] = (milliamp >> 8) & 0xFF;
        buf[6] = milliamp & 0xFF;
        crc = crc16(&buf[2], 5);  // CRC sur payload (5 octets)
        buf[7] = (crc >> 8) & 0xFF;
        buf[8] = crc & 0xFF;
        buf[9] = 0x03;
        // Envoyer (on ignore le retour car pas de réponse attendue)
        HAL_UART_Transmit(uart, buf, 10, 100);
    }

    void setRPM(int32_t rpm) {
        uint8_t buf[11];
        uint16_t crc;
        buf[0] = 0x02;
        buf[1] = 0x05;            // payload 5 octets
        buf[2] = COMM_SET_RPM;
        buf[3] = (rpm >> 24) & 0xFF;
        buf[4] = (rpm >> 16) & 0xFF;
        buf[5] = (rpm >> 8) & 0xFF;
        buf[6] = rpm & 0xFF;
        crc = crc16(&buf[2], 5);
        buf[7] = (crc >> 8) & 0xFF;
        buf[8] = crc & 0xFF;
        buf[9] = 0x03;
        HAL_UART_Transmit(uart, buf, 10, 100);
    }

    float getRPM() {
        if (!getValues()) {
            return -1.0f; // erreur de comm
        }
        return values.rpm;
    }

    float getCurrent() {
        if (!getValues()) {
            return -1.0f;
        }
        return values.current;
    }

    float getDutyCycle() {
        if (!getValues()) {
            return -1.0f;
        }
        return values.dutyCycle;
    }

    float getVoltage() {
        getValues();
        return values.voltage;
    }

};



