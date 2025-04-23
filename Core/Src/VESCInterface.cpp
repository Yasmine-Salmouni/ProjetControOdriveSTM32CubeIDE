#include "../Inc/VESCInterface.hpp"

#include "../Inc/ScreenDisplay.hpp"

//C'est VESCI qui décide
#define COMM_SET_CURRENT    5
#define COMM_SET_RPM        8
#define COMM_GET_VALUES     4

VESCInterface::VESCInterface(UART_HandleTypeDef* ControlUart)
    : control_uart(ControlUart), rpm(0.0f), inputCurrent(0.0f), dutyCycle(0.0f)
    {
        screen = new ScreenDisplay(screen_uart);
    }

void VESCInterface::setCurrent(float current)
//Cette fonction sert à envoyer une commande au VESC pour lui demander: aplique un courant current (en ampères) au moteur
{
    uint8_t payload[5];
    payload[0] = COMM_SET_CURRENT;
    int32_t iCurrent = static_cast<int32_t>/*Ecriture de la valeur en mA sur 32bits donc 4 octets*/(current * 1000.0f/*Conversion du float en mA*/);
    //On chisit 32 bits car ca permet de représenter un grand intervalles de courants (de -2M à +2M mA)
    payload[1] = (iCurrent >> 24) & 0xFF; //payload[1] comporte le premier octet le plus fort
    payload[2] = (iCurrent >> 16) & 0xFF; //payload[2] comporte le deuxième octet le plus fort
    payload[3] = (iCurrent >> 8) & 0xFF; //payload[3] comporte le troisième octet le plus fort
    payload[4] = iCurrent & 0xFF; ////payload[4] comporte l'octet le plus faible
    //pour 3,5A payload = [commande, 0x00, 0x00, 0x0D, 0xAC]
    sendPacket(payload, 5); //le 5 est la longueur du message
}

void VESCInterface::setRPM(int32_t rpmValue) //Un int32_t peut stocker de -2,147,483,648  →  +2,147,483,647 (en tr/min) ce qui est suffisant pour tout les moteurs
{
    uint8_t payload[5];
    payload[0] = COMM_SET_RPM;
    payload[1] = (rpmValue >> 24) & 0xFF;
    payload[2] = (rpmValue >> 16) & 0xFF;
    payload[3] = (rpmValue >> 8) & 0xFF;
    payload[4] = rpmValue & 0xFF;
    sendPacket(payload, 5);
}

float VESCInterface::getRPM() {
    if (getValues())
    {
        return rpm;
    }
    return -1.0f;
}

bool VESCInterface::getValues()
//Cette fonction permet de lire l'etat actuel du moteur
{
    uint8_t cmd = COMM_GET_VALUES;
    sendPacket(&cmd, 1);

    uint16_t len;
    if (!receivePacket(rxBuffer, len)) return false;

    if (rxBuffer[0] != COMM_GET_VALUES) return false; //rxBuffer[0] n’est pas le 1er octet total de la trame C’est le 1er octet du payload

    uint8_t* ptr = &rxBuffer[1]; //On fait pointer ptr vers la première donnée utile

    ptr += 4;  //  saute Temp FET
    ptr += 4;  //  saute Temp moteur
    ptr += 4;  //  saute courant moteur
    ptr += 4;  //  saute courant batterie (input current)
    ptr += 4;  //  saute ID moteur
    ptr += 4;  //  saute IQ moteur

    //Après ces 6 sauts, ptr pointe maintenant vers le champ suivant : le RPM (encodé sur 4 octets)

    int32_t rpmRaw = (ptr[0] << 24) | (ptr[1] << 16) | (ptr[2] << 8) | ptr[3];
    rpm = static_cast<float>(rpmRaw);
    return true;
}

float VESCInterface::getCurrent() {
    uint8_t cmd = COMM_GET_VALUES;
    sendPacket(&cmd, 1);

    uint16_t len;
    if (!receivePacket(rxBuffer, len)) return -1.0f;

    if (rxBuffer[0] != COMM_GET_VALUES) return -1.0f;

    uint8_t* ptr = &rxBuffer[1]; // Pointeur sur le début du payload (après code commande)

    ptr += 4;  // Temp FET
    ptr += 4;  // Temp motor
    ptr += 4;  // Current motor

    union {
        float f;
        uint8_t b[4];
    } uCurrent;
    /*Une union est un type spécial qui permet de partager la même zone mémoire entre plusieurs variables.
    Ça veut dire que toutes les variables dans la union occupent le même espace mémoire, on peux accéder à ces données sous différentes formes.*/

    uCurrent.b[0] = ptr[0];
    uCurrent.b[1] = ptr[1];
    uCurrent.b[2] = ptr[2];
    uCurrent.b[3] = ptr[3];

    return uCurrent.f;  // En ampères
}

float VESCInterface::getDutyCycle() {
    uint8_t cmd = COMM_GET_VALUES;
    sendPacket(&cmd, 1);

    uint16_t len;
    if (!receivePacket(rxBuffer, len)) return -1.0f;
    if (rxBuffer[0] != COMM_GET_VALUES) return -1.0f;

    uint8_t* ptr = &rxBuffer[1];
    ptr += 4 * 6;  // Skip to RPM
    ptr += 4;      // RPM
    ptr += 4;      // Input voltage. La tension de ta batterie

    // Lecture du dutyCycle (4 octets float)
    union {
        float f;
        uint8_t b[4];
    } uDuty;

    uDuty.b[0] = ptr[0];
    uDuty.b[1] = ptr[1];
    uDuty.b[2] = ptr[2];
    uDuty.b[3] = ptr[3];

    return uDuty.f;  // entre -1.0 et 1.0
}

void VESCInterface::sendPacket(uint8_t* data, uint16_t len) {
    uint8_t packet[128]; //trame finale à envoyer au VESC. (128 octets = largement suffisant pour n'importe quelle commande)
    uint16_t index = 0;

    packet[index++] = 2; //2 est le start bite. packet[0] = 0x02
    packet[index++] = len; //la longueur de la trame

    memcpy(&packet[index], data, len); //On copie le contenu de la trame dans packet
    index += len;

    uint16_t crc = crc16(data, len); //On calcule le CRC (vérification) sur le `payload`. C'est une sorte de signature numérique qui permt de s'assurer que les données n'ont pas été modifiés ors de l'envoi.
    packet[index++] = (crc >> 8) & 0xFF; //Le crc est sur deux octets donc en l'envoi en deux fois
    packet[index++] = crc & 0xFF;

    packet[index++] = 3;// Le bit de stop

    HAL_UART_Transmit(control_uart, packet, index, HAL_MAX_DELAY);
}

bool VESCInterface::receivePacket(uint8_t* buffer, uint16_t& len, uint32_t timeout) //On passe len en parametre parce qu'on veut la remplir et pouvoir l'utiliser plus tard
//Lit la réponse envoyée par le VESC via UART, et vérifie qu’elle est correcte.
{
    uint8_t header[2];
    if (HAL_UART_Receive(control_uart, header, 2, timeout) != HAL_OK) return false;

    if (header[0] != 2) return false;
    len = header[1];

    if (HAL_UART_Receive(control_uart, buffer, len + 3, timeout) != HAL_OK) return false;
    /*On lit les len + 3 octets suivants:
    - len octets de payload (les données utiles)
    - +2 octets de CRC
    - +1 octet de fin de trame (0x03)
    Si ça échoue : retour `false`.*/

    uint16_t received_crc = (buffer[len] << 8) | buffer[len + 1]; //On reconstitue le CRC reçu à partir des 2 derniers octets du payload
    if (crc16(buffer, len) != received_crc) return false;

    return true;
}

uint16_t VESCInterface::crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xFF) << 4) << 1;
    }
    return crc;
}
