#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <RadioLib.h>

// Глобальные переменные LoRa
extern STM32WLx_Module module;
extern STM32WLx radio;
extern volatile bool transmittedFlag;

// Функции LoRa
void initLoRa();
void sendPacket(const char* payload);
bool isLoRaAvailable();
String receivePacket();
void startReceiving();

#endif // LORA_MANAGER_H