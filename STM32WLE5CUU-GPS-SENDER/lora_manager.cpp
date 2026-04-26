#include "lora_manager.h"
#include "config.h"
#include <Arduino.h>

extern HardwareSerial Dbg;

// Определение глобальных переменных
STM32WLx_Module module;
STM32WLx radio = STM32WLx(&module);
volatile bool transmittedFlag = false;

// RF Switch таблица
static const uint32_t rfswitch_pins[] = {PA6, PA7, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
  {STM32WLx::MODE_RX,    {LOW,  HIGH}},
  {STM32WLx::MODE_TX_LP, {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {HIGH, LOW}},
  END_OF_MODE_TABLE,
};

// Прерывание
void setTransmitFlag(void) { 
  transmittedFlag = true; 
}

void initLoRa() {
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  
  Dbg.print("LoRa init... ");
  int st = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING, 
                       LORA_CODING, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Dbg.print("FAIL Error: ");
    Dbg.println(st);
    Dbg.flush();
    while(1);
  }
  Dbg.println("OK");
  
  radio.setTCXO(1.7);
  radio.setDio1Action(setTransmitFlag);
  
  Dbg.println("LoRa ready");
  Dbg.flush();
}

void sendPacket(const char* payload) {
  Dbg.print("Sending: ");
  Dbg.println(payload);
  
  radio.startTransmit((uint8_t*)payload, strlen(payload));
  
  unsigned long start = millis();
  while (!transmittedFlag && millis() - start < 1000) {
    delay(1);
  }
  
  transmittedFlag = false;
  radio.finishTransmit();
  startReceiving();
}

bool isLoRaAvailable() {
  return radio.available() > 0;
}

String receivePacket() {
  String pkt;
  radio.readData(pkt);
  return pkt;
}

void startReceiving() {
  radio.startReceive();
}