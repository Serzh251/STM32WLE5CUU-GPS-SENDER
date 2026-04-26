/*
 * E77-900M22S - LoRa + GPS (LPUART1) + отладка
 * Отправка, ожидание ACK, ретраи
 * Отладка: USART1 @ PB6(TX)/PB7(RX), 115200 бод
 * GPS: LPUART1 @ PA2(TX)/PA3(RX), 9600 бод
 * Питание GPS: PA9 (LOW = ON)
 */

#include <RadioLib.h>
#include <HardwareSerial.h>
#include "config.h"
#include "gps_functions.h"

// === UART ===
HardwareSerial Dbg(USART1);   // Отладка @ 115200

// === LoRa ===
STM32WLx_Module module;
STM32WLx radio = STM32WLx(&module);

// === RF Switch таблица ===
static const uint32_t rfswitch_pins[] = {PA6, PA7, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
  {STM32WLx::MODE_RX,    {LOW,  HIGH}},
  {STM32WLx::MODE_TX_LP, {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {HIGH, LOW}},
  END_OF_MODE_TABLE,
};

// === Глобальные ===
int attempt = 0;
bool waitingForAck = false;
unsigned long ackWaitStart = 0;
int packetCounter = 0;
volatile bool transmittedFlag = false;

// === Прерывание ===
void setTransmitFlag(void) { transmittedFlag = true; }

// === Инициализация отладки ===
void initDebug() {
  pinMode(DEBUG_TX_PIN, OUTPUT);
  pinMode(DEBUG_RX_PIN, INPUT);
  Dbg.setTx(DEBUG_TX_PIN);
  Dbg.setRx(DEBUG_RX_PIN);
  Dbg.begin(115200);
}

// === Отправка пакета ===
void sendPacket() {
  packetCounter++;
  Dbg.print("TX #");
  Dbg.println(packetCounter);
  
  char buf[32];
  if (gpsValid) {
    snprintf(buf, sizeof(buf), "PKT%d:%ld,%ld", packetCounter, gpsLat, gpsLon);
  } else {
    snprintf(buf, sizeof(buf), "PKT%d:NOFIX", packetCounter);
  }
  
  Dbg.print("Payload: ");
  Dbg.println(buf);
  
  radio.startTransmit((uint8_t*)buf, strlen(buf));
  unsigned long t = millis();
  while (!transmittedFlag && millis() - t < 1000) delay(1);
  transmittedFlag = false;
  radio.finishTransmit();
  radio.startReceive();
  
  waitingForAck = true;
  ackWaitStart = millis();
  Dbg.println("-> wait ACK");
  Dbg.flush();
}

// === Setup ===
void setup() {
  initDebug();
  Dbg.println("=== Start ===");
  Dbg.flush();
  
  initGps();           // Инициализация GPS
  gpsValid = readGps(); // Первое чтение GPS
  
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  Dbg.print("LoRa... ");
  int st = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING, 
                       LORA_CODING, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) {
    Dbg.print("FAIL ");
    Dbg.println(st);
    while(1);
  }
  Dbg.println("OK");
  
  radio.setTCXO(1.7);
  radio.setDio1Action(setTransmitFlag);
  
  Dbg.println("Ready");
  Dbg.flush();
  sendPacket();
}

// === Loop ===
void loop() {
  if (radio.available() > 0) {
    String pkt;
    radio.readData(pkt);
    Dbg.print("RX: ");
    Dbg.println(pkt);
    if (pkt == "ACK") {
      Dbg.println(">>> ACK!");
      waitingForAck = false;
      attempt = 0;
      delay(2000);
      sendPacket();
      return;
    }
  }
  
  if (waitingForAck && millis() - ackWaitStart >= ACK_TIMEOUT_MS) {
    Dbg.println("ACK timeout");
    waitingForAck = false;
    if (++attempt >= RETRY_COUNT) {
      Dbg.println(">>> Retries exhausted");
      Dbg.flush();
      delay(10000);
      gpsValid = readGps(); // Перечитываем GPS
      attempt = 0;
      sendPacket();
    } else {
      Dbg.print("Retry ");
      Dbg.print(attempt);
      Dbg.println("/3");
      Dbg.flush();
      delay(3000);
      sendPacket();
    }
  }
  delay(10);
}