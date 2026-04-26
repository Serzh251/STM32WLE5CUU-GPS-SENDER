/*
 * E77-900M22S - LoRa + GPS (LPUART1) + отладка
 * Отправка, ожидание ACK, ретраи
 */

#include <HardwareSerial.h>
#include "config.h"
#include "lora_manager.h"
#include "gps_functions.h"

// === UART отладки ===
HardwareSerial Dbg(USART1);

// === Глобальные переменные протокола ===
int attempt = 0;
bool waitingForAck = false;
unsigned long ackWaitStart = 0;
int packetCounter = 0;

// === Инициализация отладки ===
void initDebug() {
  pinMode(DEBUG_TX_PIN, OUTPUT);
  pinMode(DEBUG_RX_PIN, INPUT);
  Dbg.setTx(DEBUG_TX_PIN);
  Dbg.setRx(DEBUG_RX_PIN);
  Dbg.begin(115200);
}

// === Формирование и отправка пакета ===
void sendDataPacket() {
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
  
  sendPacket(buf);
  
  waitingForAck = true;
  ackWaitStart = millis();
  Dbg.println("-> wait ACK");
  Dbg.flush();
}

// === Setup ===
void setup() {
  initDebug();
  Dbg.println("=== System Start ===");
  Dbg.flush();
  
  initGps();
  gpsValid = readGps();
  
  initLoRa();
  
  Dbg.println("Ready");
  Dbg.flush();
  
  sendDataPacket();
}

// === Loop ===
void loop() {
  // Проверка входящих сообщений
  if (isLoRaAvailable()) {
    String pkt = receivePacket();
    Dbg.print("RX: ");
    Dbg.println(pkt);
    
    if (pkt == "ACK") {
      Dbg.println(">>> ACK received!");
      waitingForAck = false;
      attempt = 0;
      delay(2000);
      sendDataPacket();
      return;
    }
  }
  
  // Обработка таймаута ACK
  if (waitingForAck && millis() - ackWaitStart >= ACK_TIMEOUT_MS) {
    Dbg.println("ACK timeout");
    waitingForAck = false;
    
    if (++attempt >= RETRY_COUNT) {
      Dbg.println(">>> Retries exhausted, re-reading GPS");
      Dbg.flush();
      delay(10000);
      
      gpsValid = readGps();
      attempt = 0;
      sendDataPacket();
    } else {
      Dbg.print("Retry ");
      Dbg.print(attempt);
      Dbg.println("/3");
      Dbg.flush();
      delay(3000);
      sendDataPacket();
    }
  }
  
  delay(10);
}