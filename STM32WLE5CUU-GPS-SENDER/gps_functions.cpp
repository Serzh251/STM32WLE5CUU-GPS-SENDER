#include "gps_functions.h"
#include "config.h"
#include <Arduino.h>

// Объявляем отладку (будет определена в основном файле)
extern HardwareSerial Dbg;

// Определение глобальных переменных
HardwareSerial Gps(LPUART1);
TinyGPSPlus gps;
bool gpsValid = false;
int32_t gpsLat = 0;
int32_t gpsLon = 0;
uint8_t gpsSats = 0;
float gpsHdop = 99.99;

void initGps() {
  // 1. LPUART1: пины как в рабочем коде
  pinMode(GPS_TX_PIN, OUTPUT);
  pinMode(GPS_RX_PIN, INPUT);
  Gps.setTx(GPS_TX_PIN);
  Gps.setRx(GPS_RX_PIN);
  Gps.begin(9600);
  
  // 2. Питание PA9: точная последовательность из моста
  delay(200);
  pinMode(GPS_PWR_Pin, INPUT);
  delay(50);
  pinMode(GPS_PWR_Pin, OUTPUT);
  digitalWrite(GPS_PWR_Pin, LOW);  // LOW = включено
  
  Dbg.println("[GPS] Init OK (LPUART1, PA9=LOW)");
  Dbg.flush();
}

bool readGps() {
  unsigned long start = millis();
  bool gotFix = false;
  uint32_t byteCount = 0, dollarCount = 0;
  
  Dbg.println("[GPS] Reading...");
  Dbg.flush();
  
  while (millis() - start < (GPS_TIMEOUT_SEC * 1000UL)) {
    while (Gps.available()) {
      char c = Gps.read();
      byteCount++;
      
      // Считаем '$' для статистики
      if (c == '$') dollarCount++;
      
      // Выводим строки: буферизуем до '\n'
      static char line[128];
      static int idx = 0;
      if (c == '$') idx = 0;
      if (idx < 127) line[idx++] = c;
      if (c == '\n' && idx > 10) {
        line[idx] = '\0';
        Dbg.print("[RAW] ");
        Dbg.println(line);
        Dbg.flush();
        idx = 0;
      }
      
      // Парсинг
      if (gps.encode(c) && gps.location.isValid() && gps.location.isUpdated()) {
        gpsLat = (int32_t)(gps.location.lat() * 1e7);
        gpsLon = (int32_t)(gps.location.lng() * 1e7);
        gpsSats = gps.satellites.value();
        gpsHdop = gps.hdop.value() / 100.0f;
        gotFix = true;
      }
    }
  }
  
  Dbg.println();
  Dbg.print("Bytes: ");
  Dbg.print(byteCount);
  Dbg.print(" | '$': ");
  Dbg.println(dollarCount);
  
  if (dollarCount == 0) {
    Dbg.println("[!] No NMEA - check baud/wiring");
  }
  
  if (gotFix) {
    Dbg.println("[+] FIX:");
    Dbg.print("Lat: ");
    Dbg.println(gpsLat / 1e7, 7);
    Dbg.print("Lon: ");
    Dbg.println(gpsLon / 1e7, 7);
    Dbg.print("Sats: ");
    Dbg.println(gpsSats);
    Dbg.print("HDOP: ");
    Dbg.println(gpsHdop);
  } else {
    Dbg.println("[-] No fix");
    gpsLat = 0;
    gpsLon = 0;
    gpsSats = 0;
    gpsHdop = 99.99;
  }
  
  Dbg.flush();
  gpsValid = gotFix;
  return gotFix;
}