/*
 * E77-900M22S - LoRa + GPS (LPUART1) + отладка
 * Отправка, ожидание ACK, ретраи
 * Отладка: USART1 @ PB6(TX)/PB7(RX), 115200 бод
 * GPS: LPUART1 @ PA2(TX)/PA3(RX), 9600 бод
 * Питание GPS: PA9 (LOW = ON) — инициализация как в рабочем мосте
 * Сон: ОТКЛЮЧЁН
 */

#include <RadioLib.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// === LoRa настройки ===
#define LORA_FREQUENCY      915.0f
#define LORA_BANDWIDTH      125.0f
#define LORA_SPREADING      7
#define LORA_CODING         5
#define LORA_PREAMBLE       8
#define LORA_TX_POWER       22
#define LORA_SYNC_WORD      0x14

// === UART ===
HardwareSerial Dbg(USART1);   // Отладка @ 115200
HardwareSerial Gps(LPUART1);  // GPS @ 9600 (PA2/PA3)
TinyGPSPlus gps;

// === Пины ===
#define GPS_PWR_Pin PA9  // LOW = включено

// === LoRa ===
STM32WLx_Module module;
STM32WLx radio = STM32WLx(&module);

static const uint32_t rfswitch_pins[] = {PA6, PA7, RADIOLIB_NC, RADIOLIB_NC, RADIOLIB_NC};
static const Module::RfSwitchMode_t rfswitch_table[] = {
  {STM32WLx::MODE_IDLE,  {LOW,  LOW}},
  {STM32WLx::MODE_RX,    {LOW,  HIGH}},
  {STM32WLx::MODE_TX_LP, {HIGH, LOW}},
  {STM32WLx::MODE_TX_HP, {HIGH, LOW}},
  END_OF_MODE_TABLE,
};

// === Протокол ===
#define RETRY_COUNT 3
#define ACK_TIMEOUT_MS 2000
#define GPS_TIMEOUT_SEC 10

// === Глобальные ===
int attempt = 0;
bool waitingForAck = false;
unsigned long ackWaitStart = 0;
int packetCounter = 0;
volatile bool transmittedFlag = false;
bool gpsValid = false;
int32_t gpsLat = 0, gpsLon = 0;
uint8_t gpsSats = 0;
float gpsHdop = 99.99;

// === Прерывание ===
void setTransmitFlag(void) { transmittedFlag = true; }

// === Инициализация отладки ===
void initDebug() {
  pinMode(PB6, OUTPUT); pinMode(PB7, INPUT);
  Dbg.setTx(PB6); Dbg.setRx(PB7);
  Dbg.begin(115200);
}

// === Инициализация GPS (ТОЧНО как в рабочем мосте) ===
void initGps() {
  // 1. LPUART1: пины как в мосте
  pinMode(PA2, OUTPUT); pinMode(PA3, INPUT);  // <-- как в рабочем коде
  Gps.setTx(PA2);   // <-- как в рабочем коде
  Gps.setRx(PA3);   // <-- как в рабочем коде
  Gps.begin(9600);
  
  // 2. Питание PA9: ТОЧНАЯ последовательность из моста
  delay(200);              // стабилизация
  pinMode(GPS_PWR_Pin, INPUT);  // отпустить линию
  delay(50);
  pinMode(GPS_PWR_Pin, OUTPUT);
  digitalWrite(GPS_PWR_Pin, LOW);  // LOW = включено (active-low)
  
  Dbg.println("[GPS] Init OK (LPUART1, PA9=LOW)");
  Dbg.flush();
}

// === Чтение GPS ===
// === Чтение GPS: упрощённый вывод + парсинг ===
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
      if (c == '$') idx = 0;  // начало новой строки
      if (idx < 127) line[idx++] = c;
      if (c == '\n' && idx > 10) {  // мин. длина строки
        line[idx] = '\0';
        Dbg.print("[RAW] "); Dbg.println(line);
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
  Dbg.print("Bytes: "); Dbg.print(byteCount);
  Dbg.print(" | '$': "); Dbg.println(dollarCount);
  
  if (dollarCount == 0) Dbg.println("[!] No NMEA - check baud/wiring");
  if (gotFix) {
    Dbg.println("[+] FIX:");
    Dbg.print("Lat: "); Dbg.println(gpsLat / 1e7, 7);
    Dbg.print("Lon: "); Dbg.println(gpsLon / 1e7, 7);
  } else {
    Dbg.println("[-] No fix");
    gpsLat = 0; gpsLon = 0; gpsSats = 0; gpsHdop = 99.99;
  }
  Dbg.flush();
  gpsValid = gotFix;
  return gotFix;
}

// === Отправка пакета ===
void sendPacket() {
  packetCounter++;
  Dbg.print("TX #"); Dbg.println(packetCounter);
  
  char buf[32];
  if (gpsValid) snprintf(buf, sizeof(buf), "PKT%d:%ld,%ld", packetCounter, gpsLat, gpsLon);
  else snprintf(buf, sizeof(buf), "PKT%d:NOFIX", packetCounter);
  
  Dbg.print("Payload: "); Dbg.println(buf);
  
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
  Dbg.println("=== Start ==="); Dbg.flush();
  
  initGps();           // GPS + питание PA9 (как в мосте)
  gpsValid = readGps();
  
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  Dbg.print("LoRa... ");
  int st = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING, 
                       LORA_CODING, LORA_SYNC_WORD, LORA_TX_POWER, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) { Dbg.print("FAIL "); Dbg.println(st); while(1); }
  Dbg.println("OK");
  
  radio.setTCXO(1.7);
  radio.setDio1Action(setTransmitFlag);
  
  Dbg.println("Ready"); Dbg.flush();
  sendPacket();
}

// === Loop ===
void loop() {
  if (radio.available() > 0) {
    String pkt; radio.readData(pkt);
    Dbg.print("RX: "); Dbg.println(pkt);
    if (pkt == "ACK") {
      Dbg.println(">>> ACK!");
      waitingForAck = false; attempt = 0;
      delay(2000); sendPacket(); return;
    }
  }
  
  if (waitingForAck && millis() - ackWaitStart >= ACK_TIMEOUT_MS) {
    Dbg.println("ACK timeout");
    waitingForAck = false;
    if (++attempt >= RETRY_COUNT) {
      Dbg.println(">>> Retries exhausted");
      Dbg.flush(); delay(10000);
      gpsValid = readGps(); attempt = 0; sendPacket();
    } else {
      Dbg.print("Retry "); Dbg.print(attempt); Dbg.println("/3");
      Dbg.flush(); delay(3000); sendPacket();
    }
  }
  delay(10);
}