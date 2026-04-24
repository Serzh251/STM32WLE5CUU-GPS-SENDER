#include <RadioLib.h>
#include <STM32LowPower.h>

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

#define RETRY_COUNT 3
#define ACK_TIMEOUT_MS 2000

int attempt = 0;
bool waitingForAck = false;
unsigned long ackWaitStart = 0;
int packetCounter = 0;
volatile bool transmittedFlag = false;

void setTransmitFlag(void) {
  transmittedFlag = true;
}

void sendPacket() {
  packetCounter++;
  
  // Отправка
  char buffer[16];
  snprintf(buffer, sizeof(buffer), "PKT%d", packetCounter);
  
  radio.startTransmit(buffer);
  
  // Ждем окончания передачи
  unsigned long start = millis();
  while (!transmittedFlag && (millis() - start < 1000));
  transmittedFlag = false;
  
  radio.finishTransmit();
  
  // Переходим в режим приема для получения ACK
  radio.startReceive();
  
  waitingForAck = true;
  ackWaitStart = millis();
}

void setup() {
  // Настройка RF переключателя
  radio.setRfSwitchTable(rfswitch_pins, rfswitch_table);
  
  // Инициализация LoRa
  radio.begin(868.1, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14);
  radio.setTCXO(1.7);
  radio.setDio1Action(setTransmitFlag);
  
  // Инициализация LowPower
  LowPower.begin();
  
  // Первая отправка
  sendPacket();
}

void loop() {
  // Проверка получения ACK
  if (radio.available() > 0) {
    String packet;
    radio.readData(packet);
    
    if (packet == "ACK") {
      // ACK получен - успех
      waitingForAck = false;
      attempt = 0;
      
      // Пауза 2 секунды
      delay(2000);
      
      // Следующая отправка
      sendPacket();
      return;
    }
  }
  
  // Проверка таймаута ACK
  if (waitingForAck && (millis() - ackWaitStart >= ACK_TIMEOUT_MS)) {
    waitingForAck = false;
    attempt++;
    
    if (attempt >= RETRY_COUNT) {
      // Все попытки неудачны - уходим в сон на 10 секунд
      // radio.sleep();  // Отключаем радио
      
      // Сон на 10 секунд
      // LowPower.deepSleep(10000);
      
      // Проснулись - переинициализируем радио
      radio.begin(868.1, 125.0, 12, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 14);
      radio.setTCXO(1.7);
      radio.setDio1Action(setTransmitFlag);
      
      attempt = 0;
      
      // Отправляем после пробуждения
      sendPacket();
    } else {
      // Повторная отправка через 3 секунды
      delay(3000);
      sendPacket();
    }
  }
  
  delay(10);
}