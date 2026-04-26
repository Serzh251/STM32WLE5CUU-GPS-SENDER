#ifndef CONFIG_H
#define CONFIG_H

// === LoRa настройки ===
#define LORA_FREQUENCY      915.0f
#define LORA_BANDWIDTH      125.0f
#define LORA_SPREADING      7
#define LORA_CODING         5
#define LORA_PREAMBLE       8
#define LORA_TX_POWER       22
#define LORA_SYNC_WORD      0x14

// === Пины ===
#define GPS_PWR_Pin         PA9     // LOW = включено
#define DEBUG_TX_PIN        PB6
#define DEBUG_RX_PIN        PB7
#define GPS_TX_PIN          PA2
#define GPS_RX_PIN          PA3

// === Протокол ===
#define RETRY_COUNT         3
#define ACK_TIMEOUT_MS      2000
#define GPS_TIMEOUT_SEC     10

#endif // CONFIG_H