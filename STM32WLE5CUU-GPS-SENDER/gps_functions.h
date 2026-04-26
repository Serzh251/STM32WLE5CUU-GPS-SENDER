#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Глобальные переменные GPS (как в исходном коде)
extern HardwareSerial Gps;
extern TinyGPSPlus gps;
extern bool gpsValid;
extern int32_t gpsLat;
extern int32_t gpsLon;
extern uint8_t gpsSats;
extern float gpsHdop;

// Функции GPS
void initGps();
bool readGps();

#endif // GPS_FUNCTIONS_H