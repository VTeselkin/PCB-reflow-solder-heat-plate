#pragma once
#include <Arduino.h>

// EEPROM storage locations
#define CRC_ADDR 0
#define FIRSTTIME_BOOT_ADDR 4
#define TEMP_INDEX_ADDR 5
#define RESISTANCE_INDEX_ADDR 6
#define DIGITAL_TEMP_ID_ADDR 10

class Preference
{

public:
    void setCRC(uint32_t new_crc);
    uint32_t eepromCRC();
    void updateCRC();
    bool validateCRC();
    bool isFirstBoot();
    void setFirstBoot();
    float getResistance();
    void setResistance(float resistance);
    void setMaxTempIndex(int index);
    int getMaxTempIndex(void);

private:
};