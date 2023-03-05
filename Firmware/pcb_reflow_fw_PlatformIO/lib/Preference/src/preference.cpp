#include <preference.h>
#include <helpers.h>
#include <debug.h>
#include <EEPROM.h>

void Preference::setCRC(uint32_t new_crc) { EEPROM.put(CRC_ADDR, new_crc); }

uint32_t Preference::eepromCRC()
{
  static const uint32_t crc_table[16] = {
      0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4,
      0x4db26158, 0x5005713c, 0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
      0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};
  uint32_t crc = ~0L;
  // Skip first 4 bytes of EEPROM as thats where we store the CRC
  for (int index = 4; index < EEPROM.length(); ++index)
  {
    crc = crc_table[(crc ^ EEPROM[index]) & 0x0f] ^ (crc >> 4);
    crc = crc_table[(crc ^ (EEPROM[index] >> 4)) & 0x0f] ^ (crc >> 4);
    crc = ~crc;
  }

  return crc;
}

void Preference::updateCRC()
{
  uint32_t new_crc = eepromCRC();
  setCRC(new_crc);
}

bool Preference::validateCRC()
{
  uint32_t stored_crc;
  EEPROM.get(CRC_ADDR, stored_crc);
  uint32_t calculated_crc = eepromCRC();
  debugprint("got CRCs, stored: ");
  debugprint(stored_crc);
  debugprint(", calculated: ");
  debugprintln(calculated_crc);
  return stored_crc == calculated_crc;
}

bool Preference::isFirstBoot()
{
  uint8_t first_boot = EEPROM.read(FIRSTTIME_BOOT_ADDR);
  debugprint("Got first boot flag: ");
  debugprintln(first_boot);
  return first_boot != 1;
}

void Preference::setFirstBoot()
{
  EEPROM.write(FIRSTTIME_BOOT_ADDR, 1);
  updateCRC();
}

float Preference::getResistance()
{
  float f;
  return EEPROM.get(RESISTANCE_INDEX_ADDR, f);
  return f;
}

void Preference::setResistance(float resistance)
{
  EEPROM.put(RESISTANCE_INDEX_ADDR, resistance);
  updateCRC();
}

void Preference::setMaxTempIndex(int index)
{
  EEPROM.update(TEMP_INDEX_ADDR, index);
  updateCRC();
}

int Preference::getMaxTempIndex(void)
{
  return EEPROM.read(TEMP_INDEX_ADDR) % sizeof(max_temp_array);
}