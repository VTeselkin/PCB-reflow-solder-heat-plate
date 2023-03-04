#pragma once
#include <Arduino.h>

#define NUM_PROFILES 2

// This needs to be specified or the compiler will fail as you can't initialize
// a flexible array member in a nested context
#define MAX_PROFILE_LENGTH 8

// Version Definitions
static const PROGMEM float hw = 0.9;
static const PROGMEM float sw = 0.15;

// TODO(HEIDT) may need to switch away from floats for speed/sizeA
struct solder_profile_t
{
  uint8_t points;
  float seconds[MAX_PROFILE_LENGTH];
  float fraction[MAX_PROFILE_LENGTH];
};

// TODO(HEIDT) how to adjust for environments where the board starts hot or
// cold? profiles pulled from here:
// https://www.compuphase.com/electronics/reflowsolderprofiles.htm#_
const static solder_profile_t profiles[NUM_PROFILES] = {
    {.points = 4,
     .seconds = {90, 180, 240, 260},
     .fraction = {.65, .78, 1.00, 1.00}},
    {.points = 2, .seconds = {162.0, 202.0}, .fraction = {.95, 1.00}}};

// Temperature Info
static byte max_temp_array[] = {140, 150, 160, 170, 180};
static byte max_temp_index = 0;

enum buttons_state_t
{
  BUTTONS_NO_PRESS,
  BUTTONS_BOTH_PRESS,
  BUTTONS_UP_PRESS,
  BUTTONS_DN_PRESS
};
enum single_button_state_t
{
  BUTTON_PRESSED,
  BUTTON_RELEASED,
  BUTTON_NO_ACTION
};



class Helpers
{
public:
private:
};

#ifdef Data
#undef Data 
#endif

extern Helpers Data;