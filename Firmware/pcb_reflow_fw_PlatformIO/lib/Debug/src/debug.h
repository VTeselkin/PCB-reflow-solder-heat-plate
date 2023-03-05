#pragma once
#define DEBUG

#include <Arduino.h>

#ifdef DEBUG
#define debugprint(x) Serial.print(x);
#define debugprintln(x) Serial.println(x);
#else
#define debugprint(x)
#define debugprintln(x)
#endif

class Debug
{
public:
    void setup();

private:
};