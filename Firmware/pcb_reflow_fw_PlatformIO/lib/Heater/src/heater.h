#pragma once
#include <display.h>
#include <helpers.h>
#include <controll.h>

class Heater
{

public:
    bool heat(void (*showHeatMenu)(float), buttons_state_t (*getButtonsState)(void),
              void (*cancelledTimer)(void),void (*heatAnimate)(int, int, float, float, float),
               byte max_temp, int profile_index);
    void evaluate_heat();
    void setupSensors();
    float getTemp();
    float getVolts();
    void stepPID(float target_temp, float current_temp, float last_temp, float dt, int min_pwm);

private:
};