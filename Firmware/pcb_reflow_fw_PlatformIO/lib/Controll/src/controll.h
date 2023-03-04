#pragma once
#include <helpers.h>

#define BUTTON_PRESS_TIME 50

class Controll
{

public:
  
  buttons_state_t getButtonsState();
  static void dnsw_change_isr();
  static void upsw_change_isr();

private:
};