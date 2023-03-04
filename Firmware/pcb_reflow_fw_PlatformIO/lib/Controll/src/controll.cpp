#include <Arduino.h>
#include <controll.h>

// Button interrupt state
volatile single_button_state_t up_button_state = BUTTON_NO_ACTION;
volatile single_button_state_t dn_button_state = BUTTON_NO_ACTION;

volatile unsigned long up_state_change_time = 0;
volatile unsigned long down_state_change_time = 0;

buttons_state_t Controll::getButtonsState()
{
  single_button_state_t button_dn;
  single_button_state_t button_up;
  unsigned long button_dn_time;
  unsigned long button_up_time;

  noInterrupts();
  button_dn = dn_button_state;
  button_up = up_button_state;
  button_dn_time = down_state_change_time;
  button_up_time = up_state_change_time;
  interrupts();

  unsigned long cur_time = millis();
  buttons_state_t state = BUTTONS_NO_PRESS;

  if (button_dn == BUTTON_PRESSED && button_up == BUTTON_PRESSED &&
      abs(button_dn_time - button_up_time) < BUTTON_PRESS_TIME)
  {
    if (cur_time - button_dn_time > BUTTON_PRESS_TIME &&
        cur_time - button_up_time > BUTTON_PRESS_TIME)
    {
      state = BUTTONS_BOTH_PRESS;
      noInterrupts();
      dn_button_state = BUTTON_NO_ACTION;
      up_button_state = BUTTON_NO_ACTION;
      interrupts();
    }
  }
  else if (button_up == BUTTON_PRESSED &&
           cur_time - button_up_time > BUTTON_PRESS_TIME)
  {
    state = BUTTONS_UP_PRESS;
    noInterrupts();
    up_button_state = BUTTON_NO_ACTION;
    interrupts();
  }
  else if (button_dn == BUTTON_PRESSED &&
           cur_time - button_dn_time > BUTTON_PRESS_TIME)
  {
    state = BUTTONS_DN_PRESS;
    noInterrupts();
    dn_button_state = BUTTON_NO_ACTION;
    interrupts();
  }

  return state;
}

void Controll::dnsw_change_isr()
{
  dn_button_state = BUTTON_PRESSED;
  down_state_change_time = millis();
}

void Controll::upsw_change_isr()
{
  up_button_state = BUTTON_PRESSED;
  up_state_change_time = millis();
}