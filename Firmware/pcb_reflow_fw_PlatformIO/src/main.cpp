/* Solder Reflow Plate Sketch
 *  H/W - Ver Spatz-1.0
 *  S/W - Ver 0.35
 *  by Chris Halsall and Nathan Heidt
 */

/*
 * TODOS:
 * - add digital sensor setup routine if they are detected, but not setup
 * - figure out a method for how to use all the temperature sensors
 * - implement an observer/predictor for the temperature sensors.  Kalman filter
 * time?!?
 */

#include <display.h>
#include <heater.h>
#include <controll.h>
#include <helpers.h>
#include <debug.h>
#include <preference.h>

#include <SimpleKalmanFilter.h>

Display display;
Controll controll;
Preference preference;
Heater heater;
Debug debug;

int profile_index = 0;

void mainMenu();
void doSetup();
float getTempCallBack();
void cancelledTimerCallBack();
buttons_state_t getButtonsStateCallBack();
void showHeatMenuCallBack(float max_temp);
void heatAnimateCallBack(int x, int y, float v, float t, float target);

void setup()
{
  // Pin Direction control
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(UPSW_PIN, INPUT);
  pinMode(DNSW_PIN, INPUT);
  pinMode(TEMP_PIN, INPUT);
  pinMode(VCC_PIN, INPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);

  digitalWrite(LED_GREEN_PIN, HIGH);
  analogWrite(MOSFET_PIN, 255); // VERY IMPORTANT, DONT CHANGE!

  attachInterrupt(DNSW_PIN, controll.dnsw_change_isr, FALLING);
  attachInterrupt(UPSW_PIN, controll.upsw_change_isr, FALLING);

  debug.setup();

  // Enable Fast PWM with no prescaler
  analogWriteFrequency(64);
  analogReference(INTERNAL1V5);

  // Start-up Diplay
  debugprintln("Showing startup");
  display.showLogo(doSetup, getButtonsStateCallBack);

  debugprintln("Checking sensors");
  // check onewire TEMP_PIN sensors
  heater.setupSensors();

  debugprintln("Checking first boot");
  if (preference.isFirstBoot() || !preference.validateCRC())
  {
    doSetup();
  }

  // Pull saved values from EEPROM
  max_temp_index = preference.getMaxTempIndex();
  bed_resistance = preference.getResistance();

  debugprintln("Entering main menu");
  // Go to main menu
  mainMenu();
}

void loop()
{
  // Not used
}

void mainMenu()
{
  // Debounce
  menu_state_t cur_state = MENU_IDLE;

  int x = 0;   // Display change counter
  int y = 200; // Display change max (modulused below)

  while (1)
  {
    switch (cur_state)
    {
    case MENU_IDLE:
    {
      display.clearMainMenu();
      buttons_state_t cur_button = controll.getButtonsState();

      if (cur_button == BUTTONS_BOTH_PRESS)
      {
        cur_state = MENU_SELECT_PROFILE;
      }
      else if (cur_button == BUTTONS_UP_PRESS)
      {
        cur_state = MENU_INC_TEMP;
      }
      else if (cur_button == BUTTONS_DN_PRESS)
      {
        cur_state = MENU_DEC_TEMP;
      }
    }
    break;
    case MENU_SELECT_PROFILE:
    {
      debugprintln("getting thermal profile");
      profile_index = display.getProfile(getButtonsStateCallBack);
      cur_state = MENU_HEAT;
    }
    break;
    case MENU_HEAT:
    {
      if (!heater.heat(showHeatMenuCallBack, getButtonsStateCallBack,
                       cancelledTimerCallBack, heatAnimateCallBack,
                       max_temp_array[max_temp_index], profile_index))
      {
        display.cancelledPB();
        display.coolDown(getTempCallBack, getButtonsStateCallBack);
      }
      else
      {
        display.coolDown(getTempCallBack, getButtonsStateCallBack);
        display.completed(getButtonsStateCallBack);
      }
      cur_state = MENU_IDLE;
    }
    break;
    case MENU_INC_TEMP:
    {
      if (max_temp_index < sizeof(max_temp_array) - 1)
      {
        max_temp_index++;
        debugprintln("incrementing max temp");
        preference.setMaxTempIndex(max_temp_index);
      }
      cur_state = MENU_IDLE;
    }
    break;
    case MENU_DEC_TEMP:
    {
      if (max_temp_index > 0)
      {
        max_temp_index--;
        debugprintln("decrementing max temp");
        preference.setMaxTempIndex(max_temp_index);
      }
      cur_state = MENU_IDLE;
    }
    break;
    }

    // Change Display (left-side)
    display.showMainMenuLeft(x, y);

    // Update Display (right-side)
    display.showMainMenuRight(max_temp_index);
  }
}

void doSetup()
{
  debugprintln("Performing setup");
  // TODO(HEIDT) show an info screen if we're doing firstime setup or if memory
  // is corrupted

  display.getResistanceFromUser(preference, getButtonsStateCallBack);
  // TODO(HEIDT) do a temperature module setup here

  preference.setFirstBoot();
}

//----------------------- call back region ---------------------------//

float getTempCallBack()
{
  return heater.getTemp();
}

buttons_state_t getButtonsStateCallBack()
{
  return controll.getButtonsState();
}

void showHeatMenuCallBack(float max_temp)
{
  display.showHeatMenu(max_temp);
}

void cancelledTimerCallBack()
{
  display.cancelledTimer(getButtonsStateCallBack);
}

void heatAnimateCallBack(int x, int y, float v, float t, float target)
{
  display.heatAnimate(x, y, v, t, target);
}

//------------------------------------------------------------------//
