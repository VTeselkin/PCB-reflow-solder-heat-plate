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

#include <Adafruit_GFX.h>
#include <display.h>
#include <controll.h>
#include <helpers.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <SPI.h>
#include <SimpleKalmanFilter.h>

Display display = Display();
Controll controll;



// Pin Definitions
#define MOSFET_PIN PIN_PC3
#define UPSW_PIN PIN_PF3
#define DNSW_PIN PIN_PD4
#define TEMP_PIN PIN_PF2 // A2
#define VCC_PIN PIN_PF4  // A0
#define LED_GREEN_PIN PIN_PC5
#define LED_RED_PIN PIN_PC4
#define ONE_WIRE_BUS PIN_PD5

#define MOSFET_PIN_OFF 255

enum menu_state_t
{
  MENU_IDLE,
  MENU_SELECT_PROFILE,
  MENU_HEAT,
  MENU_INC_TEMP,
  MENU_DEC_TEMP
};

float bed_resistance = 1.88;
#define MAX_AMPERAGE 5.0
#define PWM_VOLTAGE_SCALAR 2.0

// These values were derived using a regression from real world data.
// See the jupyter notebooks for more detail
#define ANALOG_APPROXIMATION_SCALAR 1.612
#define ANALOG_APPROXIMATION_OFFSET -20.517

// EEPROM storage locations
#define CRC_ADDR 0
#define FIRSTTIME_BOOT_ADDR 4
#define TEMP_INDEX_ADDR 5
#define RESISTANCE_INDEX_ADDR 6
#define DIGITAL_TEMP_ID_ADDR 10

// Voltage Measurement Info
#define VOLTAGE_REFERENCE 1.5

// temperature must be within this range to move on to next step
#define TARGET_TEMP_THRESHOLD 2.5

// PID values
float kI = 0.2;
float kD = 0.25;
float kP = 8.0;
float I_clip = 220;
float error_I = 0;

// Optional temperature sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int sensor_count = 0;
DeviceAddress temp_addresses[3];

#define DEBUG

#ifdef DEBUG
#define debugprint(x) Serial.print(x);
#define debugprintln(x) Serial.println(x);
#else
#define debugprint(x)
#define debugprintln(x)
#endif


void setCRC(uint32_t new_crc) { EEPROM.put(CRC_ADDR, new_crc); }

uint32_t eepromCRC(void)
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

void updateCRC()
{
  uint32_t new_crc = eepromCRC();
  setCRC(new_crc);
}

bool validateCRC()
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

inline void setupSensors()
{
  sensors.begin();
  sensor_count = sensors.getDeviceCount();
  debugprint("Looking for sensors, found: ");
  debugprintln(sensor_count);
  for (int i = 0; i < min(sensor_count, sizeof(temp_addresses)); i++)
  {
    sensors.getAddress(temp_addresses[i], i);
  }
}

inline void setFastPwm() { analogWriteFrequency(64); }

inline void setVREF() { analogReference(INTERNAL1V5); }

inline bool isFirstBoot()
{
  uint8_t first_boot = EEPROM.read(FIRSTTIME_BOOT_ADDR);
  debugprint("Got first boot flag: ");
  debugprintln(first_boot);
  return first_boot != 1;
}

inline void setFirstBoot()
{
  EEPROM.write(FIRSTTIME_BOOT_ADDR, 1);
  updateCRC();
}

inline float getResistance()
{
  float f;
  return EEPROM.get(RESISTANCE_INDEX_ADDR, f);
  return f;
}

inline void setResistance(float resistance)
{
  EEPROM.put(RESISTANCE_INDEX_ADDR, resistance);
  updateCRC();
}

inline void setMaxTempIndex(int index)
{
  EEPROM.update(TEMP_INDEX_ADDR, index);
  updateCRC();
}

inline int getMaxTempIndex(void)
{
  return EEPROM.read(TEMP_INDEX_ADDR) % sizeof(max_temp_array);
}

float getTemp()
{
  debugprint("Temps: ");
  float t = 0;
  for (byte i = 0; i < 100; i++)
  { // Poll TEMP_PIN reading 100 times
    t = t + analogRead(TEMP_PIN);
  }
  t /= 100.0;                      // average
  t *= VOLTAGE_REFERENCE / 1024.0; // voltage
  // conversion to temp, consult datasheet:
  // https://www.ti.com/document-viewer/LMT85/datasheet/detailed-description#snis1681040
  // this is optimized for 25C to 150C
  // TODO(HEIDT) this is linearized and innacurate, could probably use the
  // nonlinear functions without much overhead.
  t = (t - 1.365) / ((.301 - 1.365) / (150.0 - 25.0)) + 25.0;

  // The analog sensor is too far from the bed for an accurate reading
  // this simple function estimates the true bed temperature based off the
  // thermal gradient
  float estimated_temp =
      t * ANALOG_APPROXIMATION_SCALAR + ANALOG_APPROXIMATION_OFFSET;
  debugprint(estimated_temp);
  debugprint(" ");

  sensors.requestTemperatures();
  for (int i = 0; i < sensor_count; i++)
  {
    float temp_in = sensors.getTempC(temp_addresses[i]);
    debugprint(temp_in);
    debugprint(" ");
  }
  debugprintln();

  return max(t, estimated_temp);
}

float getVolts()
{
  float v = 0;
  for (byte i = 0; i < 20; i++)
  { // Poll Voltage reading 20 times
    v = v + analogRead(VCC_PIN);
  }
  v /= 20;

  float vin = (v / 1023.0) * 1.5;
  debugprint("voltage at term: ");
  debugprintln(vin);
  vin = (vin / 0.090981) + 0.3;
  return vin;
}



void stepPID(float target_temp, float current_temp, float last_temp, float dt,
             int min_pwm)
{
  float error = target_temp - current_temp;
  float D = (current_temp - last_temp) / dt;

  error_I += error * dt * kI;
  error_I = constrain(error_I, 0, I_clip);

  // PWM is inverted so 0 duty is 100% power
  float PWM = 255.0 - (error * kP + D * kD + error_I);
  PWM = constrain(PWM, min_pwm, 255);

  debugprintln("PID");
  debugprintln(dt);
  debugprintln(error);
  debugprintln(error_I);
  debugprint("PWM: ");
  debugprintln(PWM);
  analogWrite(MOSFET_PIN, (int)PWM);
}

bool heat(byte max_temp, int profile_index)
{
  // Heating Display
  display.showHeatMenu(max_temp);
  delay(3000);

  float t; // Used to store current temperature
  float v; // Used to store current voltage

  unsigned long profile_max_time = millis() / 1000 + (8 * 60);
  unsigned long step_start_time = (millis() / 1000);
  int current_step = 0;

  // Other control variables
  int x = 0;  // Heat Animate Counter
  int y = 80; // Heat Animate max (modulused below)

  float start_temp = getTemp();
  float goal_temp = profiles[profile_index].fraction[0] * max_temp;
  float step_runtime = profiles[profile_index].seconds[0];
  float last_time = 0;
  float last_temp = getTemp();
  error_I = 0;

  while (1)
  {
    // Cancel heat, don't even wait for uppress so we don't risk missing it
    // during the loop
    if (controll.getButtonsState() != BUTTONS_NO_PRESS)
    {
      analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
      debugprintln("cancelled");
      return 0;
    }

    // Check Heating not taken more than 8 minutes
    if (millis() / 1000 > profile_max_time)
    {
      analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
      debugprintln("exceeded time");
      display.cancelledTimer(controll);
      return 0;
    }

    // Measure Values
    // TODO(HEIDT) getting the temperature from the digital sensors is by far
    // the slowest part of this loop. figure out an approach that allows control
    // faster than sensing
    t = getTemp();
    v = getVolts();
    float max_possible_amperage = v / bed_resistance;
    // TODO(HEIDT) approximate true resistance based on cold resistance and
    // temperature
    float vmax = (MAX_AMPERAGE * bed_resistance) * PWM_VOLTAGE_SCALAR;
    int min_PWM = 255 - ((vmax * 255.0) / v);
    min_PWM = constrain(min_PWM, 0, 255);
    debugprint("Min PWM: ");
    debugprintln(min_PWM);
    debugprintln(bed_resistance);

    // Determine what target temp is and PID to it
    float time_into_step = ((float)millis() / 1000.0) - (float)step_start_time;
    float target_temp =
        min(((goal_temp - start_temp) * (time_into_step / step_runtime)) +
                start_temp,
            goal_temp);

    // TODO(HEIDT) PID for a ramp will always lag, other options may be better
    stepPID(target_temp, t, last_temp, time_into_step - last_time, min_PWM);
    last_time = time_into_step;

    // if we finish the step timewise
    if (time_into_step >= step_runtime)
    {
      // and if we're within the goal temperature of the step
      if (abs(t - goal_temp) < TARGET_TEMP_THRESHOLD)
      {
        // move onto the next step in the profile
        current_step++;
        // if that was the last step, we're done!
        if (current_step == profiles[profile_index].points)
        {
          analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
          return 1;
        }
        // otherwise, get the next goal temperature and runtime, and do the
        // process again
        last_time = 0.0;
        start_temp = t;
        goal_temp = profiles[profile_index].fraction[current_step] * max_temp;
        step_runtime = profiles[profile_index].seconds[current_step] -
                       profiles[profile_index].seconds[current_step - 1];
        step_start_time = millis() / 1000.0;
      }
    }

    display.heatAnimate(x, y, v, t, target_temp);
  }
}

void evaluate_heat()
{
  debugprintln("Starting thermal evaluation");
  uint8_t duties[] = {255, 225, 200, 150, 100, 50, 0};
  unsigned long runtime = 60 * 5; // run each for 5 minutes

  for (int i = 0; i < sizeof(duties); i++)
  {
    debugprint("Running to duty of: ");
    debugprintln(duties[i]);
    unsigned long start_time = millis();
    analogWrite(MOSFET_PIN, duties[i]);
    float elapsed_time = (millis() - start_time) / 1000.0;
    while (elapsed_time < runtime)
    {
      debugprint("elapsed time: ");
      debugprintln(elapsed_time);
      debugprint("runtime: ");
      debugprintln(runtime);
      elapsed_time = (millis() - start_time) / 1000.0;
      float v = getVolts();
      float t = getTemp();
      delay(500);
    }
  }

  analogWrite(MOSFET_PIN, MOSFET_PIN_OFF);
}

inline void mainMenu()
{
  // Debounce
  menu_state_t cur_state = MENU_IDLE;

  int x = 0;   // Display change counter
  int y = 200; // Display change max (modulused below)
  uint8_t profile_index = 0;

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
      profile_index = display.getProfile(controll);
      cur_state = MENU_HEAT;
    }
    break;
    case MENU_HEAT:
    {
      if (!heat(max_temp_array[max_temp_index], profile_index))
      {
        display.cancelledPB();
        display.coolDown(getTemp, controll);
      }
      else
      {
        display.coolDown(getTemp,controll);
        display.completed(controll);
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
        setMaxTempIndex(max_temp_index);
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
        setMaxTempIndex(max_temp_index);
      }
      cur_state = MENU_IDLE;
    }
    break;
    }

    // Change Display (left-side)
    display.showMainMenuLeft(x, y);

    // Update Display (right-side)
    display.showMainMenuRight();
  }
}

inline void doSetup()
{
  debugprintln("Performing setup");
  // TODO(HEIDT) show an info screen if we're doing firstime setup or if memory
  // is corrupted

  display.getResistanceFromUser(setResistance, controll);
  // TODO(HEIDT) do a temperature module setup here

  setFirstBoot();
}
// -------------------- Main Logic -----------------------------------

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

  Serial.begin(9600);
  
  // Enable Fast PWM with no prescaler
  setFastPwm();
  setVREF();

  // Start-up Diplay
  debugprintln("Showing startup");
  display.showLogo(doSetup, controll);

  debugprintln("Checking sensors");
  // check onewire TEMP_PIN sensors
  setupSensors();

  debugprintln("Checking first boot");
  if (isFirstBoot() || !validateCRC())
  {
    doSetup();
  }

  // Pull saved values from EEPROM
  max_temp_index = getMaxTempIndex();
  bed_resistance = getResistance();

  debugprintln("Entering main menu");
  // Go to main menu
  mainMenu();
}

void loop()
{
  // Not used
}

