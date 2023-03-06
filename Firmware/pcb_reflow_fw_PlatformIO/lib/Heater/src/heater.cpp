#include <heater.h>
#include <debug.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SimpleKalmanFilter.h>

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
SimpleKalmanFilter tempKalmanFilter(1, 1, 0.01);
SimpleKalmanFilter voltKalmanFilter(1, 1, 0.01);

bool Heater::heat(void (*showHeatMenu)(float), buttons_state_t (*getButtonsState)(void),
                  void (*cancelledTimer)(void), void (*heatAnimate)(int, int, float, float, float),
                  byte max_temp, int profile_index)
{
  // Heating Display
  showHeatMenu(max_temp);
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
    if (getButtonsState() != BUTTONS_NO_PRESS)
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
      cancelledTimer();
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
    heatAnimate(x, y, v, t, target_temp);
  }
}

void Heater::evaluate_heat()
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

void Heater::setupSensors()
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

float Heater::getTemp()
{
  debugprint("Temps: ");
  float t = 0;
  for (byte i = 0; i < 100; i++)
  {
    t = tempKalmanFilter.updateEstimate(analogRead(TEMP_PIN));
  }

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

float Heater::getVolts()
{
  float v = 0;
  for (byte i = 0; i < 20; i++)
  { // Poll Voltage reading 20 times
    v = voltKalmanFilter.updateEstimate(analogRead(VCC_PIN));
  }
  float vin = (v / 1023.0) * 1.5;
  debugprint("voltage at term: ");
  debugprintln(vin);
  vin = (vin / 0.090981) + 0.3;
  return vin;
}

void Heater::stepPID(float target_temp, float current_temp, float last_temp, float dt,
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
