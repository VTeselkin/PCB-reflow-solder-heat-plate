#include <display.h>

Adafruit_SSD1306 led(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // Create Display

void Display::clearMainMenu()
{
  led.clearDisplay();
  led.setTextSize(1);
  led.drawRoundRect(0, 0, 83, 32, 2, SSD1306_WHITE);
}

void Display::getResistanceFromUser(Preference preference, buttons_state_t(*getButtonsState)(void))
{
  float resistance = 1.88;
  while (1)
  {
    clearMainMenu();
    led.setCursor(3, 4);
    led.print(F("Resistance"));
    led.drawLine(3, 12, 79, 12, SSD1306_WHITE);
    led.setCursor(3, 14);
    led.print(F("UP/DN: change"));
    led.setCursor(3, 22);
    led.print(F("BOTH: choose"));
    buttons_state_t button = getButtonsState();
    if (button == BUTTONS_UP_PRESS)
    {
      resistance += 0.01;
    }
    else if (button == BUTTONS_DN_PRESS)
    {
      resistance -= 0.01;
    }
    else if (button == BUTTONS_BOTH_PRESS)
    {
      preference.setResistance(resistance);
      return;
    }
    resistance = constrain(resistance, 0, MAX_RESISTANCE);

    led.setCursor(90, 12);
    led.print(resistance);
    led.display();
  }
}

void Display::showLogo(void (*doSetup)(void), buttons_state_t(*getButtonsState)(void))
{
  unsigned long start_time = millis();
  led.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  while (start_time + 2000 > millis())
  {
    led.clearDisplay();
    led.setTextSize(1);
    led.setTextColor(SSD1306_WHITE);
    led.setCursor(0, 0);
    led.drawBitmap(0, 0, logo, logo_width, logo_height, SSD1306_WHITE);
    led.setCursor(80, 16);
    led.print(F("S/W V"));
    led.print(sw, 1);
    led.setCursor(80, 24);
    led.print(F("H/W V"));
    led.print(hw, 1);
    led.display();
    // If we press both buttons during boot, we'll enter the setup process
    if (getButtonsState() == BUTTONS_BOTH_PRESS)
    {
      doSetup();
      return;
    }
  }
}

void Display::displayProfileRight(int8_t cur_profile)
{
  int cur_x = 90;
  int cur_y = 30;
  // start at x=90, go to SCREEN_WIDTH-8, save 6 pixels for cooldown
  float x_dist = SCREEN_WIDTH - 90 - 8;
  led.setCursor(cur_x, cur_y);
  float total_seconds =
      (int)profiles[cur_profile].seconds[profiles[cur_profile].points - 1];

  for (int i = 0; i < profiles[cur_profile].points; i++)
  {
    int x_next =
        (int)((profiles[cur_profile].seconds[i] / total_seconds) * x_dist) + 90;
    int y_next = 30 - (int)(profiles[cur_profile].fraction[i] * 28.0);
    led.drawLine(cur_x, cur_y, x_next, y_next, SSD1306_WHITE);
    cur_x = x_next;
    cur_y = y_next;
  }
  // draw down to finish TEMP_PIN
  led.drawLine(cur_x, cur_y, SCREEN_WIDTH - 2, 30, SSD1306_WHITE);
}

uint8_t Display::getProfile(buttons_state_t(*getButtonsState)(void))
{
  uint8_t cur_profile = 0;
  while (1)
  {
    clearMainMenu();
    led.setCursor(3, 4);
    led.print(F("Pick profile"));
    led.drawLine(3, 12, 79, 12, SSD1306_WHITE);
    led.setCursor(3, 14);
    led.print(F("UP:    cycle"));
    led.setCursor(3, 22);
    led.print(F("DOWN: choose"));
    buttons_state_t cur_button = getButtonsState();

    if (cur_button == BUTTONS_DN_PRESS)
    {
     clearMainMenu();
     return cur_profile;
    }
    else if (cur_button == BUTTONS_UP_PRESS)
    {
      cur_profile++;
    }
    cur_profile %= NUM_PROFILES;
    displayProfileRight(cur_profile);
    led.display();
  }
}

void Display::cancelledTimer(buttons_state_t(*getButtonsState)(void))
{ // Cancelled via 5 minute Time Limit
  // Initiate Swap Display
  int x = 0;   // Display change counter
  int y = 150; // Display change max (modulused below)

  // Wait to return on any button press
  while (getButtonsState() == BUTTONS_NO_PRESS)
  {
    // Update Display
    led.clearDisplay();
    led.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
    led.setCursor(25, 4);
    led.print(F("  TIMED OUT"));
    led.drawLine(25, 12, 103, 12, SSD1306_WHITE);

    // Swap Main Text
    if (x < (y * 0.3))
    {
      led.setCursor(25, 14);
      led.println(" Took longer");
      led.setCursor(25, 22);
      led.println(" than 5 mins");
    }
    else if (x < (y * 0.6))
    {
      led.setCursor(28, 14);
      led.println("Try a higher");
      led.setCursor(25, 22);
      led.println(" current PSU");
    }
    else
    {
      led.setCursor(25, 14);
      led.println(" Push button");
      led.setCursor(25, 22);
      led.println("  to return");
    }
    x = (x + 1) % y; // Display change increment and modulus

    led.setTextSize(3);
    led.setCursor(5, 4);
    led.print(F("!"));
    led.setTextSize(3);
    led.setCursor(108, 4);
    led.print(F("!"));
    led.setTextSize(1);
    led.display();
    delay(50);
  }
}

void Display::showHeatMenu(byte max_temp)
{
  led.clearDisplay();
  led.setTextSize(2);
  led.setCursor(22, 4);
  led.print(F("HEATING"));
  led.setTextSize(1);
  led.setCursor(52, 24);
  led.print(max_temp);
  led.print(F("C"));
  led.display();
}

void Display::heatAnimate(int &x, int &y, float v, float t, float target)
{
  // Heat Animate Control
  led.clearDisplay();
  led.drawBitmap(0, 3, heat_animate, heat_animate_width,
                 heat_animate_height, SSD1306_WHITE);
  led.drawBitmap(112, 3, heat_animate, heat_animate_width,
                 heat_animate_height, SSD1306_WHITE);
  led.fillRect(0, 3, heat_animate_width, heat_animate_height * (y - x) / y,
               SSD1306_BLACK);
  led.fillRect(112, 3, heat_animate_width,
               heat_animate_height * (y - x) / y, SSD1306_BLACK);
  x = (x + 1) % y; // Heat animate increment and modulus

  // Update display
  led.setTextSize(2);
  led.setCursor(22, 4);
  led.print(F("HEATING"));
  led.setTextSize(1);
  led.setCursor(20, 24);
  led.print(F("~"));
  led.print(v, 1);
  led.print(F("V"));
  if (t >= 100)
  {
    led.setCursor(63, 24);
  }
  else if (t >= 10)
  {
    led.setCursor(66, 24);
  }
  else
  {
    led.setCursor(69, 24);
  }
  led.print(F("~"));
  led.print(t, 0);
  led.print(F("C"));
  led.print(F("/"));
  led.print(target, 0);
  led.print(F("C"));
  led.display();
}

void Display::cancelledPB()
{ // Cancelled via push button
  // Update Display
  led.clearDisplay();
  //led.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
  led.setTextSize(2);
  led.setCursor(12, 4);
  led.print(F("CANCELLED"));
  led.setTextSize(1);
  led.display();
  delay(5000);
}

void Display::coolDown(float(*getTemp)(void), buttons_state_t(*getButtonsState)(void))
{
  float t = getTemp(); // Used to store current temperature

  // Wait to return on any button press, or TEMP_PIN below threshold
  while (getButtonsState() == BUTTONS_NO_PRESS && t > 45.00)
  {
    led.clearDisplay();
    led.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
    led.setCursor(25, 4);
    led.print(F("  COOL DOWN"));
    led.drawLine(25, 12, 103, 12, SSD1306_WHITE);
    led.setCursor(25, 14);
    led.println("  Still Hot");
    t = getTemp();
    if (t >= 100)
    {
      led.setCursor(49, 22);
    }
    else
    {
      led.setCursor(52, 22);
    }
    led.print(F("~"));
    led.print(t, 0);
    led.print(F("C"));
    led.setTextSize(3);
    led.setCursor(5, 4);
    led.print(F("!"));
    led.setTextSize(3);
    led.setCursor(108, 4);
    led.print(F("!"));
    led.setTextSize(1);
    led.display();
  }
}

void Display::completed(buttons_state_t(*getButtonsState)(void))
{
  // Update Display
  led.clearDisplay();
  led.drawRoundRect(22, 0, 84, 32, 2, SSD1306_WHITE);
  led.setCursor(25, 4);
  led.print(F("  COMPLETED  "));
  led.drawLine(25, 12, 103, 12, SSD1306_WHITE);
  led.setCursor(25, 14);
  led.println(" Push button");
  led.setCursor(25, 22);
  led.println("  to return");
  led.drawBitmap(0, 9, tick, tick_width, tick_height, SSD1306_WHITE);
  led.drawBitmap(112, 9, tick, tick_width, tick_height, SSD1306_WHITE);
  led.display();

  // Wait to return on any button press
  while (getButtonsState() == BUTTONS_NO_PRESS)
  {
  }
}

void Display::showMainMenuLeft(int &x, int &y)
{
  if (x < (y * 0.5))
  {
    led.setCursor(3, 4);
    led.print(F("PRESS: UP"));
    led.drawLine(3, 12, 79, 12, SSD1306_WHITE);
    led.setCursor(3, 14);
    led.print(F(" Change  MAX"));
    led.setCursor(3, 22);
    led.print(F(" Temperature"));
  }
  else
  {
    led.setCursor(3, 4);
    led.print(F("PRESS: DOWN"));
    led.drawLine(3, 12, 79, 12, SSD1306_WHITE);
    led.setCursor(3, 18);
    led.print(F("Begin Heating"));
  }
  x = (x + 1) % y; // Display change increment and modulus
}

void Display::showMainMenuRight(int index)
{
  led.setCursor(95, 6);
  led.print(F("TEMP"));
  led.setCursor(95, 18);
  led.print(max_temp_array[index]);
  led.print(F("C"));
  led.display();
}