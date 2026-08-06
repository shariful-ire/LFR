#include "U8glib.h"
#include <EEPROM.h>

U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0 | U8G_I2C_OPT_NO_ACK | U8G_I2C_OPT_FAST);

// Motor Pins
#define right_motor_forward 6
#define right_motor_backward 4
#define left_motor_forward 3
#define left_motor_backward 2
#define right_motor_speed 9
#define left_motor_speed 5

// Buttons
#define BUTTON_UP_PIN 7
#define BUTTON_SELECT_PIN A2
#define BUTTON_DOWN_PIN A3

// LED
#define led 13

// MUX Pins
#define S0 8
#define S1 10
#define S2 11
#define S3 12
#define SIG_PIN A7

// Sensor Variables
#define sensorNumber 14
uint16_t sensorADC[sensorNumber];
uint8_t s[sensorNumber];
uint16_t bitWeight[sensorNumber] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192 };
const uint8_t WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140 };
uint8_t sumOnSensor;
uint16_t sensorWight;
uint16_t bitSensor;
uint16_t Max_ADC[sensorNumber];
uint16_t Min_ADC[sensorNumber];
uint16_t Reference_ADC[sensorNumber];

// PID Variables
float line_position;
float error;
float center_position = 75;
float previous_error;
uint8_t base_speed = 150;
int16_t right_motor_correction;
int16_t left_motor_correction;
float kp = 8;
float kd = 200;
int16_t position = 0;

// Turns Variable
uint8_t turnSpeed = 150;

// Inverse Parameter
bool inverseON = 0;

// OLED Related Parameters
const unsigned char upir_logo[] PROGMEM = {
  B00010101, B11010111,
  B00010101, B01000101,
  B00010101, B10010110,
  B00011001, B00010101
};

const unsigned char bitmap_scrollbar_background[] PROGMEM = {
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02,
  0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00
};

const unsigned char bitmap_item_sel_outline[] PROGMEM = {
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0,
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30,
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0,
  0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0
};

const int NUM_ITEMS = 9;
const int MAX_ITEM_LENGTH = 20;

char menu_items[NUM_ITEMS][MAX_ITEM_LENGTH] = {
  { "Start" },
  { "Calibration" },
  { "Analog Value" },
  { "Digital Value" },
  { "Setting" },
  { "Motor Test" },
  { "EMPTY" },
  { "EMPTY" },
  { "RESET SETTING" },
};

char menu_items2[NUM_ITEMS][MAX_ITEM_LENGTH] = {
  { "EMPTY" },
  { "KP" },
  { "KD" },
  { "EMPTY" },
  { "EMPTY" },
  { "EMPTY" },
  { "Back to Main" },
  { "EMPTY" },
  { "Speed" },
};

bool button_up_clicked = 0;
bool button_select_clicked = 0;
bool button_down_clicked = 0;
int8_t item_selected = 0;
int8_t item_sel_previous;
int8_t item_sel_next;
int8_t current_screen = 0;

//==================================== SETUP ====================================
void setup() {
  Serial.begin(9600);
  
  // Initialize pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);
  pinMode(led, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(right_motor_speed, OUTPUT);
  
  // Load settings from EEPROM
  loadSettings();
  
  // Show splash screen
  showSplashScreen();
  
  // Enter menu
  menu();
}

//==================================== LOOP ====================================
void loop() {
  // Main loop - now properly structured
  read_black_line();
  
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_profont12);
    u8g.drawStr(10, 15, "RoboTech Innovator");
    u8g.setFont(u8g_font_7x14B);
    u8g.setPrintPos(5, 60); u8g.print(s[0]);
    u8g.setPrintPos(15, 60); u8g.print(s[1]);
    u8g.setPrintPos(25, 60); u8g.print(s[2]);
    u8g.setPrintPos(20, 40); u8g.print(s[3]);
    u8g.setPrintPos(30, 40); u8g.print(s[4]);
    u8g.setPrintPos(40, 40); u8g.print(s[5]);
    u8g.setPrintPos(50, 40); u8g.print(s[6]);
    u8g.setPrintPos(60, 40); u8g.print(s[7]);
    u8g.setPrintPos(70, 40); u8g.print(s[8]);
    u8g.setPrintPos(80, 40); u8g.print(s[9]);
    u8g.setPrintPos(90, 40); u8g.print(s[10]);
    u8g.setPrintPos(100, 60); u8g.print(s[11]);
    u8g.setPrintPos(110, 60); u8g.print(s[12]);
    u8g.setPrintPos(120, 60); u8g.print(s[13]);
  } while (u8g.nextPage());
  
  // Check for menu button
  if (button(BUTTON_DOWN_PIN) == 1) {
    menu();
  }
}

//==================================== BUTTON HANDLER ====================================
uint8_t button(byte buttonNumber) {
  uint32_t t = 0;
  if (!digitalRead(buttonNumber)) {
    delay(50);
    while (!digitalRead(buttonNumber)) {
      delay(50);
      t += 50;
      if (t > 500) digitalWrite(led, 1);
    }
    digitalWrite(led, 0);
    if (t > 500) {
      return 2;  // Long press
    } else {
      return 1;  // Short press
    }
  }
  return 0;
}

//==================================== SPLASH SCREEN ====================================
void showSplashScreen() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(20, 30, "RoboTech");
    u8g.drawStr(10, 50, "Innovator");
    u8g.setFont(u8g_font_profont12);
    u8g.drawStr(30, 62, "Loading...");
  } while (u8g.nextPage());
  delay(1000);
}

//==================================== MENU SYSTEM ====================================
void menu() {
  while (1) {
    if (current_screen == 0) {
      if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) {
        item_selected = item_selected - 1;
        button_up_clicked = 1;
        if (item_selected < 0) {
          item_selected = NUM_ITEMS - 1;
        }
      } else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) {
        item_selected = item_selected + 1;
        button_down_clicked = 1;
        if (item_selected >= NUM_ITEMS) {
          item_selected = 0;
        }
      }
      
      if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) {
        button_up_clicked = 0;
      }
      if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) {
        button_down_clicked = 0;
      }
    }
    
    if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) {
      button_select_clicked = 1;
    }
    
    if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) {
      button_select_clicked = 0;
    }
    
    item_sel_previous = item_selected - 1;
    if (item_sel_previous < 0) {
      item_sel_previous = NUM_ITEMS - 1;
    }
    item_sel_next = item_selected + 1;
    if (item_sel_next >= NUM_ITEMS) {
      item_sel_next = 0;
    }
    
    // Menu item actions
    if (item_selected == 0 && button_select_clicked == 1) {
      current_screen = 0;
      startFollow();
    }
    if (item_selected == 1 && button_select_clicked == 1) {
      current_screen = 0;
      calibrateSensor();
    }
    if (item_selected == 2 && button_select_clicked == 1) {
      current_screen = 0;
      analog_value();
    }
    if (item_selected == 3 && button_select_clicked == 1) {
      current_screen = 0;
      digital_value();
    }
    if (item_selected == 4 && button_select_clicked == 1) {
      current_screen = 0;
      setting();
    }
    if (item_selected == 5 && button_select_clicked == 1) {
      current_screen = 0;
      motor_test();
    }
    if (item_selected == 6 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 7 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 8 && button_select_clicked == 1) {
      current_screen = 0;
      resetSettings();
    }
    
    u8g.firstPage();
    do {
      if (current_screen == 0) {
        u8g.drawBitmapP(0, 22, 128 / 8, 21, bitmap_item_sel_outline);
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15, menu_items[item_sel_previous]);
        u8g.setFont(u8g_font_7x14B);
        u8g.drawStr(25, 15 + 20 + 2, menu_items[item_selected]);
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15 + 20 + 20 + 2 + 2, menu_items[item_sel_next]);
        u8g.drawBitmapP(128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);
        u8g.drawBox(125, 64 / NUM_ITEMS * item_selected, 3, 64 / NUM_ITEMS);
      }
    } while (u8g.nextPage());
  }
}

//==================================== SETTINGS MENU ====================================
void setting() {
  while (1) {
    if (current_screen == 0) {
      if ((digitalRead(BUTTON_UP_PIN) == LOW) && (button_up_clicked == 0)) {
        item_selected = item_selected - 1;
        button_up_clicked = 1;
        if (item_selected < 0) {
          item_selected = NUM_ITEMS - 1;
        }
      } else if ((digitalRead(BUTTON_DOWN_PIN) == LOW) && (button_down_clicked == 0)) {
        item_selected = item_selected + 1;
        button_down_clicked = 1;
        if (item_selected >= NUM_ITEMS) {
          item_selected = 0;
        }
      }
      
      if ((digitalRead(BUTTON_UP_PIN) == HIGH) && (button_up_clicked == 1)) {
        button_up_clicked = 0;
      }
      if ((digitalRead(BUTTON_DOWN_PIN) == HIGH) && (button_down_clicked == 1)) {
        button_down_clicked = 0;
      }
    }
    
    if ((digitalRead(BUTTON_SELECT_PIN) == LOW) && (button_select_clicked == 0)) {
      button_select_clicked = 1;
    }
    
    if ((digitalRead(BUTTON_SELECT_PIN) == HIGH) && (button_select_clicked == 1)) {
      button_select_clicked = 0;
    }
    
    item_sel_previous = item_selected - 1;
    if (item_sel_previous < 0) {
      item_sel_previous = NUM_ITEMS - 1;
    }
    item_sel_next = item_selected + 1;
    if (item_sel_next >= NUM_ITEMS) {
      item_sel_next = 0;
    }
    
    if (item_selected == 0 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 1 && button_select_clicked == 1) {
      current_screen = 0;
      setKP();
    }
    if (item_selected == 2 && button_select_clicked == 1) {
      current_screen = 0;
      setKD();
    }
    if (item_selected == 3 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 4 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 5 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 6 && button_select_clicked == 1) {
      current_screen = 0;
      menu();
    }
    if (item_selected == 7 && button_select_clicked == 1) {
      current_screen = 0;
    }
    if (item_selected == 8 && button_select_clicked == 1) {
      current_screen = 0;
      setSpeed();
    }
    
    u8g.firstPage();
    do {
      if (current_screen == 0) {
        u8g.drawBitmapP(0, 22, 128 / 8, 21, bitmap_item_sel_outline);
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15, menu_items2[item_sel_previous]);
        u8g.setFont(u8g_font_7x14B);
        u8g.drawStr(25, 15 + 20 + 2, menu_items2[item_selected]);
        u8g.setFont(u8g_font_7x14);
        u8g.drawStr(25, 15 + 20 + 20 + 2 + 2, menu_items2[item_sel_next]);
        u8g.drawBitmapP(128 - 8, 0, 8 / 8, 64, bitmap_scrollbar_background);
        u8g.drawBox(125, 64 / NUM_ITEMS * item_selected, 3, 64 / NUM_ITEMS);
        u8g.drawBitmapP(128 - 16 - 4, 64 - 4, 16 / 8, 4, upir_logo);
      }
    } while (u8g.nextPage());
  }
}

//==================================== PID SETTINGS ====================================
void setSpeed() {
  while (1) {
    if (button(BUTTON_DOWN_PIN) == 1) base_speed = base_speed + 5;
    if (button(BUTTON_UP_PIN) == 1) base_speed = base_speed - 5;
    if (base_speed > 250) base_speed = 250;
    if (base_speed < 5) base_speed = 5;
    
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "Speed = ");
      u8g.setPrintPos(80, 30);
      u8g.print(base_speed);
    } while (u8g.nextPage());
    
    if (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(50, base_speed);
      delay(50);
      saveSettings();
      return setting();
    }
  }
}

void setKP() {
  while (1) {
    if (button(BUTTON_DOWN_PIN) == 1) kp = kp + 1;
    if (button(BUTTON_UP_PIN) == 1) kp = kp - 1;
    if (kp > 50) kp = 50;
    if (kp < 0) kp = 0;
    
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "KP = ");
      u8g.setPrintPos(80, 30);
      u8g.print(kp);
    } while (u8g.nextPage());
    
    if (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(51, (uint8_t)kp);
      delay(50);
      saveSettings();
      return setting();
    }
  }
}

void setKD() {
  while (1) {
    if (button(BUTTON_DOWN_PIN) == 1) kd = kd + 20;
    if (button(BUTTON_UP_PIN) == 1) kd = kd - 20;
    if (kd > 2500) kd = 2500;
    if (kd < 0) kd = 0;
    
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.drawStr(20, 30, "KD = ");
      u8g.setPrintPos(80, 30);
      u8g.print(kd);
    } while (u8g.nextPage());
    
    if (button(BUTTON_SELECT_PIN) == 2) {
      EEPROM.write(52, (uint8_t)(kd / 10));
      delay(50);
      saveSettings();
      return setting();
    }
  }
}

//==================================== SENSOR CALIBRATION ====================================
void calibrateSensor() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(10, 25, "Calibration");
    u8g.drawStr(5, 45, "Place on WHITE");
    u8g.drawStr(5, 60, "Press SELECT");
  } while (u8g.nextPage());
  
  while(digitalRead(BUTTON_SELECT_PIN) == HIGH);
  delay(100);
  
  // Calibrate white reference
  for(int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    uint32_t sum = 0;
    for(int j = 0; j < 20; j++) {
      sum += analogRead(SIG_PIN);
      delayMicroseconds(100);
    }
    Reference_ADC[i] = sum / 20;
  }
  
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(10, 25, "Calibration");
    u8g.drawStr(5, 45, "Place on BLACK");
    u8g.drawStr(5, 60, "Press SELECT");
  } while (u8g.nextPage());
  
  while(digitalRead(BUTTON_SELECT_PIN) == HIGH);
  delay(100);
  
  // Calibrate black reference
  for(int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    uint32_t sum = 0;
    for(int j = 0; j < 20; j++) {
      sum += analogRead(SIG_PIN);
      delayMicroseconds(100);
    }
    uint16_t blackValue = sum / 20;
    Reference_ADC[i] = (Reference_ADC[i] + blackValue) / 2;
  }
  
  // Save calibration
  saveCalibration();
  
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(20, 35, "Calibration");
    u8g.drawStr(15, 55, "Complete!");
  } while (u8g.nextPage());
  
  delay(1000);
  menu();
}

//==================================== LINE FOLLOWING ====================================
void startFollow() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(30, 35, "Line Follow");
    u8g.drawStr(40, 50, "Activated");
  } while (u8g.nextPage());
  delay(500);
  
  while (1) {
    read_black_line();
    
    if (sumOnSensor == 0) {
      // Lost line - search
      motor(base_speed, base_speed);
      delay(100);
      motor(turnSpeed, -turnSpeed);
      uint32_t timeout = millis() + 3000;
      while (!s[6] && !s[7] && millis() < timeout) {
        read_black_line();
      }
      if (millis() >= timeout) {
        // Still lost - stop and return to menu
        motor(0, 0);
        return menu();
      }
    } else if ((s[3] == 1 && s[10] == 1) && (bitSensor < 16383)) {
      // Toggle inverse mode at junction
      inverseON = !inverseON;
      digitalWrite(led, inverseON);
      delay(50);
      continue;
    }
    
    if (sumOnSensor > 0) {
      line_position = sensorWight / sumOnSensor;
    }
    
    error = center_position - line_position;
    right_motor_correction = base_speed + ((error * kp) + ((error - previous_error) * kd));
    left_motor_correction = base_speed - ((error * kp) + ((error - previous_error) * kd));
    previous_error = error;
    
    // Constrain motor values
    right_motor_correction = constrain(right_motor_correction, -255, 255);
    left_motor_correction = constrain(left_motor_correction, -255, 255);
    
    motor(left_motor_correction, right_motor_correction);
    
    // Check for stop button
    if (digitalRead(BUTTON_UP_PIN) == LOW) {
      motor(0, 0);
      delay(200);
      return menu();
    }
  }
}

//==================================== SENSOR READING ====================================
void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor = 0;
  
  for (int i = 0; i < sensorNumber; i++) {
    selectChannel(i);
    delayMicroseconds(50);
    sensorADC[i] = analogRead(SIG_PIN);
    
    if (sensorADC[i] > Reference_ADC[i]) {
      if (inverseON == 0) {
        s[i] = 1;
      } else {
        s[i] = 0;
      }
    } else {
      if (inverseON == 0) {
        s[i] = 0;
      } else {
        s[i] = 1;
      }
    }
    
    sumOnSensor += s[i];
    sensorWight += s[i] * WeightValue[i];
    bitSensor += s[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

void selectChannel(int channel) {
  digitalWrite(S0, bitRead(channel, 0));
  digitalWrite(S1, bitRead(channel, 1));
  digitalWrite(S2, bitRead(channel, 2));
  digitalWrite(S3, bitRead(channel, 3));
}

//==================================== DISPLAY FUNCTIONS ====================================
void analog_value() {
  while (1) {
    for (int channel = 0; channel < sensorNumber; channel++) {
      selectChannel(channel);
      sensorADC[channel] = analogRead(SIG_PIN);
    }
    
    u8g.setFont(u8g_font_profont12);
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 15);
      u8g.print(sensorADC[0]);
      u8g.setPrintPos(35, 15);
      u8g.print(sensorADC[1]);
      u8g.setPrintPos(65, 15);
      u8g.print(sensorADC[2]);
      
      u8g.setFont(u8g_font_profont12);
      u8g.setPrintPos(10, 30);
      u8g.print(sensorADC[3]);
      u8g.setPrintPos(40, 30);
      u8g.print(sensorADC[4]);
      u8g.setPrintPos(70, 30);
      u8g.print(sensorADC[5]);
      u8g.setPrintPos(100, 30);
      u8g.print(sensorADC[6]);
      
      u8g.setPrintPos(10, 45);
      u8g.print(sensorADC[7]);
      u8g.setPrintPos(40, 45);
      u8g.print(sensorADC[8]);
      u8g.setPrintPos(70, 45);
      u8g.print(sensorADC[9]);
      u8g.setPrintPos(100, 45);
      u8g.print(sensorADC[10]);
      
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(40, 60);
      u8g.print(sensorADC[11]);
      u8g.setPrintPos(70, 60);
      u8g.print(sensorADC[12]);
      u8g.setPrintPos(100, 60);
      u8g.print(sensorADC[13]);
    } while (u8g.nextPage());
    
    if (button(BUTTON_DOWN_PIN) == 1) {
      return menu();
    }
  }
}

void digital_value() {
  while (1) {
    read_black_line();
    u8g.firstPage();
    do {
      u8g.setFont(u8g_font_profont12);
      u8g.drawStr(30, 15, "SENSOR VIEW");
      
      u8g.setFont(u8g_font_7x14B);
      u8g.setPrintPos(5, 60);
      u8g.print(s[0]);
      u8g.setPrintPos(15, 60);
      u8g.print(s[1]);
      u8g.setPrintPos(25, 60);
      u8g.print(s[2]);
      
      u8g.setPrintPos(20, 40);
      u8g.print(s[3]);
      u8g.setPrintPos(30, 40);
      u8g.print(s[4]);
      u8g.setPrintPos(40, 40);
      u8g.print(s[5]);
      u8g.setPrintPos(50, 40);
      u8g.print(s[6]);
      u8g.setPrintPos(60, 40);
      u8g.print(s[7]);
      u8g.setPrintPos(70, 40);
      u8g.print(s[8]);
      u8g.setPrintPos(80, 40);
      u8g.print(s[9]);
      u8g.setPrintPos(90, 40);
      u8g.print(s[10]);
      
      u8g.setPrintPos(100, 60);
      u8g.print(s[11]);
      u8g.setPrintPos(110, 60);
      u8g.print(s[12]);
      u8g.setPrintPos(120, 60);
      u8g.print(s[13]);
    } while (u8g.nextPage());
    
    if (button(BUTTON_DOWN_PIN) == 1) {
      return menu();
    }
  }
}

//==================================== MOTOR CONTROL ====================================
void motor(int LPWM, int RPWM) {
  // Left motor direction
  if (LPWM > 0) {
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } else {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  }
  
  // Right motor direction
  if (RPWM > 0) {
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } else {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  }
  
  // Constrain and apply PWM
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);
  analogWrite(left_motor_speed, abs(LPWM));
  analogWrite(right_motor_speed, abs(RPWM));
}

void motor_test() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(5, 35, "Motor Testing...");
  } while (u8g.nextPage());
  
  // Forward
  motor(250, 250);
  digitalWrite(led, HIGH);
  delay(2000);
  
  // Stop
  motor(0, 0);
  digitalWrite(led, LOW);
  delay(500);
  
  // Backward
  motor(-250, -250);
  digitalWrite(led, HIGH);
  delay(2000);
  
  // Stop
  motor(0, 0);
  digitalWrite(led, LOW);
  delay(500);
  
  menu();
}

//==================================== EEPROM FUNCTIONS ====================================
void saveCalibration() {
  int address = 0;
  for (int i = 0; i < sensorNumber; i++) {
    EEPROM.write(address++, Reference_ADC[i] & 0xFF);
    EEPROM.write(address++, (Reference_ADC[i] >> 8) & 0xFF);
  }
}

void loadCalibration() {
  int address = 0;
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(address++) | (EEPROM.read(address++) << 8);
    if (Reference_ADC[i] == 0xFFFF || Reference_ADC[i] == 0) {
      // Set default calibration if not programmed
      Reference_ADC[i] = 512;
    }
  }
}

void saveSettings() {
  EEPROM.write(50, base_speed);
  EEPROM.write(51, (uint8_t)kp);
  EEPROM.write(52, (uint8_t)(kd / 10));
}

void loadSettings() {
  loadCalibration();
  
  base_speed = EEPROM.read(50);
  if (base_speed < 5 || base_speed > 250) base_speed = 150;
  
  kp = EEPROM.read(51);
  if (kp < 0 || kp > 50) kp = 8;
  
  kd = EEPROM.read(52) * 10;
  if (kd < 0 || kd > 2500) kd = 200;
}

void resetSettings() {
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(10, 30, "Reset Settings?");
    u8g.drawStr(5, 50, "Press SELECT");
    u8g.drawStr(5, 62, "to confirm");
  } while (u8g.nextPage());
  
  while(digitalRead(BUTTON_SELECT_PIN) == HIGH);
  delay(100);
  
  // Reset to defaults
  base_speed = 150;
  kp = 8;
  kd = 200;
  inverseON = 0;
  
  // Set default calibration
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = 512;
  }
  
  saveSettings();
  saveCalibration();
  
  u8g.firstPage();
  do {
    u8g.setFont(u8g_font_7x14B);
    u8g.drawStr(20, 35, "Settings");
    u8g.drawStr(15, 55, "Reset Complete!");
  } while (u8g.nextPage());
  
  delay(1000);
  menu();
}

//==================================== TEST FUNCTIONS ====================================
void test_PID() {
  while (1) {
    read_black_line();
    if (sumOnSensor > 0) line_position = sensorWight / sumOnSensor;
    error = center_position - line_position;
    
    Serial.print("Decimal: " + String(bitSensor));
    Serial.print(" | Binary: ");
    for (int i = (sensorNumber - 1); i >= 0; i--) {
      Serial.print(String(bitRead(bitSensor, i)));
    }
    
    right_motor_correction = base_speed + ((error * kp) + ((error - previous_error) * kd));
    left_motor_correction = base_speed - ((error * kp) + ((error - previous_error) * kd));
    previous_error = error;
    
    Serial.print(" | Error: " + String(error));
    Serial.print(" | LM: " + String(left_motor_correction));
    Serial.print(" RM: " + String(right_motor_correction));
    Serial.println();
    
    delay(50);
  }
}

void Bit_Sensor_Show() {
  read_black_line();
  Serial.print("Bit Sensor in Decimal: " + String(bitSensor));
  Serial.print(" | Bit Sensor in Binary: ");
  for (int i = (sensorNumber - 1); i >= 0; i--) {
    Serial.print(String(bitRead(bitSensor, i)));
  }
  Serial.println();
}

void sensor_value_test() {
  while (1) {
    read_black_line();
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(String(sensorADC[i]) + "  ");
    }
    Serial.println();
    delay(100);
  }
}