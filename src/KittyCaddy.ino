#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <SpeedyStepper.h>
#include <RTClib.h>

#define FEED_HRS_EEPROM_ADDR 0x10
#define FEED_MIN_EEPROM_ADDR 0x11

#define FIELD_NONE 0
#define FIELD_TOD_HRS 1
#define FIELD_TOD_MIN 2
#define FIELD_TOD_SEC 3
#define FIELD_FEED_HRS 4
#define FIELD_FEED_MIN 5

#define MSG_DEFAULT  "   Ready to feed"
#define MSG_TOD_HRS  "  Set clock hour"
#define MSG_TOD_MIN  " Set clock minute"
#define MSG_TOD_SEC  "Reset clock seconds"
#define MSG_FEED_HRS " Set feeding hour"
#define MSG_FEED_MIN "Set feeding minute"

#define INPUT_INTERVAL 200 // ms between inputs
#define INPUT_TIMEOUT 10000 // ms before state restores to default
#define REPEAT_PERIOD 100 // ms before inputs autorepeat

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define FEED_TIME_CURSORX 90
#define FEED_TIME_CURSORY 0
#define TOD_CURSORX 20
#define TOD_CURSORY 20 // Cursor position for time of day (TOD)
#define MSG_CURSORX 10
#define MSG_CURSORY 56


#define HOMING_THRESHOLD 500 // Threshold for homing sensor, below this value means the sensor sees the mark
#define STEPS_PER_TRAY 33500 // Steps per tray movement

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; either 0x3D or 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Declaration for a DRV8825 stepper driver
SpeedyStepper stepper;

// Declaration for DS3231 Real time clock
RTC_DS3231 rtc;

const byte AlarmPin = 3; // interrupt from RTC

// Buttons
const byte UpButtonPin = 0; 
const byte MidButtonPin = 1;
const byte DownButtonPin = 2;

const byte HomingPin = 4; // Homing sensor
// const byte LED_PIN = 13; // builtin LED

// Motor pins
const byte DirectionPin = 10;
const byte StepPin = 9;
// const byte SleepPin = 8; 
// const byte ResetPin = 9; 
const byte M2Pin = 8;
const byte M1Pin = 7;
const byte M0Pin = 6;
const byte EnablePin = 5;

bool shouldFeed = false;
bool UpWasPressed = false;
bool MidWasPressed = false;
bool DownWasPressed = false;

unsigned long input_time;
unsigned long repeat_time;

DateTime now;
DateTime feedingTime;
unsigned int active_field = FIELD_NONE;

char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
char ss_str[3];
char mm_str[3];
char hh_str[3];

void setup() {
  Serial.begin(115200);
  // IO Setup
  pinMode(AlarmPin, INPUT_PULLUP);
  pinMode(UpButtonPin, INPUT_PULLUP);
  pinMode(MidButtonPin, INPUT_PULLUP);
  pinMode(DownButtonPin, INPUT_PULLUP);
  // pinMode(LED_PIN, OUTPUT); // Builtin LED

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Clear the display buffer
  display.clearDisplay();


  // TODO: Display startup logo

  // Motor setup 
  stepper.connectToPins(StepPin, DirectionPin);
  pinMode(EnablePin, OUTPUT); digitalWrite(EnablePin, LOW);
  // pinMode(M0Pin, OUTPUT); digitalWrite(SleepPin, HIGH);
  // pinMode(M0Pin, OUTPUT); digitalWrite(ResetPin, HIGH);
  pinMode(M0Pin, OUTPUT); digitalWrite(M0Pin, HIGH);
  pinMode(M1Pin, OUTPUT); digitalWrite(M1Pin, HIGH);
  pinMode(M2Pin, OUTPUT); digitalWrite(M2Pin, HIGH);
  // stepper.begin();
  // stepper.moveHome();

  stepper.setSpeedInStepsPerSecond(6400);

  // homing before input handlers init
  HomeTray();
  stepper.setAccelerationInStepsPerSecondPerSecond(5000);

  // init_internal_RTC(); // internal rtc for 1Hz clock
  init_RTC(); // external rtc for date/time
  feedingTime = LoadFeedingTime();
  Serial.print("Feeding Time: ");
  Serial.print(feedingTime.hour(),DEC);
  Serial.print(":");
  Serial.print(feedingTime.minute(),DEC);

  // Interrupt when RTC alarm
  attachInterrupt(digitalPinToInterrupt(AlarmPin), AlarmRecorder, FALLING); 
  // Interrupt when buttons pressed
  // attachInterrupt(digitalPinToInterrupt(UpButtonPin), UpButtonRecorder, FALLING);
  // attachInterrupt(digitalPinToInterrupt(MidButtonPin), MidButtonRecorder, FALLING);
  // attachInterrupt(digitalPinToInterrupt(DownButtonPin), DownButtonRecorder, FALLING);
  
  input_time = millis(); // init debounce
  // repeat_time = millis();



}

void init_RTC(){ 
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    SetFeedingTime(DateTime(F(__DATE__), F(__TIME__)));

  }

  // disable 32K pin
  rtc.disable32K();
  // disable square wave pin
  rtc.writeSqwPinMode(DS3231_OFF);
  // clear any alarm flags
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  // disable second alarm
  rtc.disableAlarm(2);
}

bool HomeTray(){
  stepper.setAccelerationInStepsPerSecondPerSecond(1000000);
  delay(100); // let measurements settle
  if(GetHomingBool()) return true;

  digitalWrite(EnablePin, LOW);
  long homeStart = 0;
  long homeEnd = 0;

  // move to the start of the trigger zone while watching homing sensor
  stepper.setupRelativeMoveInSteps(2*STEPS_PER_TRAY);
  while(!stepper.motionComplete() && !GetHomingBool()) stepper.processMovement();
  if (GetHomingBool()) {
    homeStart = stepper.getCurrentPositionInSteps();
  }
  // move past trigger
  delay(50);
  while(GetHomingBool()) {
    stepper.moveRelativeInSteps(500);
  }
  // find end of trigger zone
  delay(50);
  while(!GetHomingBool()) {
    stepper.moveRelativeInSteps(-500);
    homeEnd = stepper.getCurrentPositionInSteps();
  }
  delay(50);
  stepper.moveToPositionInSteps((homeStart + homeEnd)/2);
  stepper.setCurrentPositionInSteps(0L);
  digitalWrite(EnablePin, HIGH);
}



void loop() {
  now = rtc.now();

  if (!digitalRead(UpButtonPin)) UpButtonRecorder();
  if (!digitalRead(MidButtonPin)) MidButtonRecorder();
  if (!digitalRead(DownButtonPin)) DownButtonRecorder();

  if (UpWasPressed) UpButtonHandler();
  if (MidWasPressed) MidButtonHandler();
  if (DownWasPressed) DownButtonHandler();

  if ( millis() - repeat_time > REPEAT_PERIOD ) {
    // int rpt_cnt = (millis() - input_time > REPEAT_PERIOD * 5) ? 10 : 1;
    // for(int i = 0; i < rpt_cnt; i++ ) {
      if (!digitalRead(UpButtonPin)) {
        UpButtonHandler();
        repeat_time = millis();
      }
      else if (!digitalRead(DownButtonPin)) {
        DownButtonHandler();
        repeat_time = millis();
      }
    // }
    
  }
  
  if (millis() - repeat_time > INPUT_TIMEOUT) active_field = FIELD_NONE;

  if (shouldFeed) {
    DisplayFeedingScreen();
    RotateTray(1);
    shouldFeed = false;
  } else {
     digitalWrite(EnablePin, HIGH);
  }
  UpdateDisplay();
  
}

void UpdateDisplay(){
  display.clearDisplay();
  bool flash_flag = active_field > 0 && active_field > FIELD_TOD_SEC;
   // Display feeding time
  display.setCursor(10, FEED_TIME_CURSORY);
  display.print(F("Feeding Time: "));
  DisplayTime(GetFeedHour(), GetFeedMinute(), 0, 1, FEED_TIME_CURSORX, FEED_TIME_CURSORY, false, flash_flag, active_field);

  display.drawLine(0,TOD_CURSORY-5,display.width()-1, TOD_CURSORY-5, SSD1306_WHITE);

  // Display time of day
  DisplayTime(GetHour(), GetMinute(), GetSecond(), 3, TOD_CURSORX, TOD_CURSORY, true, !flash_flag, active_field);

  display.drawLine(0,MSG_CURSORY-5,display.width()-1, MSG_CURSORY-5, SSD1306_WHITE);

  // Display current homing sensor reading (boolean)
  DisplayMessage(active_field);

  display.display();
}

void DisplayTime(uint8_t hour, uint8_t minute, uint8_t second, uint8_t size, uint8_t cursorX, uint8_t cursorY, bool showSeconds, bool shouldFlash, uint8_t flash){
  // flash active field @1Hz 50% duty
  shouldFlash &= (millis()%1000) > 500;



  uint8_t isPM = false;
   if (hour == 0) { // midnight
      isPM = false;
      hour = 12;
    } else if (hour == 12) { // noon
      isPM = true;
      hour = 12;
    } else if (hour < 12) { // morning
      isPM = false;
      hour = hour;
    } else { // 1 o'clock or after
      isPM = true;
      hour = hour - 12;
    }
  
  sprintf(ss_str,"%02u", second);
  sprintf(mm_str,"%02u", minute);
  sprintf(hh_str,"%02u", hour);

  // Hour and minute fields
  display.setTextSize(size);
  display.setCursor(cursorX, cursorY);
  // Display hours (flash if selected)
  if ((flash == FIELD_TOD_HRS | flash == FIELD_FEED_HRS) && shouldFlash) display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  else display.setTextColor(SSD1306_WHITE);

  display.print(hh_str);

  display.setTextColor(SSD1306_WHITE);
  display.print(":");

  // Display minutes (flash if selected)
  if ((flash == FIELD_TOD_MIN | flash == FIELD_FEED_MIN) && shouldFlash) display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  else display.setTextColor(SSD1306_WHITE);
  
  display.print(mm_str);

  // Seconds field
  if (showSeconds) {
    display.setTextSize(1);
    // display.setCursor(cursorX + 94, cursorY);
    if (flash == FIELD_TOD_SEC && shouldFlash) display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    else display.setTextColor(SSD1306_WHITE);
    
    display.println(ss_str); 

    display.setTextColor(SSD1306_WHITE);
    if (isPM) display.print("PM");
    else display.print("AM");
  }
  else {
    display.setTextColor(SSD1306_WHITE);
    if (isPM) display.print("P");
    else display.print("A");
  }
  
  

  // Restore default text color and size
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
}

void DisplayMessage(uint8_t msg_ind){
  display.setCursor(MSG_CURSORX, MSG_CURSORY);
  display.setTextSize(1);
  display.invertDisplay(false);
  display.setTextColor(SSD1306_WHITE);
  switch (msg_ind) {
    case FIELD_TOD_HRS:
      display.print(MSG_TOD_HRS);
      break;
    case FIELD_TOD_MIN:
      display.print(MSG_TOD_MIN);
      break;
    case FIELD_TOD_SEC:
      display.print(MSG_TOD_SEC);
      break;
    case FIELD_FEED_HRS:
      display.print(MSG_FEED_HRS);
      break;
    case FIELD_FEED_MIN:
      display.print(MSG_FEED_MIN);
      break;
    default:
      display.print(MSG_DEFAULT);
      break;
  }
  display.setTextColor(SSD1306_WHITE);
}

void DisplayFeedingScreen(){
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(5, 20);
  display.print("Feeding...");
  // display.invertDisplay(true);
  display.display();
}

void AlarmRecorder(){
  shouldFeed = true;
}

void RotateTray(int direction){
  digitalWrite(EnablePin, LOW);
  // Cycle the tray
  stepper.moveRelativeInSteps(direction * STEPS_PER_TRAY);
  digitalWrite(EnablePin, HIGH);
  rtc.clearAlarm(1);
}

bool Debounce() {
  unsigned long t = millis();
  if (t >= input_time + INPUT_INTERVAL) {
    input_time = millis();
    repeat_time = millis();
    return false;
  }
  else return true;
}

void UpButtonRecorder() {
  if (Debounce()) return;
  UpWasPressed = true;
}
void MidButtonRecorder() {
  if (Debounce()) return;
  MidWasPressed = true;
}
void DownButtonRecorder() {
  if (Debounce()) return;
  DownWasPressed = true;
}

void UpButtonHandler() {
  UpWasPressed = false;
  // Increment active field
  switch(active_field){
    case FIELD_TOD_HRS:
      IncrementHours();
      break;
    case FIELD_TOD_MIN:
      IncrementMinutes();
      break;
    case FIELD_TOD_SEC:
      ResetSeconds();
      break;
    case FIELD_FEED_HRS:
      IncrementFeedingHour();
      break;
    case FIELD_FEED_MIN:
      IncrementFeedingMinute();
      break;
    default:
      break;
  }
  SaveTime();
}

void MidButtonHandler() {
  MidWasPressed = false;
  // Cycle selected field
  active_field = (active_field + 1) % 6;
}

void DownButtonHandler() {
  DownWasPressed = false;
  // Decrement active field
  switch(active_field){
    case FIELD_TOD_HRS:
      DecrementHours();
      break;
    case FIELD_TOD_MIN:
      DecrementMinutes();
      break;
    case FIELD_TOD_SEC:
      ResetSeconds();
      break;
    case FIELD_FEED_HRS:
      DecrementFeedingHour();
      break;
    case FIELD_FEED_MIN:
      DecrementFeedingMinute();
      break;
    default:
      break;
  }
  SaveTime();
}

bool GetHomingBool(){
  // return analogRead(HomingPin) < HOMING_THRESHOLD;
  return !digitalRead(HomingPin);
}

unsigned int GetSecond(){
  return now.second();
}

unsigned int GetMinute(){
  return now.minute();
}

unsigned int GetHour(){
  return now.hour();
}

unsigned int GetFeedHour(){
  return feedingTime.hour();
}

unsigned int GetFeedMinute(){
  return feedingTime.minute();
}

void LoadTime(){
  now = rtc.now();
}

void SaveTime(){
  
}

DateTime LoadFeedingTime(){
  uint8_t FEED_HRS, FEED_MIN;
  FEED_HRS = EEPROM.read(FEED_HRS_EEPROM_ADDR);
  FEED_MIN = EEPROM.read(FEED_MIN_EEPROM_ADDR);
  if (FEED_HRS > 23) FEED_HRS = 12;
  if (FEED_MIN > 59) FEED_MIN = 0;
  return DateTime(
    2020, // year
    0, // month
    0, // day
    FEED_HRS, // hour
    FEED_MIN, // minute
    0 // second
  );
}

void SaveFeedingTime(DateTime feedTime) {
  EEPROM.update(FEED_HRS_EEPROM_ADDR, feedTime.hour());
  EEPROM.update(FEED_MIN_EEPROM_ADDR, feedTime.minute());
}

void SetFeedingTime(DateTime feedTime){
  rtc.setAlarm1(
          feedTime,
          DS3231_A1_Hour // this mode triggers the alarm when the hours, minutes, and seconds match (once per day)
  );
  SaveFeedingTime(feedTime);
}

void ResetSeconds() {
  rtc.adjust(now - TimeSpan(now.second()));
}

void IncrementMinutes(){
  rtc.adjust(now + TimeSpan(60));
}

void DecrementMinutes(){
  rtc.adjust(now - TimeSpan(60));
}

void IncrementHours(){
  rtc.adjust(now + TimeSpan(3600));
}

void DecrementHours(){
  rtc.adjust(now - TimeSpan(3600));
}

void IncrementFeedingHour(){
  feedingTime = feedingTime + TimeSpan(3600);
  SetFeedingTime(feedingTime);
}

void DecrementFeedingHour(){
  feedingTime = feedingTime - TimeSpan(3600);
  SetFeedingTime(feedingTime);
}

void IncrementFeedingMinute(){
  feedingTime = feedingTime + TimeSpan(60);
  SetFeedingTime(feedingTime);
}

void DecrementFeedingMinute(){
  feedingTime = feedingTime - TimeSpan(60);
  SetFeedingTime(feedingTime);
}