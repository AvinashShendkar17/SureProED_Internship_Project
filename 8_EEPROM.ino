#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <Servo.h>
#include <EEPROM.h>

// ---------------- CONFIG ----------------
#define IN1 2
#define IN2 4
#define IN3 3
#define IN4 5

#define SERVO_PIN 9
#define GREEN_LED 6
#define RED_LED   7

// Servo lock/unlock angles (change if needed)
const int LOCK_ANGLE   = 6;    // pin inserted -> LOCK
const int UNLOCK_ANGLE = 108;  // pin retracted -> UNLOCK

// delays (ms) - tune as needed
const int BEFORE_UNLOCK_DELAY = 700;
const int AFTER_UNLOCK_DELAY  = 500;
const int AFTER_MOVE_DELAY    = 800;
const int AFTER_LOCK_DELAY    = 400;

// LCD (change address to 0x3F if 0x27 doesn't match)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Keypad wiring
const byte ROWS = 4;
const byte COLS = 4;
byte rowPins[ROWS] = {10, 11, 12, 13};    // Rows
byte colPins[COLS] = {A0, A1, A2, A3};    // Cols (analog as digital)
char keysMap[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
Keypad keypad = Keypad( makeKeymap(keysMap), rowPins, colPins, ROWS, COLS );

// Stepper half/full step sequence (full-step sequence for your wiring)
const int stepSequence[4][4] = {
  {1,0,1,0},
  {0,1,1,0},
  {0,1,0,1},
  {1,0,0,1}
};
int stepIndex = 0;
const int STEP_DELAY_MS = 2; // smaller -> faster (tune)

// Weight gaps (10->20, 20->30, ..., 70->80)
const int gaps[7] = {3600, 3900, 3900, 3850, 4000, 3800, 3800};
const int weightOptions[8] = {10,20,30,40,50,60,70,80};

// State
int currentIndex = 6;     // default index = 6 -> 70kg (useful if EEPROM empty)
String inputBuffer = "";
Servo lockServo;

// EEPROM address
const int EEPROM_ADDR_INDEX = 0;

// ---------------- Helpers ----------------
void setStepperPins(const int pins[4]) {
  digitalWrite(IN1, pins[0]);
  digitalWrite(IN2, pins[1]);
  digitalWrite(IN3, pins[2]);
  digitalWrite(IN4, pins[3]);
}

void releaseStepper() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void moveStepperSteps(long steps) {
  int dir = (steps > 0) ? 1 : -1;
  long s = (steps > 0) ? steps : -steps;
  for (long i = 0; i < s; ++i) {
    stepIndex += dir;
    if (stepIndex > 3) stepIndex = 0;
    if (stepIndex < 0) stepIndex = 3;
    setStepperPins(stepSequence[stepIndex]);
    delay(STEP_DELAY_MS);
  }
}

// calculate steps between weights (positive = downward/increase weight)
long calculateStepsBetweenIndices(int fromIdx, int toIdx) {
  long steps = 0;
  if (toIdx > fromIdx) {
    for (int i = fromIdx; i < toIdx; ++i) steps += gaps[i];
    return steps; // positive
  } else if (toIdx < fromIdx) {
    for (int i = toIdx; i < fromIdx; ++i) steps += gaps[i];
    return -steps; // negative (upward)
  }
  return 0;
}

void saveIndexToEEPROM(int idx) {
  // use EEPROM.update to avoid unnecessary writes
  EEPROM.update(EEPROM_ADDR_INDEX, (uint8_t)idx);
}

int readIndexFromEEPROM() {
  int v = EEPROM.read(EEPROM_ADDR_INDEX);
  if (v >= 0 && v <= 7) return v;
  return -1;
}

// UI helpers
void showIdleLCD() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Locked at:");
  lcd.setCursor(0,1);
  lcd.print(weightOptions[currentIndex]);
  lcd.print(" kg  *clear");
}

void showEnteringLCD() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Enter Weight:");
  lcd.setCursor(0,1);
  lcd.print(inputBuffer + " kg");
}

void showMsgLCD(const char *l1, const char *l2 = nullptr) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(l1);
  if (l2) {
    lcd.setCursor(0,1);
    lcd.print(l2);
  }
}

// lock/unlock functions (use constants)
void doUnlock() {
  lockServo.write(UNLOCK_ANGLE);
}

void doLock() {
  lockServo.write(LOCK_ANGLE);
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  // Servo
  lockServo.attach(SERVO_PIN);

  // LCD
  lcd.init();
  lcd.backlight();

  // Read EEPROM for last saved index
  int saved = readIndexFromEEPROM();
  if (saved >= 0) {
    currentIndex = saved;
  } else {
    // If EEPROM empty/corrupt, default to 70kg (as you said current)
    currentIndex = 6; // index 6 -> 70kg
    saveIndexToEEPROM(currentIndex);
  }

  // Ensure known starting physical state: assume locked
  doLock();
  releaseStepper();
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(RED_LED, LOW);

  // Show initial on LCD
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Gym System Ready");
  lcd.setCursor(0,1);
  lcd.print("Locked: ");
  lcd.print(weightOptions[currentIndex]);
  lcd.print("kg");
  delay(1200);
  showIdleLCD();

  Serial.print("Startup weight: ");
  Serial.print(weightOptions[currentIndex]);
  Serial.println(" kg (index ");
  Serial.print(currentIndex);
  Serial.println(" )");
}

// ---------------- Loop ----------------
void loop() {
  char key = keypad.getKey();
  if (key) {
    if (key >= '0' && key <= '9') {
      // Append digit (allow up to two digits)
      if (inputBuffer.length() < 2) {
        inputBuffer += key;
        showEnteringLCD();
      }
    } else if (key == '*') {
      // clear input
      inputBuffer = "";
      showIdleLCD();
    } else if (key == '#') {
      // confirm
      if (inputBuffer.length() == 0) {
        showMsgLCD("No input", "Press digits then #");
        delay(900);
        showIdleLCD();
      } else {
        int val = inputBuffer.toInt();
        inputBuffer = "";
        // allow only multiples of 10 within range
        if (val % 10 == 0 && val >= 10 && val <= 80) {
          int targetIdx = (val - 10) / 10;
          if (targetIdx == currentIndex) {
            showMsgLCD("Already at", String(val).c_str());
            delay(900);
            showIdleLCD();
          } else {
            // perform change
            // indicate busy
            digitalWrite(GREEN_LED, LOW);
            digitalWrite(RED_LED, HIGH);

            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Moving to ");
            lcd.print(val);
            lcd.print("kg");
            lcd.setCursor(0,1);
            lcd.print("Please stand clear");

            delay(BEFORE_UNLOCK_DELAY);

            // Unlock
            doUnlock();
            delay(AFTER_UNLOCK_DELAY);

            // calculate and move
            long steps = calculateStepsBetweenIndices(currentIndex, targetIdx);
            Serial.print("Moving steps: ");
            Serial.println(steps);
            moveStepperSteps(steps);
            releaseStepper();

            delay(AFTER_MOVE_DELAY);

            // Lock
            doLock();
            delay(AFTER_LOCK_DELAY);

            // update state & EEPROM
            currentIndex = targetIdx;
            saveIndexToEEPROM(currentIndex);

            // LEDs and display
            digitalWrite(RED_LED, LOW);
            digitalWrite(GREEN_LED, HIGH);

            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("Locked at ");
            lcd.print(val);
            lcd.setCursor(0,1);
            lcd.print("Saved in EEPROM");
            delay(900);
            showIdleLCD();

            Serial.print("Now locked at ");
            Serial.print(weightOptions[currentIndex]);
            Serial.println(" kg (index saved to EEPROM).");
          }
        } else {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Invalid value!");
          lcd.setCursor(0,1);
          lcd.print("10,20..80 only");
          delay(900);
          showIdleLCD();
        }
      }
    } // end '#' case
  } // end key check
}
