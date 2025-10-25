#include <LiquidCrystal_I2C.h>

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6jvfvFWmW"
#define BLYNK_TEMPLATE_NAME "Robot"
#define BLYNK_AUTH_TOKEN "CmJL2kO0pnvVrDzacgdKi80sXoM3Gdac"

#include <Arduino.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>

// WiFi credentials
#define WIFI_SSID "Wifi01"
#define WIFI_PASSWORD "password123"

// Pin definitions
#define ServoPin 13
#define GasSenPin 32

#define Motor1A 27
#define Motor1B 26
#define Motor2A 25
#define Motor2B 33
#define MotorEN 14  // Shared speed control

// Global hardware objects
Servo MyServo;
BluetoothSerial SerialBT;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Control variables
volatile int XValue = 0;
volatile int YValue = 0;
int MotorSpeed = 100;
int gasValue = 0;
const int gasThreshold = 1500;
bool gasAlertState = true;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// PWM variables
int dutyCycle = 0;  // 0-100 %
const int pwmCycleMs = 100;

// Timers
unsigned long lastMotorUpdate = 0;
const int MOTOR_UPDATE_INTERVAL = 50;
unsigned long lastGasCheck = 0;

int newGasValue = 0;

int newState = 0;
int prevState = 0;

// Software PWM function
void updateSoftwarePWM() {
  static unsigned long lastCycle = 0;
  unsigned long now = millis();

  if (now - lastCycle >= pwmCycleMs) {
    lastCycle = now;

    digitalWrite(MotorEN, HIGH);
    delayMicroseconds(dutyCycle * 400);  // convert % to microseconds
    digitalWrite(MotorEN, LOW);
  }
}

// Blynk virtual pins
BLYNK_WRITE(V0) {
  XValue = param.asInt();
  processBlynkJoystick();
}

BLYNK_WRITE(V1) {
  YValue = param.asInt();
  processBlynkJoystick();
}

BLYNK_WRITE(V2) {
  newState = param.asInt();
  if (newState != prevState) {
    MyServo.write(newState ? 90 : 0);
    prevState = newState;
  } else {
  }
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP Robot");

  // Motor setup
  pinMode(Motor1A, OUTPUT);
  pinMode(Motor1B, OUTPUT);
  pinMode(Motor2A, OUTPUT);
  pinMode(Motor2B, OUTPUT);
  pinMode(MotorEN, OUTPUT);
  stopMotors();

  MyServo.attach(ServoPin);
  MyServo.write(0);
  lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("First Aid Caring");
  lcd.setCursor(0, 1);
  lcd.print("Assistant Robot");
  delay(2000);
  lcd.clear();
  Serial.print("Connecting to WiFi ..");
  lcd.setBacklight(255);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wifi Connecting");
  lcd.setCursor(0, 1);
  lcd.print(".....");
  delay(1000);
  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Wifi Connected");
  lcd.setBacklight(255);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wifi Connected");
  delay(1000);
}

void loop() {
  if (!SerialBT.connected()) {
    Blynk.run();


    if (millis() - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL) {
      processBlynkJoystick();
      lastMotorUpdate = millis();
    }

    processGasSensor();
    updateSoftwarePWM();  // PWM pulse generation

  } else {
  }

  while (SerialBT.connected()) {

    processGasSensor();
    if (SerialBT.available()) {
      handleCommand(SerialBT.read());
    } else {
    }
  }
}

void processGasSensor() {
  newGasValue = analogRead(GasSenPin);
  if ((newGasValue > gasThreshold) != gasAlertState) {
    gasValue = newGasValue;
    gasAlertState = (gasValue > gasThreshold);

    Blynk.virtualWrite(V3, gasAlertState ? "Gas Detected!" : "Gas Clear");
   
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(gasAlertState ? "GAS ALERT! " : "Normal Op. ");
    lcd.print(gasValue);
  }
}

void processBlynkJoystick() {
  int x, y;
  taskENTER_CRITICAL(&mux);
  x = XValue;
  y = YValue;
  taskEXIT_CRITICAL(&mux);

  const int deadBand = 15;
  if (abs(x) < deadBand && abs(y) < deadBand) {
    stopMotors();
    return;
  }

  id setMotors(float left, float right) {
  // Direction control
  digitalWrite(Motor1A, right < 0 ? HIGH : LOW);
  digitalWrite(Motor1B, right > 0 ? HIGH : LOW);
  digitalWrite(Motor2A, left < 0 ? HIGH : LOW);
  digitalWrite(Motor2B, left > 0 ? HIGH : LOW);

  // Shared speed from average of absolute values
  dutyCycle = constrain((abs((int)left) + abs((int)right)) / 2, 0, 100);
}

void stopMotors() {
  digitalWrite(MotorEN, LOW);
  digitalWrite(Motor1A, LOW);
  digitalWrite(Motor1B, LOW);
  digitalWrite(Motor2A, LOW);
  digitalWrite(Motor2B, LOW);
  dutyCycle = 0;
}

void handleCommand(char cmd) {
  switch (cmd) {
    case 'F':
      digitalWrite(MotorEN, HIGH);
      setMotors(100, 100);
      break;
    case 'B':
      digitalWrite(MotorEN, HIGH);
      setMotors(-100, -100);
      break;
    case 'L':
      digitalWrite(MotorEN, HIGH);
      setMotors(-100, 100);
      break;
    case 'R':
      digitalWrite(MotorEN, HIGH);
      setMotors(100, -100);
      break;
    case 'G':
      digitalWrite(MotorEN, HIGH);
      setMotors(0, -100);
      break;
    case 'I':
      digitalWrite(MotorEN, HIGH);
      setMotors(-100, 0);
      break;
    case 'H':
      digitalWrite(MotorEN, HIGH);
      setMotors(0, 100);
      break;
    case 'J':
      digitalWrite(MotorEN, HIGH);
      setMotors(100, 0);
      break;
    case 'x': MyServo.write(0); break;
    case 'X': MyServo.write(90); break;
    case 'S': stopMotors(); break;
    default: stopMotors(); break;
  }
}
