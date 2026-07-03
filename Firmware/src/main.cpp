#include <EEPROM.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>

// ==================== PIN DEFINITIONS (STM32 Blue Pill) ====================
#define LED_PIN         PA1     // WS2812B Data (via 220Ω resistor)
#define PIEZO_PIN       PA0     // PVDF Piezo Analog Input (via 1MΩ pull-down)
#define NUM_PIXELS      1

// I2C Pins for MPU6050
#define I2C_SCL         PB6     // MPU6050 SCL
#define I2C_SDA         PB7     // MPU6050 SDA

// ==================== GLOBAL OBJECTS ====================
MPU6050 mpu(Wire);
Adafruit_NeoPixel pixels(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
arduinoFFT FFT = arduinoFFT();

// ==================== STATE MACHINE VARIABLES ====================
int currentState = 0; // 0=Idle, 1=Motion, 2=Impact, 3=Analyze
unsigned long lastMotionTime = 0;
const unsigned long MOTION_TIMEOUT = 200; // ms

// ==================== THRESHOLD CONFIGURATION ====================
float MOTION_THRESHOLD = 2.0;      // g-force for motion detection
int IMPACT_THRESHOLD = 2000;       // Piezo ADC value for impact detection
float FFT_MATCH_THRESHOLD = 85.0;  // % match required for success

// ==================== FFT & CALIBRATION VARIABLES ====================
#define SAMPLES 64
#define SAMPLING_FREQUENCY 1000
double vReal[SAMPLES];
double vImag[SAMPLES];
double goldenTemplate[SAMPLES];
bool isCalibrated = false;

void setup() {
  Serial.begin(115200);
  Serial.println("--- Smart Glove System Started ---");

  Wire.setSDA(PB7);
  Wire.setSCL(PB6);
  Wire.begin();

  mpu.begin();
  mpu.calcOffsets();
  Serial.println("MPU6050 Initialized on PB6/PB7");

  pixels.begin();
  pixels.show();

  if (checkEEPROM()) {
    loadTemplate();
    isCalibrated = true;
    Serial.println("Golden Template Loaded from EEPROM.");
    setLED(0, 255, 0);
    delay(500);
    setLED(0, 0, 0);
  } else {
    Serial.println("WARNING: Not Calibrated! Type 'C' to Calibrate.");
    setLED(255, 0, 0);
  }
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == 'C' || cmd == 'c') {
      Serial.println(">>> Entering Calibration Mode...");
      runCalibration();
      return;
    }
  }

  if (!isCalibrated) {
    delay(100);
    return;
  }

  switch (currentState) {
    case 0:
      if (detectMotion()) {
        currentState = 1;
        lastMotionTime = millis();
        Serial.println("State: Motion Detected");
      }
      break;

    case 1:
      if (detectImpact()) {
        currentState = 2;
        Serial.println("State: Impact Detected -> Running FFT");
      } else if (millis() - lastMotionTime > MOTION_TIMEOUT) {
        currentState = 0;
        Serial.println("State: Timeout -> Idle");
      }
      break;

    case 2:
      captureAndAnalyzeFFT();
      currentState = 0;
      break;
  }

  delay(10);
}

bool checkEEPROM() {
  return (EEPROM.read(0) == 0xAA);
}

void loadTemplate() {
  for (int i = 0; i < SAMPLES; i++) {
    EEPROM.get(1 + (i * 4), goldenTemplate[i]);
  }
  Serial.println("Template Loaded Successfully.");
}

void runCalibration() {
  setLED(0, 0, 255);
  Serial.println("Calibration Started.");
  Serial.println("Install 5 PERFECT locks now.");
  Serial.println("Waiting for impact...");

  int count = 0;
  float tempSum[SAMPLES];
  for (int i = 0; i < SAMPLES; i++) tempSum[i] = 0.0;

  while (count < 5) {
    while (analogRead(PIEZO_PIN) < IMPACT_THRESHOLD) {
      delay(10);
    }

    delay(20);
    Serial.print("Recording Good Lock #");
    Serial.println(count + 1);

    for (int i = 0; i < SAMPLES; i++) {
      vReal[i] = analogRead(PIEZO_PIN);
      vImag[i] = 0;
      delay(1);
    }

    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    for (int i = 0; i < SAMPLES; i++) {
      tempSum[i] += vReal[i];
    }

    count++;
    Serial.println("Waiting for next lock...");
    delay(1000);
  }

  Serial.println("Calculating Average...");
  for (int i = 0; i < SAMPLES; i++) {
    goldenTemplate[i] = tempSum[i] / 5.0;
  }

  Serial.println("Saving to EEPROM...");
  EEPROM.write(0, 0xAA);
  for (int i = 0; i < SAMPLES; i++) {
    EEPROM.put(1 + (i * 4), goldenTemplate[i]);
  }

  isCalibrated = true;
  setLED(0, 255, 0);
  Serial.println("Calibration Complete! System Ready.");
  delay(2000);
  setLED(0, 0, 0);
}

bool detectMotion() {
  mpu.update();
  float accZ = mpu.getAccZ();
  return abs(accZ) > MOTION_THRESHOLD;
}

bool detectImpact() {
  int val = analogRead(PIEZO_PIN);
  return val > IMPACT_THRESHOLD;
}

void captureAndAnalyzeFFT() {
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = analogRead(PIEZO_PIN);
    vImag[i] = 0;
    delay(1);
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  float totalDifference = 0.0;
  float totalMagnitude = 0.0;

  for (int i = 0; i < SAMPLES; i++) {
    float diff = abs(vReal[i] - goldenTemplate[i]);
    totalDifference += diff;
    totalMagnitude += goldenTemplate[i];
  }

  float matchPercentage = 100.0 - ((totalDifference / totalMagnitude) * 100.0);
  if (matchPercentage < 0) matchPercentage = 0;
  if (matchPercentage > 100) matchPercentage = 100;

  Serial.print("Match Score: ");
  Serial.print(matchPercentage);
  Serial.println("%");

  if (matchPercentage >= FFT_MATCH_THRESHOLD) {
    Serial.println("RESULT: SUCCESS (Green)");
    setLED(0, 255, 0);
  } else {
    Serial.println("RESULT: FAILURE (Red)");
    setLED(255, 0, 0);
  }

  delay(500);
  setLED(0, 0, 0);
}

void setLED(int r, int g, int b) {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}
