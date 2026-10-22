#define BLYNK_TEMPLATE_ID           "TMPL6ENvNREcS"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_FIRMWARE_VERSION      "0.1.1"
#define BLYNK_AUTH_TOKEN            "g7MSTm0G93EXFgLrKdSK_pPrdpRNBnbS"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <HTTPUpdate.h>

// ===== WiFi =====
char ssid[] = "SAYANGKU_2G";
char pass[] = "sayangku13";

// ===== Pin LED Indikator =====
const int ledBluePin = 2;   // LED Biru (WiFi / Blynk / OTA)
const int ledRedPin = 4;    // LED Merah (Status Motor Bergerak)

// ===== Motor pins =====
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 12; // PWM pin

// ===== Sensor encoder =====
const int sensorPin = 34;
volatile uint32_t pulseCount = 0;

// Encoder config
const uint16_t pulsesPerRev = 1;       // ganti sesuai encoder
const uint32_t rpmIntervalMs = 2000;   // sampling RPM tiap 2 detik

// ===== Target RPM per mode =====
const int targetLowRPM = 800;
const int targetMediumRPM = 900;
const int targetHighRPM = 2000;

// ===== Mode & state =====
enum SpeedMode { MODE_OFF, MODE_LOW, MODE_MEDIUM, MODE_HIGH };
volatile SpeedMode mode = MODE_OFF;
float targetRPM = 0.0f;

// Variable Waktu LED
unsigned long lastBlueLedToggle = 0;
unsigned long lastRedLedToggle = 0;
bool blueLedState = false;
bool redLedState = false;

// Interval Kedip (Dipercepat untuk v0.1.1)
const unsigned long blueLedIntervalFast = 150; // Kedip cepat saat terkoneksi Blynk (sebelumnya 500ms)
const unsigned long redLedIntervalFast  = 80;  // Kedip cepat saat motor menyala (sebelumnya 200ms)

// ===== PWM setup =====
const int pwmChannel = 0;
const int freq = 20000;
const int resolution = 8;
int duty = 0;

// ===== PID control =====
float Kp = 0.08f;
float Ki = 0.02f;
float Kd = 0.00f;
float integral = 0.0f;
float lastError = 0.0f;
const float integralMin = -5000.0f;
const float integralMax =  5000.0f;

// RPM smoothing
float rpmFiltered = 0.0f;
const float rpmAlpha = 0.5f;

// Waktu & OTA State
unsigned long kickStartEndTime = 0;
bool isKickStarting = false;

// Blynk Timer
BlynkTimer timer;

// ===== Interrupt =====
void IRAM_ATTR countPulse() {
  pulseCount++;
}

// ===== PID step =====
int computePidDuty(float target, float actual, float dtSec) {
  float error = target - actual;
  integral += error * dtSec;
  if (integral > integralMax) integral = integralMax;
  if (integral < integralMin) integral = integralMin;
  float derivative = (error - lastError) / dtSec;
  lastError = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  const int minDuty = 70;
  int dutyOut = (int)output;
  dutyOut = constrain(dutyOut, 0, 255);
  if (dutyOut > 0 && dutyOut < minDuty) dutyOut = minDuty;
  return dutyOut;
}

// ===== Mode setter non-blocking =====
void setMode(SpeedMode newMode, float newTarget, int kickDuty) {
  mode = newMode;
  targetRPM = newTarget;
  integral = 0; 
  lastError = 0;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);

  ledcWrite(pwmChannel, kickDuty);
  kickStartEndTime = millis() + 400;
  isKickStarting = true;
  
  Serial.print("Kick start duty: "); 
  Serial.println(kickDuty);
}

// ===== BLYNK OTA HANDLER =====
BLYNK_WRITE(InternalPinOTA) {
  String overTheAirURL = param.asString();
  Serial.print("Sinyal OTA diterima! Mengunduh firmware dari: ");
  Serial.println(overTheAirURL);

  // Matikan motor & LED merah saat pembaruan firmware demi keamanan
  mode = MODE_OFF;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(ledRedPin, LOW);
  ledcWrite(pwmChannel, 0);

  // Indikator Progress OTA: LED Biru berkedip sangat cepat (50ms)
  httpUpdate.onProgress([](int cur, int total) {
    static unsigned long lastOtaBlink = 0;
    if (millis() - lastOtaBlink >= 50) { 
      digitalWrite(ledBluePin, !digitalRead(ledBluePin));
      lastOtaBlink = millis();
    }
  });

  WiFiClient client;
  t_httpUpdate_return ret = httpUpdate.update(client, overTheAirURL);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("OTA Gagal. Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      digitalWrite(ledBluePin, LOW);
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("Tidak ada pembaruan firmware.");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("OTA Berhasil! ESP32 akan merestart...");
      digitalWrite(ledBluePin, HIGH); 
      break;
  }
}

// Synchronize state saat Blynk terhubung
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V0);
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// ===== BLYNK INPUT (Virtual Pin V0) =====
BLYNK_WRITE(V0) {
  int value = param.asInt();
  Serial.print("Blynk Control Input (V0): "); 
  Serial.println(value);

  switch (value) {
    case 0: // OFF
      mode = MODE_OFF;
      targetRPM = 0;
      integral = 0; 
      lastError = 0;
      duty = 0;
      isKickStarting = false;
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      digitalWrite(ledRedPin, LOW);
      ledcWrite(pwmChannel, 0);
      Serial.println("Motor OFF");
      break;

    case 1: // LOW
      setMode(MODE_LOW, targetLowRPM, 200);
      Serial.println("Motor LOW target 800 RPM");
      break;

    case 2: // MEDIUM
      setMode(MODE_MEDIUM, targetMediumRPM, 220);
      Serial.println("Motor MEDIUM target 900 RPM");
      break;

    case 3: // HIGH
      setMode(MODE_HIGH, targetHighRPM, 255);
      Serial.println("Motor HIGH target 2000 RPM");
      break;
  }

  Blynk.virtualWrite(V1, value);
}

// Process RPM and PID Loop
void processRpmAndPid() {
  noInterrupts();
  uint32_t pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  float revs = (float)pulses / (float)pulsesPerRev;
  float rpm = revs * (60000.0f / (float)rpmIntervalMs);
  rpmFiltered = (rpmAlpha * rpm) + ((1.0f - rpmAlpha) * rpmFiltered);

  Serial.print("RPM raw: "); Serial.print(rpm);
  Serial.print(" | RPM filt: "); Serial.println(rpmFiltered);

  Blynk.virtualWrite(V2, rpmFiltered);

  if (mode != MODE_OFF) {
    if (isKickStarting) {
      if (millis() >= kickStartEndTime) {
        isKickStarting = false;
      }
    } else {
      float dtSec = (float)rpmIntervalMs / 1000.0f;
      duty = computePidDuty(targetRPM, rpmFiltered, dtSec);
      ledcWrite(pwmChannel, duty);
    }
  } else {
    duty = 0;
    ledcWrite(pwmChannel, 0);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(ledBluePin, OUTPUT);
  pinMode(ledRedPin, OUTPUT);

  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(sensorPin, countPulse, RISING);

  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  mode = MODE_OFF;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(ledRedPin, LOW);
  ledcWrite(pwmChannel, 0);

  rpmFiltered = 0.0f;

  timer.setInterval(rpmIntervalMs, processRpmAndPid);
}

void loop() {
  Blynk.run();
  timer.run();

  unsigned long now = millis();

  // --- LED Biru: Berkedip Terus-Menerus Ketika WiFi/Blynk Tersambung (150ms) ---
  if (Blynk.connected()) {
    if (now - lastBlueLedToggle >= blueLedIntervalFast) {
      blueLedState = !blueLedState;
      digitalWrite(ledBluePin, blueLedState);
      lastBlueLedToggle = now;
    }
  } else {
    digitalWrite(ledBluePin, LOW);
  }

  // --- LED Merah: Berkedip Cepat Ketika Motor Bergerak (80ms) ---
  if (mode != MODE_OFF) {
    if (now - lastRedLedToggle >= redLedIntervalFast) {      redLedState = !redLedState;
      digitalWrite(ledRedPin, redLedState);
      lastRedLedToggle = now;
    }
  } else {
    digitalWrite(ledRedPin, LOW);
  }
}