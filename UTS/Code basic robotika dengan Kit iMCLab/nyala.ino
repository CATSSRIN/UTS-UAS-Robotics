#include <Arduino.h>

// --- KONFIGURASI PIN & PWM ---
// Pin untuk Driver Motor
int motorPin1 = 27;     // Pin IN1 pada driver L298N
int motorPin2 = 26;     // Pin IN2 pada driver L298N
int enablePin = 12;     // Pin ENA (Enable A) pada driver, untuk sinyal PWM

// Properti PWM
const int PWM_CHANNEL = 0;    // Channel PWM (0-15)
const int PWM_FREQ = 5000;    // Frekuensi (Hz)
const int PWM_RESOLUTION = 8; // Resolusi 8-bit (nilai 0-255)

// Atur kecepatan motor di sini!
// Ganti nilai ini dari 0 (berhenti) hingga 255 (kecepatan penuh)
int motorSpeed = 200;

void setup() {
  // Atur semua pin motor sebagai OUTPUT
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Konfigurasi fungsi PWM dengan properti yang sudah ditentukan
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // Hubungkan channel PWM ke pin 'enable'
  ledcAttachPin(enablePin, PWM_CHANNEL);

  // --- Atur Arah Putaran Motor ---
  // Untuk berputar maju (contoh)
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);

  /*
    // Untuk berputar mundur, gunakan konfigurasi ini:
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  */

  // --- Jalankan Motor ---
  // Kirim sinyal PWM ke pin 'enable' untuk memulai putaran
  ledcWrite(PWM_CHANNEL, motorSpeed);
}

void loop() {
  // Biarkan kosong, karena motor sudah diatur untuk berjalan
  // terus menerus di dalam fungsi setup().
}