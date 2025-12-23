#include <WiFi.h>
#include <PubSubClient.h>

// ===== WiFi =====
const char* ssid = "Redmi Note 11";
const char* password = "mywifi123";

// ===== MQTT =====
const char* mqtt_server = "broker.hivemq.com";
WiFiClient espClient;
PubSubClient client(espClient);

// ===== Motor pins =====
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 12; // PWM pin

// ===== Sensor encoder =====
const int sensorPin = 34;       // hall/encoder input
volatile uint32_t pulseCount = 0;

// Encoder config
const uint16_t pulsesPerRev = 1;      // ganti sesuai encoder kamu (mis. 20 PPR)
const uint32_t rpmIntervalMs = 1000;  // sampling RPM tiap 1 detik

// ===== Target RPM per mode =====
const int targetLowRPM = 200;
const int targetMediumRPM = 2000;
const int targetHighRPM = 8000;

// ===== Mode & state =====
enum SpeedMode { MODE_OFF, MODE_LOW, MODE_MEDIUM, MODE_HIGH };
volatile SpeedMode mode = MODE_OFF;
float targetRPM = 0.0f;

// ===== LED indikator =====
const int ledPin = 2;
unsigned long lastLedToggle = 0;
bool ledState = false;

// ===== PWM setup =====
const int pwmChannel = 0;
const int freq = 20000;     // 20 kHz untuk driver motor yang lebih halus
const int resolution = 8;   // 8-bit (0–255)
int duty = 0;               // duty cycle current

// ===== PID control (tuning) =====
// Catatan: mulai dari nilai konservatif, sesuaikan di lapangan:
// - Kp: seberapa agresif merespons error RPM
// - Ki: hilangkan steady-state error (hati-hati wind-up)
// - Kd: redam perubahan cepat (noise dari encoder rendah resolusi)
float Kp = 0.08f;     // gain proporsional
float Ki = 0.02f;     // gain integral
float Kd = 0.00f;     // gain derivatif (set 0 dulu; naikkan jika respons terlalu “melompat”)
float integral = 0.0f;
float lastError = 0.0f;

// Anti-windup untuk integral
const float integralMin = -5000.0f;
const float integralMax =  5000.0f;

// RPM smoothing (optional)
float rpmFiltered = 0.0f;
const float rpmAlpha = 0.5f;  // 0–1; makin tinggi makin responsif, makin rendah makin halus

// Waktu
unsigned long lastRpmTime = 0;

// ===== Interrupt hitung pulsa =====
void IRAM_ATTR countPulse() {
  pulseCount++;
}

// ===== WiFi connect =====
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    // LED kedip pelan saat belum konek WiFi
    digitalWrite(ledPin, (millis() % 1000 < 500) ? HIGH : LOW);
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // LED berkedip cepat 3x tanda WiFi tersambung
  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH); delay(100);
    digitalWrite(ledPin, LOW);  delay(100);
  }
}

// ===== MQTT callback =====
void callback(char* topic, byte* message, unsigned int length) {
  String msg; msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)message[i];
  msg.trim();
  Serial.print("Pesan: "); Serial.println(msg);

  if (msg == "off") {
    mode = MODE_OFF;
    targetRPM = 0;
    integral = 0;
    lastError = 0;
    duty = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, 0);
    Serial.println("Motor OFF");
  } else if (msg == "LOW") {
    mode = MODE_LOW;
    targetRPM = targetLowRPM;
    integral = 0; lastError = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    Serial.println("Motor LOW target 200 RPM");
  } else if (msg == "MEDIUM") {
    mode = MODE_MEDIUM;
    targetRPM = targetMediumRPM;
    integral = 0; lastError = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    Serial.println("Motor MEDIUM target 2000 RPM");
  } else if (msg == "HIGH" || msg == "on") {
    mode = MODE_HIGH;
    targetRPM = targetHighRPM;
    integral = 0; lastError = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    Serial.println("Motor HIGH target 8000 RPM");
  }

  // Optional: set PID gains over MQTT, misal "SETKp=0.1"
  if (msg.startsWith("SETKp=")) Kp = msg.substring(6).toFloat();
  if (msg.startsWith("SETKi=")) Ki = msg.substring(6).toFloat();
  if (msg.startsWith("SETKd=")) Kd = msg.substring(6).toFloat();
  if (msg.startsWith("SETLOW="))  { /* ubah target low via MQTT */ float v = msg.substring(7).toFloat(); if (v>0) { /* keep */ } }
}

// ===== MQTT reconnect =====
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32MotorClient")) {
      Serial.println("connected");
      client.subscribe("esp32/motor/control"); // LOW/MEDIUM/HIGH/off & optional SETKp/SETKi/SETKd
    } else {
      Serial.print("failed, rc="); Serial.print(client.state());
      Serial.println(" retry in 5s...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(sensorPin, INPUT_PULLUP);
  attachInterrupt(sensorPin, countPulse, RISING);

  // PWM
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  mode = MODE_OFF;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, 0);

  lastRpmTime = millis();
}

// ===== PID step =====
int computePidDuty(float target, float actual, float dtSec) {
  // Error
  float error = target - actual;

  // Integral with anti-windup
  integral += error * dtSec;
  if (integral > integralMax) integral = integralMax;
  if (integral < integralMin) integral = integralMin;

  // Derivative
  float derivative = (error - lastError) / dtSec;
  lastError = error;

  // PID output (map ke duty 0–255)
  float output = Kp * error + Ki * integral + Kd * derivative;

  // Minimum duty to overcome dead-zone (kalibrasi sesuai motor/driver)
  const int minDuty = 20; // ~8% dari 255, sesuaikan di lapangan

  int dutyOut = (int)output;
  dutyOut = constrain(dutyOut, 0, 255);
  if (dutyOut > 0 && dutyOut < minDuty) dutyOut = minDuty;

  return dutyOut;
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  // Hitung RPM tiap interval
  if (now - lastRpmTime >= rpmIntervalMs) {
    // ambil pulsa secara atomic
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // hitung RPM
    float revs = (float)pulses / (float)pulsesPerRev;
    float rpm = revs * (60000.0f / rpmIntervalMs);

    // smoothing agar encoder low-res tidak bikin loncat
    rpmFiltered = (rpmAlpha * rpm) + ((1.0f - rpmAlpha) * rpmFiltered);

    Serial.print("RPM raw: "); Serial.print(rpm);
    Serial.print(" | RPM filt: "); Serial.println(rpmFiltered);

    // publish ke MQTT (pakai filtered biar gauge enak)
    char buf[20];
    dtostrf(rpmFiltered, 0, 1, buf);
    client.publish("esp32/motor/speed", buf);

    // PID control
    if (mode != MODE_OFF) {
      float dtSec = (float)rpmIntervalMs / 1000.0f;
      duty = computePidDuty(targetRPM, rpmFiltered, dtSec);
      ledcWrite(pwmChannel, duty);
    } else {
      duty = 0;
      ledcWrite(pwmChannel, 0);
    }

    lastRpmTime = now;
  }

  // LED indikator
  if (WiFi.status() != WL_CONNECTED) {
    // belum connect WiFi → kedip pelan
    if (now - lastLedToggle >= 1000) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastLedToggle = now;
    }
  } else if (WiFi.status() == WL_CONNECTED && mode == MODE_OFF) {
    // WiFi tersambung, motor idle → LED nyala stabil
    digitalWrite(ledPin, HIGH);
  } else {
    // WiFi tersambung, motor aktif → LED kedip cepat
    if (now - lastLedToggle >= 200) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastLedToggle = now;
    }
  }
}
