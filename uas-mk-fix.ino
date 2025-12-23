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
const int sensorPin = 34;
volatile uint32_t pulseCount = 0;

// Encoder config
const uint16_t pulsesPerRev = 1;      // ganti sesuai encoder kamu
const uint32_t rpmIntervalMs = 2000;  // sampling RPM tiap 2 detik biar lebih stabil

// ===== Target RPM per mode =====
const int targetLowRPM = 800;
const int targetMediumRPM = 900;
const int targetHighRPM = 2000;

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

// Waktu
unsigned long lastRpmTime = 0;

// ===== Interrupt =====
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
    digitalWrite(ledPin, (millis() % 1000 < 500) ? HIGH : LOW);
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  for (int i = 0; i < 3; i++) {
    digitalWrite(ledPin, HIGH); delay(100);
    digitalWrite(ledPin, LOW);  delay(100);
  }
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
  const int minDuty = 70; // dinaikkan biar motor nggak berhenti
  int dutyOut = (int)output;
  dutyOut = constrain(dutyOut, 0, 255);
  if (dutyOut > 0 && dutyOut < minDuty) dutyOut = minDuty;
  return dutyOut;
}

// ===== Mode setter dengan kick start =====
void setMode(SpeedMode newMode, float newTarget, int kickDuty) {
  mode = newMode;
  targetRPM = newTarget;
  integral = 0; lastError = 0;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);

  // Kick start setiap kali pindah mode
  ledcWrite(pwmChannel, kickDuty);
  delay(400);
  Serial.print("Kick start duty: "); Serial.println(kickDuty);
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
    integral = 0; lastError = 0;
    duty = 0;
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, 0);
    Serial.println("Motor OFF");
  } else if (msg == "LOW") {
    setMode(MODE_LOW, targetLowRPM, 200);
    Serial.println("Motor LOW target 800 RPM");
  } else if (msg == "MEDIUM") {
    setMode(MODE_MEDIUM, targetMediumRPM, 220);
    Serial.println("Motor MEDIUM target 900 RPM");
  } else if (msg == "HIGH" || msg == "on") {
    setMode(MODE_HIGH, targetHighRPM, 255);
    Serial.println("Motor HIGH target 2000 RPM");
  }
}

// ===== MQTT reconnect =====
void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32MotorClient")) {
      Serial.println("connected");
      client.subscribe("esp32/motor/control");
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
  rpmFiltered = 0.0f;
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  if (now - lastRpmTime >= rpmIntervalMs) {
    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float revs = (float)pulses / (float)pulsesPerRev;
    float rpm = revs * (60000.0f / rpmIntervalMs);
    rpmFiltered = (rpmAlpha * rpm) + ((1.0f - rpmAlpha) * rpmFiltered);

    Serial.print("RPM raw: "); Serial.print(rpm);
    Serial.print(" | RPM filt: "); Serial.println(rpmFiltered);

    char buf[20];
    dtostrf(rpmFiltered, 0, 1, buf);
    client.publish("esp32/motor/speed", buf);

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
    if (now - lastLedToggle >= 1000) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastLedToggle = now;
    }
  } else if (WiFi.status() == WL_CONNECTED && mode == MODE_OFF) {
    digitalWrite(ledPin, HIGH);
  } else {
    if (now - lastLedToggle >= 200) {
      ledState = !ledState;
      digitalWrite(ledPin, ledState);
      lastLedToggle = now;
    }
  }
}
