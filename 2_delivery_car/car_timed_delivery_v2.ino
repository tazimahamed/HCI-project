/*
  CAR - TIMED DELIVERY  (no line sensors)
  ESP32 + TB6612FNG + buzzer + onboard blue LED

  The IR line array is not used at all. The car drives on timing only.

  WHAT HAPPENS WHEN A TABLE IS CALLED

      go forward  ->  stop and blink blue  ->  turn around  ->  come back  ->  stop

    table 1 : forward for TABLE1_FORWARD_MS
    table 2 : forward for TABLE2_FORWARD_MS

  While it is stopped at the table the onboard blue LED blinks for the
  whole wait, then goes out before the car turns around.

  The car creeps forward a little while it spins round, so coming back
  for the full outbound time carries it past the station. The return
  leg is therefore a percentage of the outbound time, set by
  RETURN_PERCENT.

  Wiring (motors only):
    TB6612FNG  PWMA->4  AIN2->16  AIN1->17  STBY->5
               BIN1->18 BIN2->19  PWMB->21
    Buzzer     GPIO 12
    Blue LED   GPIO 2   (already on the ESP32 board, nothing to wire)

  MQTT:
    subscribe   restaurant/car/command    {"table":1} or {"table":2}
    publish     restaurant/car/state      idle / moving / at_table / returning
    publish     restaurant/car/presence   online / offline (last will)

  Serial Monitor at 115200, line ending "Newline".
    1   run the table 1 trip
    2   run the table 2 trip
    s   stop right now
    f   drive forward 1 second     (for measuring distance)
    r   turn around once           (for setting TURN_TIME)
*/

#include <WiFi.h>
#include <PubSubClient.h>

// ---------- network settings ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* MQTT_HOST = "192.168.0.200";   // kitchen laptop IP
const int   MQTT_PORT = 1883;

const char* TOPIC_COMMAND  = "restaurant/car/command";
const char* TOPIC_STATE    = "restaurant/car/state";
const char* TOPIC_PRESENCE = "restaurant/car/presence";

WiFiClient espClient;
PubSubClient mqtt(espClient);
unsigned long lastMqttTry = 0;

// ================= THE NUMBERS TO ADJUST =================

// How long the car drives forward for each table, in milliseconds.
// 1000 = 1 second.
int TABLE1_FORWARD_MS = 300;
int TABLE2_FORWARD_MS = 750;

// How long it pauses at the table. The blue LED blinks for this long.
int WAIT_AT_TABLE_MS = 3000;

// How fast the blue LED blinks while waiting. Smaller = faster.
int BLINK_INTERVAL_MS = 250;

// How long a half turn takes. Set this by typing r in the Serial
// Monitor and adjusting until the car ends up facing backwards.
int TURN_TIME_MS = 720;

// The car drifts forward a little while it turns around, so coming
// back for the full outbound time overshoots the station. This is the
// percentage of the outbound time used on the way back.
//   90 = come back for 90% of the time it went out for
//   still overshooting -> lower it (85, 80)
//   stopping too early -> raise it (95)
int RETURN_PERCENT = 90;

// Which way it turns around. true = spin right, false = spin left.
bool TURN_RIGHT = true;

// Speeds, 0 to 255
int DRIVE_SPEED = 150;
int TURN_SPEED  = 150;

// One wheel is often slightly faster than the other, which makes the
// car curve instead of going straight. Trim it here.
//   positive number = slow the LEFT wheel down
//   negative number = slow the RIGHT wheel down
int STRAIGHT_TRIM = 0;

// =========================================================

// ---------- motor pins ----------
#define PWMA 4
#define AIN2 16
#define AIN1 17
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

#define BUZZER 12

// the blue LED already fitted to the ESP32 board
#define BLUE_LED 2

#define CH_A 0
#define CH_B 1
#define PWM_FREQ 1000
#define PWM_BITS 8

// motor 1 is channel A. On this car channel A is the RIGHT wheel.
bool MOTOR1_IS_LEFT = false;

// ---------- states ----------
// 0 idle
// 1 driving out
// 2 waiting at the table
// 3 turning around
// 4 driving back
int state = 0;

unsigned long stateStart = 0;
int targetTable = 0;
int forwardTime = 0;      // outbound time for this trip
int returnTime  = 0;      // the shorter time used coming back

// ---------- blue LED blinking ----------
unsigned long lastBlink = 0;
bool blueOn = false;

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);

  setupPwm();
  stopMotors();

  connectWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  Serial.println();
  Serial.println("===== TIMED DELIVERY CAR =====");
  Serial.print("  table 1 forward: ");
  Serial.print(TABLE1_FORWARD_MS);
  Serial.print(" ms, back ");
  Serial.print(TABLE1_FORWARD_MS * RETURN_PERCENT / 100);
  Serial.println(" ms");
  Serial.print("  table 2 forward: ");
  Serial.print(TABLE2_FORWARD_MS);
  Serial.print(" ms, back ");
  Serial.print(TABLE2_FORWARD_MS * RETURN_PERCENT / 100);
  Serial.println(" ms");
  Serial.print("  wait at table:   ");
  Serial.print(WAIT_AT_TABLE_MS);
  Serial.println(" ms, blue LED blinking");
  Serial.println();
  Serial.println("  1  run table 1 trip     2  run table 2 trip");
  Serial.println("  s  stop");
  Serial.println("  f  forward 1 second (to measure distance)");
  Serial.println("  r  turn around once (to set TURN_TIME_MS)");
  Serial.println("==============================");
  Serial.println();
}

void loop() {
  keepNetworkAlive();
  handleSerial();
  runStateMachine();
}

// ==================================================================
//  NETWORK
// ==================================================================
void connectWiFi() {
  Serial.print("WiFi connecting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed, will keep retrying.");
  }

  digitalWrite(BLUE_LED, LOW);
}

void keepNetworkAlive() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastMqttTry > 10000) {
      lastMqttTry = millis();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    return;
  }

  if (mqtt.connected()) {
    mqtt.loop();
    return;
  }

  if (millis() - lastMqttTry < 5000) {
    return;
  }
  lastMqttTry = millis();

  if (mqtt.connect("car", TOPIC_PRESENCE, 0, true, "offline")) {
    mqtt.publish(TOPIC_PRESENCE, "online", true);
    mqtt.subscribe(TOPIC_COMMAND);
    publishState();
    Serial.println("MQTT connected.");
  }
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String text = "";
  for (unsigned int i = 0; i < length; i++) {
    text = text + (char)payload[i];
  }
  Serial.print("MQTT command: ");
  Serial.println(text);

  // payload looks like {"table":1}
  int table = 0;
  int p = text.indexOf(':');
  if (p >= 0) {
    table = text.substring(p + 1).toInt();
  }

  if (table == 1 || table == 2) {
    startTrip(table);
  }
}

void publishState() {
  String s = "idle";
  if (state == 1)      s = "moving";
  else if (state == 2) s = "at_table";
  else if (state == 3 || state == 4) s = "returning";

  if (mqtt.connected()) {
    mqtt.publish(TOPIC_STATE, s.c_str());
  }
}

// ==================================================================
//  SERIAL
// ==================================================================
void handleSerial() {
  if (!Serial.available()) {
    return;
  }

  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();
  if (input.length() == 0) {
    return;
  }

  char c = input.charAt(0);

  if (c == '1')      startTrip(1);
  else if (c == '2') startTrip(2);
  else if (c == 's') stopTrip("stopped by hand");
  else if (c == 'f') {
    Serial.println("forward 1 second...");
    driveForward();
    delay(1000);
    stopMotors();
    Serial.println("done. Measure how far it went, then work out the");
    Serial.println("milliseconds you need for each table.");
  }
  else if (c == 'r') {
    Serial.print("turning for ");
    Serial.print(TURN_TIME_MS);
    Serial.println(" ms...");
    doTurn();
    delay(TURN_TIME_MS);
    stopMotors();
    Serial.println("done. Is it facing exactly backwards?");
    Serial.println("Not far enough -> raise TURN_TIME_MS.");
    Serial.println("Too far        -> lower it.");
  }
  else {
    Serial.println("commands: 1  2  s  f  r");
  }
}

// ==================================================================
//  THE TRIP
// ==================================================================
void startTrip(int table) {
  if (state != 0) {
    Serial.println("Already running, ignoring.");
    return;
  }

  targetTable = table;
  forwardTime = (table == 1) ? TABLE1_FORWARD_MS : TABLE2_FORWARD_MS;
  returnTime  = forwardTime * RETURN_PERCENT / 100;

  Serial.print(">> TRIP to table ");
  Serial.print(table);
  Serial.print(", out ");
  Serial.print(forwardTime);
  Serial.print(" ms, back ");
  Serial.print(returnTime);
  Serial.println(" ms");

  beep(1);
  changeState(1);
}

void stopTrip(String reason) {
  state = 0;
  stopMotors();
  blueOff();
  publishState();
  Serial.print(">> STOPPED: ");
  Serial.println(reason);
}

void changeState(int newState) {
  state = newState;
  stateStart = millis();

  if (state == 1) {
    Serial.println("  driving out");
  }
  else if (state == 2) {
    Serial.print("  at the table, waiting ");
    Serial.print(WAIT_AT_TABLE_MS);
    Serial.println(" ms with the blue LED blinking");
    stopMotors();
    beep(2);
    lastBlink = millis();
    blueOn = true;
    digitalWrite(BLUE_LED, HIGH);
  }
  else if (state == 3) {
    Serial.println("  turning around");
  }
  else if (state == 4) {
    Serial.println("  driving back");
  }

  publishState();
}

void runStateMachine() {
  if (state == 0) {
    return;
  }

  unsigned long elapsed = millis() - stateStart;

  // ---------------- 1. driving out ----------------
  if (state == 1) {
    driveForward();
    if (elapsed >= (unsigned long)forwardTime) {
      changeState(2);
    }
    return;
  }

  // ---------------- 2. waiting at the table ----------------
  if (state == 2) {
    stopMotors();

    // blink the blue LED for the whole wait
    if (millis() - lastBlink >= (unsigned long)BLINK_INTERVAL_MS) {
      lastBlink = millis();
      blueOn = !blueOn;
      digitalWrite(BLUE_LED, blueOn ? HIGH : LOW);
    }

    if (elapsed >= (unsigned long)WAIT_AT_TABLE_MS) {
      blueOff();               // out before the car moves again
      changeState(3);
    }
    return;
  }

  // ---------------- 3. turning around ----------------
  if (state == 3) {
    doTurn();
    if (elapsed >= (unsigned long)TURN_TIME_MS) {
      changeState(4);
    }
    return;
  }

  // ---------------- 4. driving back ----------------
  // shorter than the way out, to cancel the drift from the turn
  if (state == 4) {
    driveForward();
    if (elapsed >= (unsigned long)returnTime) {
      beep(3);
      stopTrip("back at the station");
    }
    return;
  }
}

// ==================================================================
//  BLUE LED
// ==================================================================
void blueOff() {
  digitalWrite(BLUE_LED, LOW);
  blueOn = false;
}

// ==================================================================
//  MOTORS
//  Motor 1 is channel A, motor 2 is channel B.
//  Direction is inverted to match how the motors are wired.
// ==================================================================
void driveForward() {
  int leftSpeed  = DRIVE_SPEED - STRAIGHT_TRIM;
  int rightSpeed = DRIVE_SPEED + STRAIGHT_TRIM;

  if (leftSpeed < 0)   leftSpeed = 0;
  if (rightSpeed < 0)  rightSpeed = 0;
  if (leftSpeed > 255)  leftSpeed = 255;
  if (rightSpeed > 255) rightSpeed = 255;

  driveWheels(leftSpeed, rightSpeed);
}

void doTurn() {
  if (TURN_RIGHT) {
    pivotRight(TURN_SPEED);
  } else {
    pivotLeft(TURN_SPEED);
  }
}

void setupPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(CH_A, PWM_FREQ, PWM_BITS);
  ledcSetup(CH_B, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PWMA, CH_A);
  ledcAttachPin(PWMB, CH_B);
#endif
}

void pwmA(int speed) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWMA, speed);
#else
  ledcWrite(CH_A, speed);
#endif
}

void pwmB(int speed) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PWMB, speed);
#else
  ledcWrite(CH_B, speed);
#endif
}

void stopMotors() {
  pwmA(0);
  pwmB(0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, LOW);
}

void motor1Forward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  pwmA(speed);
}

void motor1Backward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  pwmA(speed);
}

void motor2Forward(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  pwmB(speed);
}

void motor2Backward(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  pwmB(speed);
}

void driveWheels(int leftSpeed, int rightSpeed) {
  digitalWrite(STBY, HIGH);

  if (MOTOR1_IS_LEFT) {
    motor1Forward(leftSpeed);
    motor2Forward(rightSpeed);
  } else {
    motor1Forward(rightSpeed);
    motor2Forward(leftSpeed);
  }
}

void pivotRight(int speed) {
  digitalWrite(STBY, HIGH);

  if (MOTOR1_IS_LEFT) {
    motor1Forward(speed);
    motor2Backward(speed);
  } else {
    motor1Backward(speed);
    motor2Forward(speed);
  }
}

void pivotLeft(int speed) {
  digitalWrite(STBY, HIGH);

  if (MOTOR1_IS_LEFT) {
    motor1Backward(speed);
    motor2Forward(speed);
  } else {
    motor1Forward(speed);
    motor2Backward(speed);
  }
}

// ==================================================================
//  BUZZER
// ==================================================================
void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(80);
    digitalWrite(BUZZER, LOW);
    delay(80);
  }
}
