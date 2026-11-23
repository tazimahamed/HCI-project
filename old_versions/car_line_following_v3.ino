/*
  CAR - LINE FOLLOWING DELIVERY  (WiFi + MQTT)   v3
  ESP32 + TB6612FNG + 8 channel IR array (6 channels used) + buzzer

  TRACK (Y shape):

        table 2  <--- left branch \        / right branch ---> table 1
                                   \      /
                                    junction
                                       |
                                      stem
                                       |
                                    STATION (home)

  HOW IT BEHAVES
    out    : follow the line. At the fork it does NOT stop and does NOT
             do a blind timed turn. It simply stops looking at the
             sensors on the branch it is not taking, so the normal
             steering carries it onto the branch it wants. Once the
             line is narrow again it goes back to using all sensors.
    table  : the line ends, so it stops and waits 2 seconds.
    home   : it rotates until it has first left the line and then found
             it again with a middle sensor, then follows the line back.
             The branches merge at the Y so there is no choice to make.
             It stops when the line ends at the station.

  There are no drive-by-time steps anywhere. The only timings left are
  the 2 second wait at the table and a short debounce that decides when
  a missing line really means the line has ended.

  Wiring:
    TB6612FNG  PWMA->4  AIN2->16  AIN1->17  STBY->5
               BIN1->18 BIN2->19  PWMB->21
    IR array   6 channels on GPIO 32, 33, 35, 34, 36, 39
    Buzzer     GPIO 12

  MQTT:
    subscribe   restaurant/car/command    {"table":1} or {"table":2}
    publish     restaurant/car/state      idle / moving / at_table / returning
    publish     restaurant/car/presence   online / offline (last will)

  Serial Monitor at 115200, line ending "Newline".
    1  run to table 1        2  run to table 2      s  stop
    c  print raw sensors     t 2000  set threshold
    d  toggle live debug printing
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

// ================= SETTINGS =================

// table 1 = RIGHT branch, table 2 = LEFT branch
bool TABLE1_IS_RIGHT = true;

// Sensors listed physically LEFT to RIGHT as they sit on the car.
// If steering fights the line instead of correcting it, this order is wrong.
int sensorPins[6] = { 32, 33, 35, 34, 36, 39 };

bool MOTOR1_IS_LEFT = true;

// true  = black line reads HIGHER than the floor
// false = black line reads LOWER than the floor
bool LINE_IS_HIGH = true;

int LINE_THRESHOLD = 2000;

// --- speeds ---
int BASE_SPEED   = 140;
int MAX_SPEED    = 220;
int ROTATE_SPEED = 130;   // used only while turning around at the table

// --- steering ---
int STEER_GAIN = 14;

// --- the only timings in the sketch ---
int WAIT_AT_TABLE     = 2000;   // pause at the table
int LINE_LOST_CONFIRM = 250;    // line missing this long = the line has ended
int STARTUP_IGNORE    = 500;    // ignore "line lost" this long after starting

// How wide the line has to look before the car believes it is at the fork.
// Raise to 5 if a thick straight line ever triggers it by mistake.
int JUNCTION_MIN_SENSORS = 4;

// ===========================================================

// ---------- motor pins ----------
#define PWMA 4
#define AIN2 16
#define AIN1 17
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

#define BUZZER 12

// ---------- PWM ----------
#define CH_A 0
#define CH_B 1
#define PWM_FREQ 1000
#define PWM_BITS 8

// leftmost sensor is -5, rightmost is +5
int sensorWeight[6] = { -5, -3, -1, 1, 3, 5 };

// ---------- states ----------
// 0 idle
// 1 following out to the table
// 2 waiting at the table
// 3 turning around
// 4 following home
int state = 0;

unsigned long stateStart = 0;
unsigned long lineLostSince = 0;

int  targetTable = 0;
bool turningRight = true;      // which branch this run takes
bool takingBranch = false;     // currently steering through the fork
bool junctionDone = false;     // the fork has already been passed
bool leftTheLine = false;      // used by the turn around

int lastError = 0;
bool debugOn = false;
unsigned long lastDebug = 0;

int  sensorValue[6];
bool onLine[6];
int  sensorsOnLine = 0;

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

  setupPwm();
  stopMotors();

  analogReadResolution(12);     // 0 to 4095

  connectWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  Serial.println();
  Serial.println("===== LINE FOLLOWING DELIVERY v3 =====");
  Serial.println("  1  table 1 (right)   2  table 2 (left)");
  Serial.println("  s  stop   c  sensors   t 2000  threshold   d  debug");
  Serial.println("======================================");
  Serial.println();
}

void loop() {
  keepNetworkAlive();
  handleSerial();
  runStateMachine();
  printDebug();
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
    startRun(table);
  }
}

void publishState() {
  String s = "idle";
  if (state == 1)                    s = "moving";
  else if (state == 2)               s = "at_table";
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
  if (input.length() == 0) {
    return;
  }
  input.toLowerCase();

  char command = input.charAt(0);

  String argument = "";
  if (input.length() > 1) {
    argument = input.substring(1);
    argument.trim();
  }

  if (command == '1')      startRun(1);
  else if (command == '2') startRun(2);
  else if (command == 's') stopRun("stopped by hand");
  else if (command == 'c') printSensors();
  else if (command == 'd') {
    debugOn = !debugOn;
    Serial.print("debug ");
    Serial.println(debugOn ? "on" : "off");
  }
  else if (command == 't') {
    if (argument.length() == 0) {
      Serial.print("Threshold is ");
      Serial.println(LINE_THRESHOLD);
    } else {
      LINE_THRESHOLD = argument.toInt();
      Serial.print("Threshold set to ");
      Serial.println(LINE_THRESHOLD);
    }
  }
  else {
    Serial.println("Type 1, 2, s, c, d, or t 2000");
  }
}

void printSensors() {
  readSensors();

  Serial.print("raw: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorValue[i]);
    Serial.print(" ");
  }

  Serial.print("   on line: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(onLine[i] ? "X" : ".");
  }

  Serial.print("   count: ");
  Serial.println(sensorsOnLine);
}

void printDebug() {
  if (!debugOn || state == 0) {
    return;
  }
  if (millis() - lastDebug < 150) {
    return;
  }
  lastDebug = millis();

  Serial.print("s");
  Serial.print(state);
  Serial.print(takingBranch ? "B" : " ");
  Serial.print(" [");
  for (int i = 0; i < 6; i++) {
    Serial.print(onLine[i] ? "X" : ".");
  }
  Serial.print("] n=");
  Serial.print(sensorsOnLine);
  Serial.print(" err=");
  Serial.println(lastError);
}

// ==================================================================
//  SENSORS
// ==================================================================
void readSensors() {
  sensorsOnLine = 0;

  for (int i = 0; i < 6; i++) {
    sensorValue[i] = analogRead(sensorPins[i]);

    if (LINE_IS_HIGH) {
      onLine[i] = (sensorValue[i] > LINE_THRESHOLD);
    } else {
      onLine[i] = (sensorValue[i] < LINE_THRESHOLD);
    }

    if (onLine[i]) {
      sensorsOnLine = sensorsOnLine + 1;
    }
  }
}

// only the two middle sensors
bool centreOnLine() {
  return onLine[2] || onLine[3];
}

// The fork is only called a fork when the line looks genuinely wide.
// A single thick straight line must never reach this count, which is
// what to check with the 'c' command before a run.
bool atJunction() {
  if (sensorsOnLine >= JUNCTION_MIN_SENSORS) {
    return true;
  }
  if (onLine[0] && onLine[5]) {
    return true;
  }
  return false;
}

// Steering error. While going through the fork, the sensors on the side
// we are not taking are ignored, so the average pulls the car onto the
// branch we want without any turn-for-N-milliseconds step.
int getError() {
  int total = 0;
  int count = 0;

  for (int i = 0; i < 6; i++) {
    if (!onLine[i]) {
      continue;
    }
    if (takingBranch) {
      if (turningRight && sensorWeight[i] < 0) continue;   // ignore left half
      if (!turningRight && sensorWeight[i] > 0) continue;  // ignore right half
    }
    total = total + sensorWeight[i];
    count = count + 1;
  }

  if (count == 0) {
    return lastError;      // keep steering the way the line last went
  }

  int error = total / count;
  lastError = error;
  return error;
}

// ==================================================================
//  THE RUN
// ==================================================================
void startRun(int table) {
  if (state != 0) {
    Serial.println("Already running, ignoring.");
    return;
  }

  targetTable = table;
  junctionDone = false;
  takingBranch = false;
  leftTheLine = false;
  lastError = 0;
  lineLostSince = 0;

  turningRight = (table == 1) ? TABLE1_IS_RIGHT : !TABLE1_IS_RIGHT;

  Serial.print(">> RUN to table ");
  Serial.print(table);
  Serial.print(", branch is ");
  Serial.println(turningRight ? "RIGHT" : "LEFT");

  beep(1);
  changeState(1);
}

void stopRun(String reason) {
  state = 0;
  takingBranch = false;
  stopMotors();
  publishState();
  Serial.print(">> STOPPED: ");
  Serial.println(reason);
}

void changeState(int newState) {
  state = newState;
  stateStart = millis();
  lineLostSince = 0;

  if (state == 1) {
    lastError = 0;
    Serial.println("  following line out");
  }
  else if (state == 2) {
    Serial.println("  at the table, waiting 2s");
    stopMotors();
    beep(2);
  }
  else if (state == 3) {
    leftTheLine = false;
    Serial.println("  turning around");
  }
  else if (state == 4) {
    lastError = 0;
    takingBranch = false;
    Serial.println("  following line home");
  }

  publishState();
}

void runStateMachine() {
  if (state == 0) {
    return;
  }

  readSensors();

  unsigned long elapsed = millis() - stateStart;

  // ---------------- 1. following out to the table ----------------
  if (state == 1) {

    // enter branch mode when the fork appears
    if (!junctionDone && !takingBranch && atJunction()) {
      takingBranch = true;
      Serial.println("  fork seen, steering onto the branch");
    }

    // leave branch mode once the line is a single narrow line again
    if (takingBranch && sensorsOnLine > 0 && sensorsOnLine <= 2 && centreOnLine()) {
      takingBranch = false;
      junctionDone = true;
      Serial.println("  on the branch");
    }

    if (sensorsOnLine == 0) {
      if (elapsed < (unsigned long)STARTUP_IGNORE) {
        followLine();
        return;
      }
      if (lineLostSince == 0) {
        lineLostSince = millis();
      }
      if (millis() - lineLostSince > (unsigned long)LINE_LOST_CONFIRM) {
        takingBranch = false;
        changeState(2);          // the line ended, this is the table
        return;
      }
    } else {
      lineLostSince = 0;
    }

    followLine();
    return;
  }

  // ---------------- 2. waiting at the table ----------------
  if (state == 2) {
    stopMotors();
    if (elapsed >= (unsigned long)WAIT_AT_TABLE) {
      changeState(3);
    }
    return;
  }

  // ---------------- 3. turning around ----------------
  // No fixed spin time. It has to lose the line completely first, which
  // proves it really has swung round, and only then does a middle sensor
  // finding the line count as facing back down the branch.
  if (state == 3) {
    if (turningRight) {
      pivotLeft(ROTATE_SPEED);
    } else {
      pivotRight(ROTATE_SPEED);
    }

    if (!leftTheLine) {
      if (sensorsOnLine == 0) {
        leftTheLine = true;
      }
      return;
    }

    if (centreOnLine()) {
      Serial.println("  line found, heading home");
      changeState(4);
    }
    return;
  }

  // ---------------- 4. following home ----------------
  // the branches merge into the stem, so nothing to decide
  if (state == 4) {
    if (sensorsOnLine == 0) {
      if (elapsed < (unsigned long)STARTUP_IGNORE) {
        followLine();
        return;
      }
      if (lineLostSince == 0) {
        lineLostSince = millis();
      }
      if (millis() - lineLostSince > (unsigned long)LINE_LOST_CONFIRM) {
        beep(3);
        stopRun("home, line ended");
        return;
      }
    } else {
      lineLostSince = 0;
    }

    followLine();
    return;
  }
}

// ==================================================================
//  LINE FOLLOWING
// ==================================================================
void followLine() {
  int error = getError();
  int correction = error * STEER_GAIN;

  int leftSpeed  = BASE_SPEED + correction;
  int rightSpeed = BASE_SPEED - correction;

  if (leftSpeed > MAX_SPEED)  leftSpeed = MAX_SPEED;
  if (rightSpeed > MAX_SPEED) rightSpeed = MAX_SPEED;
  if (leftSpeed < 0)  leftSpeed = 0;
  if (rightSpeed < 0) rightSpeed = 0;

  driveWheels(leftSpeed, rightSpeed);
}

// ==================================================================
//  MOTORS
//  Motor 1 is channel A, motor 2 is channel B.
//  Direction is inverted to match how the motors are wired.
// ==================================================================
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
