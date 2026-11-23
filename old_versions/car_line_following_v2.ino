/*
  CAR - LINE FOLLOWING DELIVERY  (WiFi + MQTT)   v2
  ESP32 + TB6612FNG + 8 channel IR array (6 channels used) + buzzer

  TRACK (Y shape):

        table 2  <--- left branch \        / right branch ---> table 1
                                   \      /
                                    junction
                                       |
                                      stem
                                       |
                                    STATION (home)

  RUN TO TABLE 1 : stem -> junction -> RIGHT branch -> line ends -> stop
  RUN TO TABLE 2 : stem -> junction -> LEFT  branch -> line ends -> stop
  RETURN         : wait -> spin 180 -> follow line back (branches merge,
                   no decision) -> line ends at station -> stop

  WHAT CHANGED IN v2
    - Junction detection is no longer blocked at the start of a run.
      Previously the "ignore junction" timer ran from the start, so a
      short stem meant the junction was reached while detection was off
      and the car drove straight through it.
    - The branch turn is no longer a blind timed pivot. The car pivots
      until a CENTRE sensor sits on the branch line, with a minimum and
      a maximum time as guards.
    - The 180 degree turn at the table now only finishes when a CENTRE
      sensor sees the line. Before, any edge sensor brushing the line
      ended the spin half way round and the car went off the wrong way.
    - lastError is cleared whenever a new following state begins.

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
int BASE_SPEED   = 130;
int MAX_SPEED    = 220;
int TURN_SPEED   = 140;   // pivot speed while taking a branch
int ROTATE_SPEED = 130;   // pivot speed while turning around at the table

// --- steering ---
int STEER_GAIN = 16;

// --- timings in milliseconds ---
int JUNCTION_NUDGE     = 250;   // creep forward so the wheels sit on the fork
int TURN_MIN_TIME      = 250;   // pivot at least this long before looking
int TURN_MAX_TIME      = 1600;  // give up pivoting after this long
int JUNCTION_BLOCK     = 1200;  // after a branch turn, ignore junction this long
int WAIT_AT_TABLE      = 5000;
int LINE_LOST_CONFIRM  = 300;   // line missing this long = line has ended
int ROTATE_MIN_TIME    = 1100;  // minimum spin so it really turns around
int ROTATE_SEARCH_MAX  = 9000;
int STARTUP_IGNORE     = 600;   // ignore "line lost" this long after starting

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
// 2 creeping forward over the fork
// 3 pivoting onto the chosen branch
// 4 waiting at the table
// 5 turning around
// 6 following home
int state = 0;

unsigned long stateStart = 0;
unsigned long lineLostSince = 0;
unsigned long junctionBlockUntil = 0;   // absolute time, 0 = not blocked

int  targetTable = 0;
bool turningRight = true;
bool junctionDone = false;

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
  Serial.println("===== LINE FOLLOWING DELIVERY v2 =====");
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
  if (state == 1 || state == 2 || state == 3) s = "moving";
  else if (state == 4)                        s = "at_table";
  else if (state == 5 || state == 6)          s = "returning";

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

// live view while the car runs, so you can see what it saw at the junction
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

// only the two middle sensors. Used to decide when a pivot has actually
// put the car on the line, instead of an edge sensor just grazing it.
bool centreOnLine() {
  return onLine[2] || onLine[3];
}

int getError() {
  if (sensorsOnLine == 0) {
    return lastError;
  }

  int total = 0;
  for (int i = 0; i < 6; i++) {
    if (onLine[i]) {
      total = total + sensorWeight[i];
    }
  }

  int error = total / sensorsOnLine;
  lastError = error;
  return error;
}

// The Y fork shows up as the line suddenly getting much wider, or as
// both outer sensors seeing line at the same time.
bool atJunction() {
  if (sensorsOnLine >= 4) {
    return true;
  }
  if (onLine[0] && onLine[5]) {
    return true;
  }
  // one outer plus a centre: the fork arriving from one side first
  if ((onLine[0] || onLine[5]) && centreOnLine() && sensorsOnLine >= 3) {
    return true;
  }
  return false;
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
  junctionBlockUntil = 0;
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
    Serial.println("  junction seen, creeping onto the fork");
  }
  else if (state == 3) {
    Serial.print("  pivoting onto the ");
    Serial.print(turningRight ? "RIGHT" : "LEFT");
    Serial.println(" branch");
  }
  else if (state == 4) {
    Serial.println("  at the table, waiting");
    stopMotors();
    beep(2);
  }
  else if (state == 5) {
    Serial.println("  turning around");
  }
  else if (state == 6) {
    lastError = 0;
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
    // junction detection is only blocked for a moment AFTER a branch
    // turn, never at the start of the run
    bool blocked = (junctionBlockUntil != 0 && millis() < junctionBlockUntil);

    if (!junctionDone && !blocked && atJunction()) {
      changeState(2);
      return;
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
        // line ended = arrived at the table
        changeState(4);
        return;
      }
    } else {
      lineLostSince = 0;
    }

    followLine();
    return;
  }

  // ---------------- 2. creeping onto the fork ----------------
  // Move forward a little so the wheels are at the fork point before
  // pivoting. Pivoting too early turns on the stem and misses the branch.
  if (state == 2) {
    driveWheels(BASE_SPEED - 20, BASE_SPEED - 20);
    if (elapsed >= (unsigned long)JUNCTION_NUDGE) {
      changeState(3);
    }
    return;
  }

  // ---------------- 3. pivoting onto the branch ----------------
  if (state == 3) {
    if (turningRight) {
      pivotRight(TURN_SPEED);
    } else {
      pivotLeft(TURN_SPEED);
    }

    // do not look at the sensors immediately, the car is still sitting
    // on the fork it just drove over
    if (elapsed < (unsigned long)TURN_MIN_TIME) {
      return;
    }

    // finished when a CENTRE sensor is on the branch line
    if (centreOnLine()) {
      junctionDone = true;
      junctionBlockUntil = millis() + (unsigned long)JUNCTION_BLOCK;
      changeState(1);
      return;
    }

    if (elapsed >= (unsigned long)TURN_MAX_TIME) {
      // could not find the branch, stop instead of driving off
      stopRun("branch not found at the junction");
    }
    return;
  }

  // ---------------- 4. waiting at the table ----------------
  if (state == 4) {
    stopMotors();
    if (elapsed >= (unsigned long)WAIT_AT_TABLE) {
      changeState(5);
    }
    return;
  }

  // ---------------- 5. turning around ----------------
  if (state == 5) {
    // spin the opposite way to the branch we took, which points the car
    // back down the branch towards the junction
    if (turningRight) {
      pivotLeft(ROTATE_SPEED);
    } else {
      pivotRight(ROTATE_SPEED);
    }

    // must spin long enough to actually be facing back
    if (elapsed < (unsigned long)ROTATE_MIN_TIME) {
      return;
    }

    // only a CENTRE sensor counts. An edge sensor grazing the line part
    // way round is what used to send the car off in the wrong direction.
    if (centreOnLine()) {
      Serial.println("  line found, heading home");
      changeState(6);
      return;
    }

    if (elapsed >= (unsigned long)ROTATE_SEARCH_MAX) {
      stopRun("no line found while turning around");
    }
    return;
  }

  // ---------------- 6. following home ----------------
  // the branches merge into the stem on the way back, so there is no
  // decision to make. Follow until the line ends at the station.
  if (state == 6) {
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
