/*
  CAR - LINE FOLLOWING DELIVERY  (WiFi + MQTT version)
  ESP32 + TB6612FNG + 8 channel IR array (6 channels used) + buzzer

  TRACK (Y shape):

        table 2  <--- left branch \        / right branch ---> table 1
                                   \      /
                                    \    /
                                   junction
                                       |
                                      stem
                                       |
                                    STATION (home)

  RUN TO TABLE 1 : stem -> junction -> RIGHT branch -> line ends -> stop at table
  RUN TO TABLE 2 : stem -> junction -> LEFT  branch -> line ends -> stop at table
  RETURN         : wait -> spin 180 -> follow line back (branches merge at the Y,
                   no decision needed) -> line ends at station -> stop

  Wiring:
    TB6612FNG  PWMA->4  AIN2->16  AIN1->17  STBY->5
               BIN1->18 BIN2->19  PWMB->21
    IR array   6 channels on GPIO 32, 33, 35, 34, 36, 39
               IR1 and IR8 not connected, board IR pin to VCC
    Buzzer     GPIO 12

  MQTT:
    subscribe   restaurant/car/command    {"table":1} or {"table":2}
    publish     restaurant/car/state      idle / moving / at_table / returning
    publish     restaurant/car/presence   online / offline (last will)

  Serial Monitor at 115200, line ending "Newline".
    1  start a run to table 1
    2  start a run to table 2
    s  stop
    c  print raw sensor values, use this to pick the threshold
    t 2000   set the threshold to 2000
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

// ================= SETTINGS - CHANGE THESE =================

// --- which way each table is from the junction ---
// table 1 = RIGHT branch, table 2 = LEFT branch
bool TABLE1_IS_RIGHT = true;

// --- sensor order ---
// These must be listed physically LEFT to RIGHT as they sit on the car.
// If steering fights the line instead of correcting it, this order is wrong.
int sensorPins[6] = { 32, 33, 35, 34, 36, 39 };

// --- which motor is the left wheel ---
bool MOTOR1_IS_LEFT = true;

// --- how the sensors read the line ---
// true  = the black line gives a HIGHER analog value than the floor
// false = the black line gives a LOWER analog value
bool LINE_IS_HIGH = true;

int LINE_THRESHOLD = 2000;      // analog value that separates line from floor

// --- speeds ---
int BASE_SPEED   = 140;         // normal following speed, 0 to 255
int MAX_SPEED    = 220;         // cap after steering correction
int TURN_SPEED   = 150;         // speed while taking a branch at the junction
int ROTATE_SPEED = 130;         // speed while spinning to find the line

// --- steering ---
int STEER_GAIN = 14;            // higher turns harder, lower is smoother

// --- timings in milliseconds ---
int JUNCTION_TURN_TIME = 700;   // how long to commit to a branch at the Y
int WAIT_AT_TABLE      = 5000;  // pause at the table
int LINE_LOST_CONFIRM  = 300;   // line must be missing this long to count as lost
int ROTATE_MIN_TIME    = 900;   // minimum spin time, makes sure it turns around
int ROTATE_SEARCH_MAX  = 8000;  // give up searching after this long
int STARTUP_IGNORE     = 600;   // ignore "line lost" for this long after starting
int JUNCTION_IGNORE    = 900;   // after a branch turn, ignore junction for this long

// ===========================================================

// ---------- motor pins ----------
#define PWMA 4
#define AIN2 16
#define AIN1 17
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

// ---------- buzzer ----------
#define BUZZER 12

// ---------- PWM channels (ESP32 LEDC) ----------
#define CH_A 0
#define CH_B 1
#define PWM_FREQ 1000
#define PWM_BITS 8

// ---------- weights used to work out how far off the line we are ----------
// leftmost sensor is -5, rightmost is +5
int sensorWeight[6] = { -5, -3, -1, 1, 3, 5 };

// ---------- what the car is doing ----------
// 0 = idle
// 1 = following the line out to the table
// 2 = turning onto a branch at the junction
// 3 = waiting at the table
// 4 = rotating to find the line again
// 5 = following the line home
int state = 0;

unsigned long stateStart = 0;
unsigned long lineLostSince = 0;
int  targetTable = 0;
bool turningRight = true;       // which way the junction turn goes
bool junctionDone = false;      // has the branch already been taken

// last known side of the line, used when the line slips out of view
int lastError = 0;

// reading buffers
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

  // the IR pins are analog inputs, nothing to configure on ESP32
  analogReadResolution(12);     // values run 0 to 4095

  connectWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  Serial.println();
  Serial.println("===== LINE FOLLOWING DELIVERY =====");
  Serial.println("  1        run to table 1 (right branch)");
  Serial.println("  2        run to table 2 (left branch)");
  Serial.println("  s        stop");
  Serial.println("  c        print raw sensor values");
  Serial.println("  t 2000   set the threshold");
  Serial.println("===================================");
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

  // the last will tells the kitchen we died if power or wifi is lost
  if (mqtt.connect("car", TOPIC_PRESENCE, 0, true, "offline")) {
    mqtt.publish(TOPIC_PRESENCE, "online", true);
    mqtt.subscribe(TOPIC_COMMAND);
    publishState();
    Serial.println("MQTT connected.");
  }
}

// commands from the kitchen laptop arrive here
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String text = "";
  for (unsigned int i = 0; i < length; i++) {
    text = text + (char)payload[i];
  }
  Serial.print("MQTT command: ");
  Serial.println(text);

  // payload looks like {"table":1}
  int table = 0;
  if (text.indexOf("1") >= 0 && text.indexOf("\"table\"") >= 0) {
    int p = text.indexOf(":");
    if (p >= 0) {
      table = text.substring(p + 1).toInt();
    }
  } else {
    int p = text.indexOf(":");
    if (p >= 0) {
      table = text.substring(p + 1).toInt();
    }
  }

  if (table == 1 || table == 2) {
    startRun(table);
  }
}

void publishState() {
  String s = "idle";
  if (state == 1 || state == 2) s = "moving";
  else if (state == 3)          s = "at_table";
  else if (state == 4 || state == 5) s = "returning";

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

  if (command == '1') {
    startRun(1);
  }
  else if (command == '2') {
    startRun(2);
  }
  else if (command == 's') {
    stopRun("stopped by hand");
  }
  else if (command == 'c') {
    printSensors();
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
    Serial.println("Type 1, 2, s, c, or t 2000");
  }
}

// shows what each sensor sees right now, for setting the threshold
void printSensors() {
  readSensors();

  Serial.print("raw: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorValue[i]);
    Serial.print(" ");
  }

  Serial.print("   on line: ");
  for (int i = 0; i < 6; i++) {
    if (onLine[i]) {
      Serial.print("X");
    } else {
      Serial.print(".");
    }
  }

  Serial.print("   count: ");
  Serial.println(sensorsOnLine);
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

// how far off centre the line is, negative means the line is to the left
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

// a Y junction shows up as the line suddenly getting much wider
bool atJunction() {
  if (sensorsOnLine >= 4) {
    return true;
  }
  // or as line on both outer edges at once
  if (onLine[0] && onLine[5]) {
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
  lastError = 0;
  lineLostSince = 0;

  // table 1 -> right branch, table 2 -> left branch
  if (table == 1) {
    turningRight = TABLE1_IS_RIGHT;
  } else {
    turningRight = !TABLE1_IS_RIGHT;
  }

  Serial.print(">> RUN to table ");
  Serial.print(table);
  Serial.print(", junction turn is ");
  if (turningRight) {
    Serial.println("RIGHT");
  } else {
    Serial.println("LEFT");
  }

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
    Serial.println("  following line out");
  }
  else if (state == 2) {
    Serial.println("  taking the branch at the junction");
  }
  else if (state == 3) {
    Serial.println("  at the table, waiting");
    stopMotors();
    beep(2);
  }
  else if (state == 4) {
    Serial.println("  turning around to find the line");
  }
  else if (state == 5) {
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

  // ---------------- following out to the table ----------------
  if (state == 1) {
    // take the branch when the junction appears
    // (ignored for a moment right after a branch turn, so the wide
    //  patch we just drove over is not read as a second junction)
    if (!junctionDone && elapsed > (unsigned long)JUNCTION_IGNORE && atJunction()) {
      junctionDone = true;
      changeState(2);
      return;
    }

    if (sensorsOnLine == 0) {
      // the line has gone, but ignore it briefly after starting
      if (elapsed < (unsigned long)STARTUP_IGNORE) {
        followLine();
        return;
      }
      if (lineLostSince == 0) {
        lineLostSince = millis();
      }
      if (millis() - lineLostSince > (unsigned long)LINE_LOST_CONFIRM) {
        // line ended = arrived at the table
        changeState(3);
        return;
      }
    } else {
      lineLostSince = 0;
    }

    followLine();
    return;
  }

  // ---------------- committing to a branch ----------------
  if (state == 2) {
    if (turningRight) {
      pivotRight(TURN_SPEED);
    } else {
      pivotLeft(TURN_SPEED);
    }

    if (elapsed >= (unsigned long)JUNCTION_TURN_TIME) {
      changeState(1);
    }
    return;
  }

  // ---------------- waiting at the table ----------------
  if (state == 3) {
    stopMotors();
    if (elapsed >= (unsigned long)WAIT_AT_TABLE) {
      changeState(4);
    }
    return;
  }

  // ---------------- turning around to find the line ----------------
  if (state == 4) {
    // spin the opposite way to the branch we took, that points back
    // down the branch towards the junction
    if (turningRight) {
      pivotLeft(ROTATE_SPEED);
    } else {
      pivotRight(ROTATE_SPEED);
    }

    // keep spinning for a minimum time so it really turns around
    // instead of locking onto the same line it was already sitting on
    if (elapsed < (unsigned long)ROTATE_MIN_TIME) {
      return;
    }

    if (sensorsOnLine > 0) {
      Serial.println("  line found");
      changeState(5);
      return;
    }

    if (elapsed >= (unsigned long)ROTATE_SEARCH_MAX) {
      stopRun("no line found while searching");
    }
    return;
  }

  // ---------------- following home ----------------
  // on the way back the two branches merge into the stem, so there is
  // no decision to make. Just follow until the line ends at the station.
  if (state == 5) {
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

// drives the wheels by side rather than by channel
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

// spins on the spot to the right
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

// spins on the spot to the left
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
