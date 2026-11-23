/*
  CAR - LINE FOLLOWING DELIVERY
  ESP32 + TB6612FNG + 8 channel IR array (6 channels used)

  What it does:
    1. Follows the black line away from the station.
    2. At the Y junction it takes the right branch for table 1,
       or the left branch for table 2.
    3. Keeps following until the line ends, which means it has
       reached the table. It stops there.
    4. Waits WAIT_AT_TABLE.
    5. Rotates on the spot until it finds the line again.
    6. Follows the line back. At the Y the branches merge, so no
       decision is needed on the way home.
    7. Stops when the line ends at the station.

  Wiring:
    TB6612FNG  PWMA->4  AIN2->16  AIN1->17  STBY->5
               BIN1->18 BIN2->19  PWMB->21
    IR array   6 channels on GPIO 32, 33, 35, 34, 36, 39
               IR1 and IR8 not connected, board IR pin to VCC

  Serial Monitor at 115200, line ending "Newline".
    1  start a run to table 1
    2  start a run to table 2
    s  stop
    c  print raw sensor values, use this to pick the threshold
    t 2000   set the threshold to 2000
*/

// ================= SETTINGS - CHANGE THESE =================

// --- which way each table is from the junction ---
bool TABLE1_IS_RIGHT = true;    // table 1 takes the right branch

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
int ROTATE_SEARCH_MAX  = 8000;  // give up searching after this long
int STARTUP_IGNORE     = 600;   // ignore "line lost" for this long after starting

// ===========================================================

// ---------- motor pins ----------
#define PWMA 4
#define AIN2 16
#define AIN1 17
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

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
int targetTable = 0;
bool turningRight = true;       // which way the junction turn goes
bool junctionDone = false;      // has the branch already been taken

// last known side of the line, used when the line slips out of view
int lastError = 0;

// reading buffers
int  sensorValue[6];
bool onLine[6];
int  sensorsOnLine = 0;

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  stopMotors();

  // the IR pins are analog inputs, nothing to configure on ESP32
  analogReadResolution(12);     // values run 0 to 4095

  Serial.println();
  Serial.println("===== LINE FOLLOWING DELIVERY =====");
  Serial.println("  1        run to table 1");
  Serial.println("  2        run to table 2");
  Serial.println("  s        stop");
  Serial.println("  c        print raw sensor values");
  Serial.println("  t 2000   set the threshold");
  Serial.println("===================================");
  Serial.println();
}

void loop() {
  handleSerial();
  runStateMachine();
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

  changeState(1);
}

void stopRun(String reason) {
  state = 0;
  stopMotors();
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
  }
  else if (state == 4) {
    Serial.println("  rotating to find the line");
  }
  else if (state == 5) {
    Serial.println("  following line home");
  }
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
    if (!junctionDone && atJunction()) {
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

  // ---------------- rotating to find the line ----------------
  if (state == 4) {
    // spin the opposite way to the branch we took, that points back
    if (turningRight) {
      pivotLeft(ROTATE_SPEED);
    } else {
      pivotRight(ROTATE_SPEED);
    }

    // give it a moment to leave the line it is already sitting on
    if (elapsed < 500) {
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
void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, LOW);
}

void motor1Forward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, speed);
}

void motor1Backward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, speed);
}

void motor2Forward(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, speed);
}

void motor2Backward(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, speed);
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
