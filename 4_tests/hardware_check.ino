/*
  HARDWARE CHECK - run this BEFORE the delivery sketch

  This finds the three settings that have to be right before any
  line following can work:

      1. MOTOR1_IS_LEFT      is channel A the left wheel
      2. LINE_IS_HIGH        does black read higher or lower than the floor
      3. sensorPins order    are they listed left to right

  Wiring is the same as the delivery sketch.
    TB6612FNG  PWMA->4  AIN2->16  AIN1->17  STBY->5
               BIN1->18 BIN2->19  PWMB->21
    IR array   GPIO 32, 33, 35, 34, 36, 39

  Serial Monitor at 115200, line ending "Newline".

    m1   run motor 1 (channel A) forward for 1 second
    m2   run motor 2 (channel B) forward for 1 second
    f    run both forward for 1 second
    p    pivot right for 1 second
    x    stop everything
    s    start / stop the live sensor printout
*/

// ---------- motor pins ----------
#define PWMA 4
#define AIN2 16
#define AIN1 17
#define STBY 5
#define BIN1 18
#define BIN2 19
#define PWMB 21

#define CH_A 0
#define CH_B 1
#define PWM_FREQ 1000
#define PWM_BITS 8

#define TEST_SPEED 150

// ---------- sensors ----------
int sensorPins[6] = { 32, 33, 35, 34, 36, 39 };
int sensorValue[6];

bool printing = false;
unsigned long lastPrint = 0;

// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  setupPwm();
  stopMotors();

  analogReadResolution(12);

  Serial.println();
  Serial.println("================ HARDWARE CHECK ================");
  Serial.println();
  Serial.println("STEP 1 - MOTORS");
  Serial.println("  Lift the car off the floor.");
  Serial.println("  Type m1  -> note which wheel spins, and which way.");
  Serial.println("  Type m2  -> note the other wheel.");
  Serial.println("  Type f   -> both wheels must spin FORWARD.");
  Serial.println();
  Serial.println("  If m1 spins the LEFT wheel   -> MOTOR1_IS_LEFT = true");
  Serial.println("  If m1 spins the RIGHT wheel  -> MOTOR1_IS_LEFT = false");
  Serial.println("  If f drives BACKWARD, swap the Forward and Backward");
  Serial.println("  bodies of motor1/motor2 in the delivery sketch.");
  Serial.println();
  Serial.println("STEP 2 - SENSORS");
  Serial.println("  Type s, then put the car on the floor OFF the line");
  Serial.println("  and read the six numbers. Then put it ON the black");
  Serial.println("  line and read them again.");
  Serial.println();
  Serial.println("  black HIGHER than floor -> LINE_IS_HIGH = true");
  Serial.println("  black LOWER  than floor -> LINE_IS_HIGH = false");
  Serial.println("  THRESHOLD = halfway between the two readings.");
  Serial.println();
  Serial.println("STEP 3 - SENSOR ORDER");
  Serial.println("  With s still running, slide a strip of black tape");
  Serial.println("  slowly under the car from LEFT to RIGHT.");
  Serial.println("  The marked column must move left to right too.");
  Serial.println("  If it jumps around, reorder sensorPins[] until it");
  Serial.println("  sweeps cleanly, and use that order in the sketch.");
  Serial.println();
  Serial.println("================================================");
  Serial.println();
}

void loop() {
  handleSerial();
  livePrint();
}

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

  if (input == "m1") {
    Serial.println("motor 1 (channel A) forward...");
    motor1Forward(TEST_SPEED);
    delay(1000);
    stopMotors();
    Serial.println("done. Which wheel was it, and did it go forward?");
  }
  else if (input == "m2") {
    Serial.println("motor 2 (channel B) forward...");
    motor2Forward(TEST_SPEED);
    delay(1000);
    stopMotors();
    Serial.println("done.");
  }
  else if (input == "f") {
    Serial.println("both forward...");
    digitalWrite(STBY, HIGH);
    motor1Forward(TEST_SPEED);
    motor2Forward(TEST_SPEED);
    delay(1000);
    stopMotors();
    Serial.println("done. Did the car move FORWARD?");
  }
  else if (input == "p") {
    Serial.println("pivot right...");
    digitalWrite(STBY, HIGH);
    motor1Forward(TEST_SPEED);
    motor2Backward(TEST_SPEED);
    delay(1000);
    stopMotors();
    Serial.println("done. Did it spin to the RIGHT?");
    Serial.println("If it spun LEFT, MOTOR1_IS_LEFT is the wrong way round.");
  }
  else if (input == "x") {
    stopMotors();
    Serial.println("stopped.");
  }
  else if (input == "s") {
    printing = !printing;
    Serial.println(printing ? "sensor print ON" : "sensor print OFF");
  }
  else {
    Serial.println("commands: m1  m2  f  p  x  s");
  }
}

// ==================================================================
// prints the six raw values plus a bar for each, so a strip of tape
// sliding under the car is easy to follow
void livePrint() {
  if (!printing) {
    return;
  }
  if (millis() - lastPrint < 200) {
    return;
  }
  lastPrint = millis();

  int lowest = 4095;
  int highest = 0;

  for (int i = 0; i < 6; i++) {
    sensorValue[i] = analogRead(sensorPins[i]);
    if (sensorValue[i] < lowest)  lowest = sensorValue[i];
    if (sensorValue[i] > highest) highest = sensorValue[i];
  }

  for (int i = 0; i < 6; i++) {
    Serial.print("GPIO");
    Serial.print(sensorPins[i]);
    Serial.print(":");
    if (sensorValue[i] < 1000) Serial.print(" ");
    Serial.print(sensorValue[i]);

    // mark the odd one out, that is where the tape is
    if (highest - lowest > 500) {
      if (sensorValue[i] == highest) {
        Serial.print(" <HIGH");
      } else if (sensorValue[i] == lowest) {
        Serial.print(" <LOW ");
      } else {
        Serial.print("      ");
      }
    } else {
      Serial.print("      ");
    }
    Serial.print("  ");
  }
  Serial.println();
}

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

// same inverted wiring as the delivery sketch
void motor1Forward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  pwmA(speed);
}

void motor1Backward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  pwmA(speed);
}

void motor2Forward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  pwmB(speed);
}

void motor2Backward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  pwmB(speed);
}
