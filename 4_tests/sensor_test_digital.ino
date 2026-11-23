/*
  SENSOR TEST - digital version

  This board has D1..D8 outputs, not analog ones, so the pins are read
  with digitalRead. Each channel is either 0 or 1, nothing in between.

  Wiring being tested (6 of the 8 channels):
    GPIO 32, 33, 35, 34, 36, 39

  Upload this, open Serial Monitor at 115200, and watch the line of
  0s and 1s. Then:

    - hold the car over the WHITE floor   -> note the pattern
    - hold the car over the BLACK tape    -> note the pattern
    - wave your hand under the sensors    -> the numbers must change

  If the numbers NEVER change no matter what is underneath, the
  sensors are not seeing anything and it is a hardware problem, not
  a code problem.
*/

int sensorPins[6] = { 32, 33, 35, 34, 36, 39 };

unsigned long lastPrint = 0;

void setup() {
  Serial.begin(115200);
  delay(400);

  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.println();
  Serial.println("========== SENSOR TEST (digital) ==========");
  Serial.println("Watch the six numbers.");
  Serial.println("  over white floor -> one pattern");
  Serial.println("  over black tape  -> the other pattern");
  Serial.println("If they never change, the sensors are not working.");
  Serial.println("===========================================");
  Serial.println();
}

void loop() {
  if (millis() - lastPrint < 250) {
    return;
  }
  lastPrint = millis();

  Serial.print("GPIO 32 33 35 34 36 39   =   ");

  for (int i = 0; i < 6; i++) {
    int value = digitalRead(sensorPins[i]);
    Serial.print(value);
    Serial.print("  ");
  }

  Serial.println();
}
