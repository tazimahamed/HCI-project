/*
  CARD IDENTIFIER TEST
  Tap any card and it says which table it belongs to and what it does.

  Hardware: ESP32 + RC522 RFID + 20x4 I2C LCD

  Wiring:
    RC522   SS -> 5 , SCK -> 18 , MOSI -> 23 , MISO -> 19 , RST -> 27 , 3.3V -> 3V3
    LCD     SDA -> 21 , SCL -> 22 , VCC -> VIN(5V)

  Libraries:
    "MFRC522" by GithubCommunity
    "LiquidCrystal I2C" by Frank de Brabander

  Serial Monitor at 115200.
*/

#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define SS_PIN  5
#define RST_PIN 27

MFRC522 rfid(SS_PIN, RST_PIN);
LiquidCrystal_I2C lcd(0x27, 20, 4);   // change 0x27 to 0x3F if screen is blank

// ---------- card UID tables ----------
String table1Cards[9] = {
  "50 33 7C 5C",   // START
  "50 1D BF 5C",   // CONFIRM
  "50 8B 68 5C",   // REDUCE
  "60 01 7E 5C",   // CALL CHEF
  "50 76 38 5C",   // BURGER
  "50 AB 0A 5C",   // PIZZA
  "50 6A 86 5C",   // FRIES
  "50 EF 9D 5C",   // PASTA
  "50 7E 65 5C"    // COFFEE
};

String table2Cards[9] = {
  "C0 06 21 5C",   // START
  "D3 5B D4 12",   // CONFIRM
  "C3 28 AA 34",   // REDUCE
  "A3 0D 99 35",   // CALL CHEF
  "23 03 A6 34",   // BURGER
  "73 91 9F 35",   // PIZZA
  "A3 54 88 35",   // FRIES
  "43 D2 A7 34",   // PASTA
  "A3 83 2E 30"    // COFFEE
};

// what each position in the list means
String roleNames[9] = {
  "START",
  "CONFIRM",
  "REDUCE",
  "CALL CHEF",
  "BURGER",
  "PIZZA",
  "FRIES",
  "PASTA",
  "COFFEE"
};

// ---------- card read filtering ----------
String lastUID = "";
unsigned long lastReadTime = 0;
int tapCounter = 0;

void setup() {
  Serial.begin(115200);
  delay(500);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(pad20("CARD IDENTIFIER"));
  lcd.setCursor(0, 1);
  lcd.print(pad20(""));
  lcd.setCursor(0, 2);
  lcd.print(pad20(""));
  lcd.setCursor(0, 3);
  lcd.print(pad20("Tap a card..."));

  SPI.begin();
  rfid.PCD_Init();
  delay(100);

  Serial.println();
  Serial.println("=================================");
  Serial.println("      CARD IDENTIFIER TEST");
  Serial.println("=================================");
  Serial.println("Tap any card to see what it is.");
  Serial.println();
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent()) {
    return;
  }
  if (!rfid.PICC_ReadCardSerial()) {
    return;
  }

  String uidText = readUidText();

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  // ignore the same card if tapped again too fast
  if (uidText == lastUID && millis() - lastReadTime < 1500) {
    return;
  }
  lastUID = uidText;
  lastReadTime = millis();

  tapCounter = tapCounter + 1;

  // look the card up in both tables
  int ownerTable = 0;
  int role = -1;

  for (int i = 0; i < 9; i++) {
    if (table1Cards[i] == uidText) {
      ownerTable = 1;
      role = i;
    }
    if (table2Cards[i] == uidText) {
      ownerTable = 2;
      role = i;
    }
  }

  // build the two lines we want to show
  String lineTable = "";
  String lineRole = "";

  if (role < 0) {
    lineTable = "UNKNOWN CARD";
    lineRole = "Not in either list";
  } else {
    lineTable = "TABLE " + String(ownerTable);
    lineRole = roleNames[role];

    if (role < 4) {
      lineRole = lineRole + "  (operation)";
    } else {
      lineRole = lineRole + "  (food)";
    }
  }

  // ---- serial ----
  Serial.print("Tap #");
  Serial.print(tapCounter);
  Serial.print("   UID: ");
  Serial.print(uidText);
  Serial.print("   -> ");
  Serial.print(lineTable);
  Serial.print("  /  ");
  Serial.println(lineRole);

  // ---- lcd ----
  lcd.setCursor(0, 0);
  lcd.print(pad20(uidText));
  lcd.setCursor(0, 1);
  lcd.print(pad20(lineTable));
  lcd.setCursor(0, 2);
  if (role < 0) {
    lcd.print(pad20("UNKNOWN"));
  } else {
    lcd.print(pad20(roleNames[role]));
  }
  lcd.setCursor(0, 3);
  lcd.print(pad20("Tap #" + String(tapCounter)));
}

String readUidText() {
  String uidText = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) {
      uidText = uidText + "0";
    }
    uidText = uidText + String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) {
      uidText = uidText + " ";
    }
  }
  uidText.toUpperCase();
  return uidText;
}

String pad20(String text) {
  while (text.length() < 20) {
    text = text + " ";
  }
  return text.substring(0, 20);
}
