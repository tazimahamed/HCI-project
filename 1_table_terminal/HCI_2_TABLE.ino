/*
  STEP 2 - TABLE ESP32 FIRMWARE
  Hardware: ESP32 + RC522 RFID + 20x4 I2C LCD   (no buzzer)

  Wiring:
    RC522   SS -> 5 , SCK -> 18 , MOSI -> 23 , MISO -> 19 , RST -> 27 , 3.3V -> 3V3
    LCD     SDA -> 21 , SCL -> 22 , VCC -> VIN(5V)

  Libraries needed:
    "MFRC522" by GithubCommunity
    "LiquidCrystal I2C" by Frank de Brabander
    "PubSubClient" by Nick O'Leary

  FOR TABLE 2: change TABLE_NUMBER below from 1 to 2. Nothing else changes.
*/

#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ================= CHANGE THIS FOR EACH TABLE =================
#define TABLE_NUMBER 2
// ==============================================================

// ---------- network settings ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";
const char* MQTT_HOST = "192.168.0.200";   // kitchen laptop IP
const int   MQTT_PORT = 1883;

// ---------- pins ----------
#define SS_PIN  5
#define RST_PIN 27

// ---------- objects ----------
MFRC522 rfid(SS_PIN, RST_PIN);
LiquidCrystal_I2C lcd(0x27, 20, 4);   // change 0x27 to 0x3F if screen is blank
WiFiClient espClient;
PubSubClient mqtt(espClient);

// ---------- card roles ----------
#define ROLE_START     0
#define ROLE_CONFIRM   1
#define ROLE_REDUCE    2
#define ROLE_CALLCHEF  3
// roles 4..8 are the five food items

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

String foodNames[5] = { "BURGER", "PIZZA", "FRIES", "PASTA", "COFFEE" };

// ---------- order state ----------
int  counts[5]     = { 0, 0, 0, 0, 0 };
int  history[60];               // order in which items were added
int  historyTop    = 0;
bool orderStarted  = false;
bool orderConfirmed = false;
int  orderStatus   = 0;         // 1..5 once confirmed

// ---------- display state ----------
String lineBuf[6];
int    lineCount  = 0;
int    scrollTop  = 0;
String line4Text  = "TAP START CARD";
unsigned long lastScrollTime = 0;

// ---------- card read filtering ----------
String lastUID = "";
unsigned long lastReadTime = 0;

// ---------- mqtt topics ----------
String topicOrder;
String topicStatus;
String topicCall;
String topicPresence;
String topicReset;
unsigned long lastMqttTry = 0;

// ---------- kitchen reply watchdog ----------
bool waitingForKitchen = false;
bool kitchenIssue = false;
unsigned long confirmTime = 0;

// ==================================================================
//  SETUP
// ==================================================================
void setup() {
  Serial.begin(115200);
  delay(300);

  topicOrder  = "restaurant/table" + String(TABLE_NUMBER) + "/order";
  topicStatus = "restaurant/table" + String(TABLE_NUMBER) + "/status";
  topicCall   = "restaurant/table" + String(TABLE_NUMBER) + "/call";
  topicPresence = "restaurant/table" + String(TABLE_NUMBER) + "/presence";
  topicReset  = "restaurant/table" + String(TABLE_NUMBER) + "/reset";

  lcd.init();
  lcd.backlight();

  SPI.begin();
  rfid.PCD_Init();
  delay(100);

  showBootScreen();
  connectWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);

  resetOrder();
  orderStarted = false;
  line4Text = "TAP START CARD";
  drawScreen();

  Serial.print("Table ");
  Serial.print(TABLE_NUMBER);
  Serial.println(" ready.");
}

// ==================================================================
//  MAIN LOOP
// ==================================================================
void loop() {
  keepNetworkAlive();
  handleScrolling();
  handleKitchenWatchdog();
  handleCard();
}

// ==================================================================
//  NETWORK
// ==================================================================
void connectWiFi() {
  lcd.setCursor(0, 3);
  lcd.print(pad20("CONNECTING WIFI..."));

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

  String clientId = "table" + String(TABLE_NUMBER);

  // the last will tells the kitchen we died if power or wifi is lost
  if (mqtt.connect(clientId.c_str(), topicPresence.c_str(), 0, true, "offline")) {
    mqtt.publish(topicPresence.c_str(), "online", true);
    mqtt.subscribe(topicStatus.c_str());
    Serial.println("MQTT connected.");
  }
}

// status messages from the kitchen laptop arrive here
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  if (length < 1) {
    return;
  }

  int value = payload[0] - '0';
  if (value < 1 || value > 5) {
    return;
  }

  orderStatus = value;

  // the kitchen answered, so the watchdog can stand down
  waitingForKitchen = false;
  kitchenIssue = false;

  if (orderConfirmed) {
    line4Text = statusText(orderStatus);
    drawScreen();
  }

  // once delivered, free the table for a new order
  if (orderStatus == 5) {
    delay(2000);
    resetOrder();
    orderStarted = false;
    line4Text = "TAP START CARD";
    drawScreen();
  }
}

String statusText(int s) {
  if (s == 1) return "ORDER REQUESTED";
  if (s == 2) return "ORDER ACCEPTED";
  if (s == 3) return "ORDER PROCESSING";
  if (s == 4) return "ORDER FINISHED";
  if (s == 5) return "ORDER DELIVERED";
  return "";
}

// ==================================================================
//  CARD HANDLING
// ==================================================================
void handleCard() {
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

  // find which table and which role this card belongs to
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

  if (role < 0) {
    line4Text = "UNKNOWN CARD";
    drawScreen();
    return;
  }

  doAction(role);

  // a card from the other table still works, but gets flagged
  if (ownerTable != TABLE_NUMBER) {
    line4Text = "WRONG TABLE CARD(" + String(ownerTable) + ")";
  }

  drawScreen();
  Serial.print("UID ");
  Serial.print(uidText);
  Serial.print("  role ");
  Serial.println(role);
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

void doAction(int role) {

  // START always works and always wipes everything
  if (role == ROLE_START) {
    resetOrder();
    orderStarted = true;
    // tell the kitchen to clear this table from its screen
    String msg = "{\"table\":" + String(TABLE_NUMBER) + "}";
    mqtt.publish(topicReset.c_str(), msg.c_str());
    line4Text = "ORDER STARTED";
    return;
  }

  // nothing else works before START
  if (!orderStarted) {
    line4Text = "TAP START CARD";
    return;
  }

  // CALL CHEF works at any time
  if (role == ROLE_CALLCHEF) {
    String msg = "{\"table\":" + String(TABLE_NUMBER) + "}";
    mqtt.publish(topicCall.c_str(), msg.c_str());
    line4Text = "CALLING CHEF";
    return;
  }

  // after confirming, the order can no longer be edited
  if (orderConfirmed) {
    // if the kitchen never answered, CONFIRM sends the order again
    if (role == ROLE_CONFIRM && kitchenIssue) {
      sendOrder();
      kitchenIssue = false;
      waitingForKitchen = true;
      confirmTime = millis();
      line4Text = "RESENDING ORDER...";
      return;
    }
    line4Text = "ORDER CONFIRMED";
    return;
  }

  if (role == ROLE_CONFIRM) {
    if (historyTop == 0) {
      line4Text = "ORDER IS EMPTY";
      return;
    }
    sendOrder();
    orderConfirmed = true;
    orderStatus = 1;
    waitingForKitchen = true;
    kitchenIssue = false;
    confirmTime = millis();
    line4Text = statusText(1);
    return;
  }

  if (role == ROLE_REDUCE) {
    if (historyTop == 0) {
      line4Text = "NOTHING TO REDUCE";
      return;
    }
    historyTop = historyTop - 1;
    int lastItem = history[historyTop];
    counts[lastItem] = counts[lastItem] - 1;
    line4Text = foodNames[lastItem] + " -1";
    buildItemLines();
    return;
  }

  // roles 4..8 are food items
  int item = role - 4;
  if (item >= 0 && item < 5) {
    if (counts[item] >= 99) {
      line4Text = "MAX COUNT REACHED";
      return;
    }
    counts[item] = counts[item] + 1;
    if (historyTop < 60) {
      history[historyTop] = item;
      historyTop = historyTop + 1;
    }
    line4Text = foodNames[item] + " +1";
    buildItemLines();
  }
}

void resetOrder() {
  for (int i = 0; i < 5; i++) {
    counts[i] = 0;
  }
  historyTop = 0;
  orderConfirmed = false;
  orderStatus = 0;
  waitingForKitchen = false;
  kitchenIssue = false;
  buildItemLines();
}

void sendOrder() {
  String payload = "{\"table\":";
  payload = payload + String(TABLE_NUMBER);
  payload = payload + ",\"items\":[";

  bool first = true;
  for (int i = 0; i < 5; i++) {
    if (counts[i] > 0) {
      if (!first) {
        payload = payload + ",";
      }
      payload = payload + "{\"id\":" + String(i);
      payload = payload + ",\"name\":\"" + foodNames[i] + "\"";
      payload = payload + ",\"qty\":" + String(counts[i]) + "}";
      first = false;
    }
  }
  payload = payload + "]}";

  mqtt.publish(topicOrder.c_str(), payload.c_str());
  Serial.println(payload);
}

// ==================================================================
//  DISPLAY
// ==================================================================

// pack the items side by side, wrap to the next line when they do not fit
void buildItemLines() {
  for (int i = 0; i < 6; i++) {
    lineBuf[i] = "";
  }

  int row = 0;
  for (int i = 0; i < 5; i++) {
    if (counts[i] > 0) {
      String token = foodNames[i] + "(" + String(counts[i]) + ")";

      if (lineBuf[row].length() == 0) {
        lineBuf[row] = token;
      }
      else if (lineBuf[row].length() + 1 + token.length() <= 20) {
        lineBuf[row] = lineBuf[row] + " " + token;
      }
      else {
        row = row + 1;
        if (row > 5) {
          row = 5;
        }
        lineBuf[row] = token;
      }
    }
  }

  lineCount = 0;
  for (int i = 0; i < 6; i++) {
    if (lineBuf[i].length() > 0) {
      lineCount = i + 1;
    }
  }

  scrollTop = 0;
  lastScrollTime = millis();
}

// if the kitchen does not answer within 8 seconds, say so on line 4
void handleKitchenWatchdog() {
  if (!waitingForKitchen) {
    return;
  }
  if (millis() - confirmTime < 8000) {
    return;
  }
  waitingForKitchen = false;
  kitchenIssue = true;
  line4Text = "KITCHEN ISSUE";
  drawScreen();
  Serial.println("No reply from kitchen.");
}

// if the items need more than 3 lines, roll them every 2 seconds
void handleScrolling() {
  if (lineCount <= 3) {
    return;
  }
  if (millis() - lastScrollTime < 2000) {
    return;
  }
  lastScrollTime = millis();
  scrollTop = (scrollTop + 1) % lineCount;
  drawScreen();
}

void drawScreen() {
  if (!orderStarted) {
    lcd.setCursor(0, 0);
    lcd.print(pad20("RESTAURANT TABLE " + String(TABLE_NUMBER)));
    lcd.setCursor(0, 1);
    lcd.print(pad20(""));
    lcd.setCursor(0, 2);
    lcd.print(pad20(""));
    lcd.setCursor(0, 3);
    lcd.print(pad20(line4Text));
    return;
  }

  for (int r = 0; r < 3; r++) {
    String text = "";
    if (lineCount <= 3) {
      if (r < lineCount) {
        text = lineBuf[r];
      }
    } else {
      int idx = (scrollTop + r) % lineCount;
      text = lineBuf[idx];
    }
    lcd.setCursor(0, r);
    lcd.print(pad20(text));
  }

  lcd.setCursor(0, 3);
  lcd.print(pad20(line4Text));
}

void showBootScreen() {
  lcd.setCursor(0, 0);
  lcd.print(pad20("RESTAURANT TABLE " + String(TABLE_NUMBER)));
  lcd.setCursor(0, 1);
  lcd.print(pad20("STARTING UP..."));
  lcd.setCursor(0, 2);
  lcd.print(pad20(""));
  lcd.setCursor(0, 3);
  lcd.print(pad20(""));
}

String pad20(String text) {
  while (text.length() < 20) {
    text = text + " ";
  }
  return text.substring(0, 20);
}