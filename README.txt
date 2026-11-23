BEYOND THE MENU - ALL CODE
==========================

WHAT TO UPLOAD WHERE

1_table_terminal/HCI_2_TABLE.ino
   Upload to BOTH table ESP32s.
   Set TABLE_NUMBER to 1 for table 1 and 2 for table 2 (line 25).

2_delivery_car/car_timed_delivery_v2.ino
   Upload to the car ESP32. This is the final car code:
   timed trip, 3 s stop with blue LED blinking, shorter return trip.
   Tune at the top: TABLE1_FORWARD_MS, TABLE2_FORWARD_MS,
   TURN_TIME_MS, RETURN_PERCENT. Use the values you tested on your car.

3_kitchen_laptop/
   mosquitto.conf       - broker settings (port 1883, anonymous)
   step3_kitchen_app.py - kitchen dashboard
   Run:  pip install paho-mqtt   then   python step3_kitchen_app.py

4_tests/  (not needed for the demo)
   card_identifier_test.ino - shows which table and role a card has
   hardware_check.ino       - motor direction and IR sensor test
   sensor_test_digital.ino  - digital IR sensor test

5_docs/hardware_wiring_reference.txt - all wiring and pin details

old_versions/ - earlier attempts (line following etc). Not used.

BEFORE THE DEMO
 - Laptop IP must match MQTT_HOST in the ESP32 sketches (192.168.0.200).
 - WiFi name and password are in every sketch. Change them if needed.
 - Start Mosquitto first, then the dashboard, then power the boards.
