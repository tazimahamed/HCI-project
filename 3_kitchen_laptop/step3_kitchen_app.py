"""
STEP 3 - KITCHEN LAPTOP APP
Restaurant food delivery robot system

What it does:
  - listens for orders from table 1 and table 2 ESP32s
  - lets the chef move each order through the 5 statuses
  - pushes status back to the table LCD instantly
  - shows chef calls
  - dispatches the car and marks the order delivered

Before running:
  1. Install the Mosquitto broker on this laptop and make sure it is running.
  2. pip install paho-mqtt
  3. Set this laptop to a fixed IP and put that same IP in the ESP32 sketches.
"""

import json
import queue
import tkinter as tk
from tkinter import font as tkfont

import paho.mqtt.client as mqtt

# ---------------- settings ----------------
BROKER_HOST = "localhost"
BROKER_PORT = 1883

STATUS_NAMES = {
    0: "NO ORDER",
    1: "ORDER REQUESTED",
    2: "ORDER ACCEPTED",
    3: "ORDER PROCESSING",
    4: "ORDER FINISHED",
    5: "ORDER DELIVERED",
}

STATUS_COLORS = {
    0: "#888888",
    1: "#d35400",
    2: "#2980b9",
    3: "#8e44ad",
    4: "#27ae60",
    5: "#16a085",
}

# ---------------- shared state ----------------
# orders[table] = {"items": [...], "status": int}
orders = {
    1: {"items": [], "status": 0},
    2: {"items": [], "status": 0},
}

car_state = {"state": "unknown", "weight": 0}

# which tables are currently connected to the broker
online = {1: False, 2: False}

# messages arrive on the MQTT thread, the UI reads them from here
inbox = queue.Queue()


# ==================================================================
#  MQTT SIDE
# ==================================================================
def on_connect(client, userdata, flags, rc, properties=None):
    print("Connected to broker, rc =", rc)
    client.subscribe("restaurant/table1/order")
    client.subscribe("restaurant/table2/order")
    client.subscribe("restaurant/table1/call")
    client.subscribe("restaurant/table2/call")
    client.subscribe("restaurant/table1/online")
    client.subscribe("restaurant/table2/online")
    client.subscribe("restaurant/table1/reset")
    client.subscribe("restaurant/table2/reset")
    client.subscribe("restaurant/table1/presence")
    client.subscribe("restaurant/table2/presence")
    client.subscribe("restaurant/car/state")
    client.subscribe("restaurant/car/weight")
    inbox.put(("log", "Broker connected."))


def on_message(client, userdata, msg):
    topic = msg.topic
    text = msg.payload.decode("utf-8", errors="ignore")

    if topic.endswith("/order"):
        try:
            data = json.loads(text)
            inbox.put(("order", data))
        except json.JSONDecodeError:
            inbox.put(("log", "Bad order payload: " + text))

    elif topic.endswith("/call"):
        try:
            data = json.loads(text)
            inbox.put(("call", data.get("table", 0)))
        except json.JSONDecodeError:
            inbox.put(("log", "Bad call payload: " + text))

    elif topic.endswith("/online"):
        try:
            table = int(topic.split("/")[1].replace("table", ""))
            inbox.put(("online", (table, text)))
        except ValueError:
            pass

    elif topic.endswith("/reset"):
        try:
            data = json.loads(text)
            inbox.put(("reset", data.get("table", 0)))
        except json.JSONDecodeError:
            inbox.put(("log", "Bad reset payload: " + text))

    elif topic.endswith("/presence"):
        # topic looks like restaurant/table1/presence
        try:
            table = int(topic.split("/")[1].replace("table", ""))
            inbox.put(("presence", (table, text)))
        except ValueError:
            pass

    elif topic == "restaurant/car/state":
        inbox.put(("car_state", text))

    elif topic == "restaurant/car/weight":
        inbox.put(("car_weight", text))


def make_client():
    # works with both paho-mqtt 1.x and 2.x
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="kitchen")
    except AttributeError:
        c = mqtt.Client(client_id="kitchen")
    c.on_connect = on_connect
    c.on_message = on_message
    return c


client = make_client()


def publish_status(table, status):
    """Send the status to the table LCD. Retained so a rebooting table re-syncs."""
    topic = "restaurant/table" + str(table) + "/status"
    client.publish(topic, str(status), retain=True)


def dispatch_car(table):
    payload = json.dumps({"table": table})
    client.publish("restaurant/car/command", payload)


# ==================================================================
#  UI SIDE
# ==================================================================
class TablePanel:
    def __init__(self, parent, table_number):
        self.table = table_number

        self.frame = tk.LabelFrame(
            parent,
            text="  TABLE " + str(table_number) + "  ",
            font=("Segoe UI", 14, "bold"),
            padx=12,
            pady=12,
            bg="#ffffff",
        )

        self.conn_label = tk.Label(
            self.frame,
            text="OFFLINE",
            font=("Segoe UI", 10, "bold"),
            fg="#ffffff",
            bg="#c0392b",
            pady=4,
        )
        self.conn_label.pack(fill="x", pady=(0, 8))

        self.items_box = tk.Text(
            self.frame,
            height=6,
            width=30,
            font=("Consolas", 13),
            bg="#f7f7f7",
            relief="flat",
        )
        self.items_box.pack(fill="x", expand=False, pady=(0, 10))
        self.items_box.configure(state="disabled")

        self.status_label = tk.Label(
            self.frame,
            text=STATUS_NAMES[0],
            font=("Segoe UI", 13, "bold"),
            fg="#ffffff",
            bg=STATUS_COLORS[0],
            pady=8,
        )
        self.status_label.pack(fill="x", pady=(0, 10))

        row1 = tk.Frame(self.frame, bg="#ffffff")
        row1.pack(fill="x", pady=2)
        row2 = tk.Frame(self.frame, bg="#ffffff")
        row2.pack(fill="x", pady=2)

        self.btn_accept = tk.Button(
            row1, text="Accept", width=11,
            command=lambda: self.set_status(2))
        self.btn_process = tk.Button(
            row1, text="Processing", width=11,
            command=lambda: self.set_status(3))
        self.btn_finish = tk.Button(
            row1, text="Finished", width=11,
            command=lambda: self.set_status(4))

        self.btn_accept.pack(side="left", padx=2)
        self.btn_process.pack(side="left", padx=2)
        self.btn_finish.pack(side="left", padx=2)

        self.btn_send = tk.Button(
            row2, text="Send Car", width=17,
            bg="#2c3e50", fg="white",
            command=self.send_car)
        self.btn_delivered = tk.Button(
            row2, text="Mark Delivered", width=17,
            command=lambda: self.set_status(5))

        self.btn_send.pack(side="left", padx=2)
        self.btn_delivered.pack(side="left", padx=2)

        self.btn_clear = tk.Button(
            self.frame, text="Clear Order",
            command=self.clear_order)
        self.btn_clear.pack(fill="x", pady=(8, 0))

    def refresh(self):
        data = orders[self.table]

        self.items_box.configure(state="normal")
        self.items_box.delete("1.0", "end")
        if len(data["items"]) == 0:
            self.items_box.insert("end", "  (no order)\n")
        else:
            for item in data["items"]:
                line = "  {:<10} x {}\n".format(item["name"], item["qty"])
                self.items_box.insert("end", line)
        self.items_box.configure(state="disabled")

        status = data["status"]
        self.status_label.configure(
            text=STATUS_NAMES[status],
            bg=STATUS_COLORS[status],
        )

    def set_presence(self, state):
        if state == "online":
            self.conn_label.configure(text="ONLINE", bg="#27ae60")
        else:
            self.conn_label.configure(text="OFFLINE", bg="#c0392b")

    def set_status(self, status):
        if orders[self.table]["status"] == 0:
            log("Table " + str(self.table) + ": no order to update.")
            return
        orders[self.table]["status"] = status
        publish_status(self.table, status)
        log("Table " + str(self.table) + " -> " + STATUS_NAMES[status])
        self.refresh()

    def send_car(self):
        if orders[self.table]["status"] != 4:
            log("Table " + str(self.table) + ": finish the order first.")
            return
        dispatch_car(self.table)
        log("Car dispatched to table " + str(self.table) + ".")

    def clear_order(self):
        orders[self.table] = {"items": [], "status": 0}
        self.refresh()
        log("Table " + str(self.table) + " cleared.")


def log(text):
    log_box.configure(state="normal")
    log_box.insert("end", text + "\n")
    log_box.see("end")
    log_box.configure(state="disabled")


def handle_inbox():
    """Runs on the UI thread, drains messages that arrived on the MQTT thread."""
    while not inbox.empty():
        kind, data = inbox.get()

        if kind == "order":
            table = data.get("table", 0)
            if table in orders:
                orders[table]["items"] = data.get("items", [])
                orders[table]["status"] = 1
                publish_status(table, 1)
                panels[table].refresh()
                log("NEW ORDER from table " + str(table))

        elif kind == "call":
            if data in orders:
                call_banner.configure(
                    text="TABLE " + str(data) + " IS CALLING THE CHEF",
                    bg="#c0392b",
                )
                log("Table " + str(data) + " called the chef.")

        elif kind == "reset":
            if data in orders:
                orders[data] = {"items": [], "status": 0}
                panels[data].refresh()
                # wipe the retained status so a rebooting table starts clean
                client.publish(
                    "restaurant/table" + str(data) + "/status", "", retain=True)
                log("Table " + str(data) + " started a new order, screen cleared.")

        elif kind == "presence":
            table, state = data
            if table in orders:
                panels[table].set_presence(state)
                log("Table " + str(table) + " is " + state + ".")

        elif kind == "car_state":
            car_state["state"] = data
            car_label.configure(
                text="CAR: " + data + "   |   WEIGHT: " + str(car_state["weight"]) + " g"
            )
            log("Car state: " + data)

        elif kind == "car_weight":
            car_state["weight"] = data
            car_label.configure(
                text="CAR: " + car_state["state"] + "   |   WEIGHT: " + str(data) + " g"
            )

        elif kind == "log":
            log(data)

    root.after(100, handle_inbox)


def clear_call_banner():
    call_banner.configure(text="", bg="#ecf0f1")


# ---------------- build the window ----------------
root = tk.Tk()
root.title("Kitchen Dashboard - Restaurant Delivery Robot")
root.configure(bg="#ecf0f1")
root.geometry("880x680")

title = tk.Label(
    root, text="KITCHEN DASHBOARD",
    font=("Segoe UI", 18, "bold"),
    bg="#ecf0f1", pady=10,
)
title.pack()

call_banner = tk.Label(
    root, text="", font=("Segoe UI", 13, "bold"),
    fg="white", bg="#ecf0f1", pady=8,
)
call_banner.pack(fill="x", padx=14)

tk.Button(root, text="Dismiss call", command=clear_call_banner).pack(pady=(0, 8))

# the bottom strips are packed first so they always keep their space
log_box = tk.Text(root, height=7, font=("Consolas", 10), bg="#2b2b2b", fg="#e0e0e0")
log_box.pack(side="bottom", fill="x", padx=14, pady=(0, 14))
log_box.configure(state="disabled")

car_label = tk.Label(
    root, text="CAR: unknown   |   WEIGHT: 0 g",
    font=("Segoe UI", 12, "bold"),
    bg="#34495e", fg="white", pady=8,
)
car_label.pack(side="bottom", fill="x", padx=14, pady=(10, 6))

tables_row = tk.Frame(root, bg="#ecf0f1")
tables_row.pack(fill="both", expand=True, padx=14)

panels = {}
panels[1] = TablePanel(tables_row, 1)
panels[2] = TablePanel(tables_row, 2)
panels[1].frame.pack(side="left", fill="both", expand=True, padx=6)
panels[2].frame.pack(side="left", fill="both", expand=True, padx=6)

panels[1].refresh()
panels[2].refresh()

# ---------------- start ----------------
try:
    client.connect(BROKER_HOST, BROKER_PORT, 60)
    client.loop_start()
except Exception as e:
    log("Could not reach the broker: " + str(e))
    log("Is Mosquitto running?")

root.after(100, handle_inbox)
root.mainloop()

client.loop_stop()
client.disconnect()
