import tkinter as tk
from tkinter import ttk
import paho.mqtt.client as mqtt
import threading
import struct
import time

# ================= CONFIGURATION =================
BROKER = "broker.hivemq.com"
TOPIC = "tuiracing"
PORT = 1883

# CAN IDs Mapping
CAN_ID_FRONT_ANALOG = 0x500
CAN_ID_ACCEL        = 0x501
CAN_ID_GYRO         = 0x502
CAN_ID_RPM          = 0x600
CAN_ID_VOLTAGE      = 0x601
CAN_ID_WATER_TEMP   = 0x602
CAN_ID_GEAR         = 0x700
CAN_ID_REAR_ANALOG  = 0x701
CAN_ID_GPS_POS      = 0x800
CAN_ID_GPS_SPD      = 0x801
CAN_ID_LAPTIME      = 0x900
CAN_ID_SYSTEM_HEALTH = 0xA00

HEALTH_NODE_FRONT = 1
HEALTH_NODE_REAR = 2

class TelemetryApp:
    def __init__(self, root):
        self.root = root
        self.root.title("🏁 Telemetry 2026 - Live Dashboard")
        self.root.geometry("900x700")
        self.root.configure(bg="#1e1e1e")

        # Custom Styling
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure("TFrame", background="#1e1e1e")
        self.style.configure("TLabelframe", background="#1e1e1e", foreground="white")
        self.style.configure("TLabelframe.Label", background="#1e1e1e", foreground="#00ff00", font=("Arial", 12, "bold"))
        self.style.configure("TLabel", background="#1e1e1e", foreground="white", font=("Consolas", 11))
        self.style.configure("Value.TLabel", background="#1e1e1e", foreground="#00ff00", font=("Consolas", 12, "bold"))

        # Data Storage
        self.data = {
            "RPM": 0, "GEAR": "N", "TEMP": 0.0, "BATT": 0.0,
            "SPD": 0.0, "LAT": 0.0, "LON": 0.0,
            "D1": 0, "D2": 0, "STR": 0,
            "RL": 0, "RR": 0, "BRK": 0,
            "AX": 0.0, "AY": 0.0, "AZ": 0.0,
            "LAST_MSG": "None", "MSG_COUNT": 0
        }
        self.health = {
            "FRONT": {
                "drops": 0, "failures": 0, "queue_free": 0,
                "flags": 0, "heartbeat": 0, "last_seen": 0
            },
            "REAR": {
                "drops": 0, "failures": 0, "queue_free": 0,
                "flags": 0, "heartbeat": 0, "last_seen": 0
            }
        }
        self.last_seen = {}

        self.setup_ui()
        
        # MQTT Setup
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.mqtt_thread = threading.Thread(target=self.run_mqtt, daemon=True)
        self.mqtt_thread.start()

        # Update Loop
        self.update_gui_loop()

    def setup_ui(self):
        # Header
        header = tk.Label(self.root, text="🚀 TELEMETRY 2026 - LIVE SENSORS", bg="#333333", fg="white", font=("Arial", 16, "bold"), pady=10)
        header.pack(fill=tk.X)

        main_container = ttk.Frame(self.root, padding=10)
        main_container.pack(fill=tk.BOTH, expand=True)

        # 1. ENGINE & CRITICAL (Top Row)
        engine_frame = ttk.LabelFrame(main_container, text=" ENGINE & DRIVE ", padding=10)
        engine_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)

        self.lbl_rpm = self.create_val_label(engine_frame, "RPM:", "0", row=0, col=0)
        self.lbl_gear = self.create_val_label(engine_frame, "GEAR:", "N", row=0, col=1)
        self.lbl_temp = self.create_val_label(engine_frame, "WATER TEMP:", "0°C", row=0, col=2)
        self.lbl_batt = self.create_val_label(engine_frame, "BATTERY:", "0.0V", row=0, col=3)

        # 2. FRONT SENSORS
        front_frame = ttk.LabelFrame(main_container, text=" FRONT SENSORS ", padding=10)
        front_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

        self.lbl_d1 = self.create_val_label(front_frame, "Damper FL:", "0", row=0, col=0)
        self.lbl_d2 = self.create_val_label(front_frame, "Damper FR:", "0", row=1, col=0)
        self.lbl_str = self.create_val_label(front_frame, "Steering:", "0", row=2, col=0)

        # 3. REAR SENSORS
        rear_frame = ttk.LabelFrame(main_container, text=" REAR SENSORS ", padding=10)
        rear_frame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

        self.lbl_rl = self.create_val_label(rear_frame, "Damper RL:", "0", row=0, col=0)
        self.lbl_rr = self.create_val_label(rear_frame, "Damper RR:", "0", row=1, col=0)
        self.lbl_brk = self.create_val_label(rear_frame, "Brake Press:", "0", row=2, col=0)

        # 4. GPS & MOTION
        gps_frame = ttk.LabelFrame(main_container, text=" GPS & MOTION ", padding=10)
        gps_frame.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

        self.lbl_spd = self.create_val_label(gps_frame, "Speed:", "0.0 km/h", row=0, col=0)
        self.lbl_coords = self.create_val_label(gps_frame, "Coords:", "0.0, 0.0", row=1, col=0)
        self.lbl_accel = self.create_val_label(gps_frame, "Accel (XYZ):", "0, 0, 0", row=2, col=0)

        # 5. SYSTEM HEALTH
        health_frame = ttk.LabelFrame(main_container, text=" SYSTEM HEALTH ", padding=10)
        health_frame.grid(row=2, column=1, sticky="nsew", padx=5, pady=5)

        self.lbl_front_health = self.create_val_label(health_frame, "Front:", "No data", row=0, col=0)
        self.lbl_rear_health = self.create_val_label(health_frame, "Rear:", "No data", row=1, col=0)

        # 6. STATUS BAR
        status_frame = tk.Frame(self.root, bg="#333333", pady=5)
        status_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        self.status_var = tk.StringVar(value="Connecting to MQTT...")
        lbl_status = tk.Label(status_frame, textvariable=self.status_var, bg="#333333", fg="#aaaaaa", font=("Arial", 9))
        lbl_status.pack(side=tk.LEFT, padx=10)

        main_container.columnconfigure(0, weight=1)
        main_container.columnconfigure(1, weight=1)

    def create_val_label(self, parent, text, val_text, row, col):
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=col, sticky="w", padx=15, pady=2)
        
        lbl = ttk.Label(frame, text=text)
        lbl.pack(side=tk.LEFT)
        
        val_lbl = ttk.Label(frame, text=val_text, style="Value.TLabel")
        val_lbl.pack(side=tk.LEFT, padx=5)
        return val_lbl

    def run_mqtt(self):
        try:
            self.client.connect(BROKER, PORT, 60)
            self.client.loop_forever()
        except Exception as e:
            self.status_var.set(f"MQTT Error: {e}")

    def on_connect(self, client, userdata, flags, rc, properties):
        self.status_var.set(f"Connected to {BROKER}")
        client.subscribe(TOPIC)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode('utf-8')
            parts = payload.split(',')
            if len(parts) < 3: return

            can_id = int(parts[1], 16)
            hex_data = parts[2]
            raw_data = bytes.fromhex(hex_data)
            
            self.last_seen[can_id] = time.time()
            self.data["MSG_COUNT"] += 1

            # Parsing Logic
            if can_id == CAN_ID_RPM and len(raw_data) >= 2:
                self.data["RPM"] = (raw_data[0] << 8) | raw_data[1]
            elif can_id == CAN_ID_GEAR and len(raw_data) >= 1:
                g = raw_data[0]
                self.data["GEAR"] = "N" if g == 0 else str(g)
            elif can_id == CAN_ID_VOLTAGE and len(raw_data) >= 1:
                self.data["BATT"] = raw_data[0] / 10.0
            elif can_id == CAN_ID_WATER_TEMP and len(raw_data) >= 1:
                self.data["TEMP"] = raw_data[0]
            elif can_id == CAN_ID_FRONT_ANALOG and len(raw_data) >= 6:
                self.data["D1"] = (raw_data[0] << 8) | raw_data[1]
                self.data["D2"] = (raw_data[2] << 8) | raw_data[3]
                self.data["STR"] = (raw_data[4] << 8) | raw_data[5]
            elif can_id == CAN_ID_REAR_ANALOG and len(raw_data) >= 6:
                self.data["RL"] = (raw_data[0] << 8) | raw_data[1]
                self.data["RR"] = (raw_data[2] << 8) | raw_data[3]
                self.data["BRK"] = (raw_data[4] << 8) | raw_data[5]
            elif can_id == CAN_ID_ACCEL and len(raw_data) >= 6:
                # Use struct to parse signed 16-bit integers (Big Endian)
                ax, ay, az = struct.unpack(">hhh", raw_data[:6])
                self.data["AX"], self.data["AY"], self.data["AZ"] = ax/100.0, ay/100.0, az/100.0
            elif can_id == CAN_ID_GPS_POS and len(raw_data) >= 8:
                # Floats are Little Endian from ESP32 memcpy
                lat, lon = struct.unpack("<ff", raw_data[:8])
                self.data["LAT"], self.data["LON"] = lat, lon
            elif can_id == CAN_ID_GPS_SPD and len(raw_data) >= 4:
                spd = struct.unpack("<f", raw_data[:4])[0]
                self.data["SPD"] = spd
            elif can_id == CAN_ID_SYSTEM_HEALTH and len(raw_data) >= 8:
                node = raw_data[0]
                module = "FRONT" if node == HEALTH_NODE_FRONT else "REAR" if node == HEALTH_NODE_REAR else None

                if module:
                    self.health[module]["flags"] = raw_data[1]
                    self.health[module]["drops"] = (raw_data[2] << 8) | raw_data[3]
                    self.health[module]["failures"] = (raw_data[4] << 8) | raw_data[5]
                    self.health[module]["queue_free"] = raw_data[6]
                    self.health[module]["heartbeat"] = raw_data[7]
                    self.health[module]["last_seen"] = time.time()

        except Exception as e:
            pass

    def update_gui_loop(self):
        # Update Labels from Storage
        self.lbl_rpm.config(text=f"{self.data['RPM']}")
        self.lbl_gear.config(text=f"{self.data['GEAR']}")
        self.lbl_temp.config(text=f"{self.data['TEMP']}°C")
        self.lbl_batt.config(text=f"{self.data['BATT']:.1f}V")
        
        self.lbl_d1.config(text=f"{self.data['D1']}")
        self.lbl_d2.config(text=f"{self.data['D2']}")
        self.lbl_str.config(text=f"{self.data['STR']}")
        
        self.lbl_rl.config(text=f"{self.data['RL']}")
        self.lbl_rr.config(text=f"{self.data['RR']}")
        self.lbl_brk.config(text=f"{self.data['BRK']}")
        
        self.lbl_spd.config(text=f"{self.data['SPD']:.1f} km/h")
        self.lbl_coords.config(text=f"{self.data['LAT']:.6f}, {self.data['LON']:.6f}")
        self.lbl_accel.config(text=f"{self.data['AX']:.2f}, {self.data['AY']:.2f}, {self.data['AZ']:.2f}")

        self.update_health_labels()

        # Update status bar with packet count
        self.status_var.set(f"Connected | Packets: {self.data['MSG_COUNT']} | Broker: {BROKER}")

        # Highlight if data is stale
        now = time.time()
        for cid, lbl in [(CAN_ID_RPM, self.lbl_rpm), (CAN_ID_GPS_SPD, self.lbl_spd)]:
            if cid in self.last_seen and now - self.last_seen[cid] > 2.0:
                lbl.config(foreground="orange")
            else:
                lbl.config(foreground="#00ff00")

        self.root.after(100, self.update_gui_loop)

    def update_health_labels(self):
        now = time.time()

        front = self.health["FRONT"]
        rear = self.health["REAR"]

        self.lbl_front_health.config(
            text=self.format_health("FRONT", front, now),
            foreground=self.health_color(front, now)
        )
        self.lbl_rear_health.config(
            text=self.format_health("REAR", rear, now),
            foreground=self.health_color(rear, now)
        )

    def format_health(self, module, health, now):
        if health["last_seen"] == 0:
            return "No data"

        age = now - health["last_seen"]
        stale = " STALE" if age > 7.0 else ""

        if module == "FRONT":
            return (
                f"Drops {health['drops']} | CAN fail {health['failures']} | "
                f"Q free {health['queue_free']} | HB {health['heartbeat']}{stale}"
            )

        flags = health["flags"]
        can_state = "CAN FAIL" if flags & 0x02 else "CAN OK"
        mqtt_state = "MQTT OK" if flags & 0x08 else "MQTT NO"
        ota_state = "OTA OK" if flags & 0x20 else "OTA NO"

        return (
            f"Drops {health['drops']} | Pub fail {health['failures']} | "
            f"Q free {health['queue_free']} | {can_state} | {mqtt_state} | {ota_state} | "
            f"HB {health['heartbeat']}{stale}"
        )

    def health_color(self, health, now):
        if health["last_seen"] == 0:
            return "#aaaaaa"
        if now - health["last_seen"] > 7.0:
            return "orange"
        if health["drops"] > 0 or health["failures"] > 0:
            return "#ff5555"
        return "#00ff00"

if __name__ == "__main__":
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
