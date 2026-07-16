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
CAN_ID_ACCEL        = 0x501
CAN_ID_GYRO         = 0x502
CAN_ID_GEAR         = 0x700
CAN_ID_GPS_POS      = 0x750
CAN_ID_GPS_SPD      = 0x751
CAN_ID_LAPTIME      = 0x777

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
            "GEAR": "N", "SPD": 0.0, "LAT": 0.0, "LON": 0.0,
            "AX": 0.0, "AY": 0.0, "AZ": 0.0,
            "GX": 0.0, "GY": 0.0, "GZ": 0.0,
            "LAP_MS": 0,
            "LAST_MSG": "None", "MSG_COUNT": 0
        }
        self.last_seen = {}
        self.data_lock = threading.Lock()
        self.mqtt_status = "Connecting to MQTT..."

        self.setup_ui()
        
        # MQTT Setup
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
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

        # 1. DRIVER (Top Row)
        driver_frame = ttk.LabelFrame(main_container, text=" DRIVER ", padding=10)
        driver_frame.grid(row=0, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)

        self.lbl_gear = self.create_val_label(driver_frame, "GEAR:", "N", row=0, col=0)
        self.lbl_spd = self.create_val_label(driver_frame, "GPS SPEED:", "0.0 km/h", row=0, col=1)
        self.lbl_lap = self.create_val_label(driver_frame, "LAST LAP:", "--:--.-", row=0, col=2)

        # 2. GPS & MOTION
        gps_frame = ttk.LabelFrame(main_container, text=" GPS & MOTION ", padding=10)
        gps_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", padx=5, pady=5)

        self.lbl_coords = self.create_val_label(gps_frame, "Coords:", "0.0, 0.0", row=0, col=0)
        self.lbl_accel = self.create_val_label(gps_frame, "Accel (XYZ):", "0, 0, 0", row=1, col=0)
        self.lbl_gyro = self.create_val_label(gps_frame, "Gyro (XYZ):", "0, 0, 0", row=2, col=0)

        # 3. STATUS BAR
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
        while True:
            try:
                self.mqtt_status = f"Connecting to {BROKER}..."
                self.client.connect(BROKER, PORT, 60)
                self.client.loop_forever()
            except Exception as e:
                self.mqtt_status = f"MQTT error: {e}; retrying"
                time.sleep(5)

    def on_connect(self, client, userdata, flags, rc, properties):
        if rc == 0:
            self.mqtt_status = f"Connected to {BROKER}"
            client.subscribe(TOPIC)
        else:
            self.mqtt_status = f"MQTT connection refused: {rc}"

    def on_disconnect(self, client, userdata, disconnect_flags, rc, properties):
        self.mqtt_status = f"Disconnected ({rc}); reconnecting"

    def on_message(self, client, userdata, msg):
        try:
            records = msg.payload.decode('utf-8').split(';')
        except UnicodeDecodeError as e:
            self.mqtt_status = f"Bad telemetry packet: {e}"
            return

        for payload in records:
            self.process_payload(payload)

    def process_payload(self, payload):
        try:
            parts = payload.split(',')
            if len(parts) < 3: return

            can_id = int(parts[1], 16)
            hex_data = parts[2]
            raw_data = bytes.fromhex(hex_data)
            
            with self.data_lock:
                self.last_seen[can_id] = time.time()
                self.data["MSG_COUNT"] += 1

                # Parsing Logic
                if can_id == CAN_ID_GEAR and len(raw_data) >= 1:
                    g = raw_data[0]
                    self.data["GEAR"] = "N" if g == 0 else str(g)
                elif can_id == CAN_ID_ACCEL and len(raw_data) >= 6:
                    ax, ay, az = struct.unpack(">hhh", raw_data[:6])
                    self.data["AX"], self.data["AY"], self.data["AZ"] = ax/100.0, ay/100.0, az/100.0
                elif can_id == CAN_ID_GYRO and len(raw_data) >= 6:
                    gx, gy, gz = struct.unpack(">hhh", raw_data[:6])
                    self.data["GX"], self.data["GY"], self.data["GZ"] = gx/100.0, gy/100.0, gz/100.0
                elif can_id == CAN_ID_GPS_POS and len(raw_data) >= 8:
                    lat, lon = struct.unpack("<ff", raw_data[:8])
                    self.data["LAT"], self.data["LON"] = lat, lon
                elif can_id == CAN_ID_GPS_SPD and len(raw_data) >= 4:
                    self.data["SPD"] = struct.unpack("<f", raw_data[:4])[0]
                elif can_id == CAN_ID_LAPTIME and len(raw_data) >= 4:
                    self.data["LAP_MS"] = struct.unpack(">I", raw_data[:4])[0]

        except Exception as e:
            self.mqtt_status = f"Bad telemetry packet: {e}"

    def update_gui_loop(self):
        with self.data_lock:
            data = self.data.copy()
            last_seen = self.last_seen.copy()

        # Update Labels from Storage
        self.lbl_gear.config(text=f"{data['GEAR']}")
        
        self.lbl_spd.config(text=f"{data['SPD']:.1f} km/h")
        self.lbl_coords.config(text=f"{data['LAT']:.6f}, {data['LON']:.6f}")
        self.lbl_accel.config(text=f"{data['AX']:.2f}, {data['AY']:.2f}, {data['AZ']:.2f}")
        self.lbl_gyro.config(text=f"{data['GX']:.2f}, {data['GY']:.2f}, {data['GZ']:.2f}")
        lap_ms = data["LAP_MS"]
        if lap_ms:
            self.lbl_lap.config(text=f"{lap_ms // 60000}:{(lap_ms // 1000) % 60:02d}.{(lap_ms // 100) % 10}")

        # Update status bar with packet count
        self.status_var.set(f"{self.mqtt_status} | Packets: {data['MSG_COUNT']}")

        # Highlight if data is stale
        now = time.time()
        stale_labels = [
            (CAN_ID_GEAR, self.lbl_gear),
            (CAN_ID_ACCEL, self.lbl_accel),
            (CAN_ID_GYRO, self.lbl_gyro),
            (CAN_ID_GPS_SPD, self.lbl_spd),
            (CAN_ID_GPS_POS, self.lbl_coords),
        ]
        for cid, lbl in stale_labels:
            timeout = 3.0 if cid in (CAN_ID_GPS_SPD, CAN_ID_GPS_POS) else 2.0
            if cid not in last_seen or now - last_seen[cid] > timeout:
                lbl.config(foreground="orange")
            else:
                lbl.config(foreground="#00ff00")

        self.lbl_lap.config(
            foreground="#00ff00" if CAN_ID_LAPTIME in last_seen else "orange"
        )

        self.root.after(100, self.update_gui_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = TelemetryApp(root)
    root.mainloop()
