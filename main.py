import paho.mqtt.client as mqtt
import time
import os
import sys
import threading

# ================= CONFIGURĂRI =================
BROKER = "broker.hivemq.com"
TOPIC = "tuiracing"
PORT = 1883

# Mapare ID-uri CAN conform fișierului 'canIDs.h'
CAN_MAP = {
    0x500: "FRONT_ANALOG",
    0x501: "ACCEL",
    0x502: "GYRO",
    0x600: "RPM",
    0x601: "VOLTAGE",
    0x602: "WATER_TEMP",
    0x700: "GEAR",
    0x701: "REAR_ANALOG",
    0x800: "GPS_POS",
    0x801: "GPS_SPD",
    0x900: "LAPTIME"
}

# Dicționar pentru stocarea stării curente a fiecărui ID
# Structura: { id: { 'last_ts': time.time(), 'data': "HEX", 'count': 0, 'freq': 0.0, 'dev_ts': 0 } }
dashboard_data = {}


def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Conectat la Broker! Cod: {rc}")
    client.subscribe(TOPIC)


def on_message(client, userdata, msg):
    global dashboard_data
    try:
        # Decodăm payload-ul din C++: "timestamp,id_hex,data_hex"
        # Exemplu: "12500,500,AABB1122"
        payload = msg.payload.decode('utf-8')
        parts = payload.split(',')

        if len(parts) < 3:
            return

        dev_ts = int(parts[0])  # Timestamp-ul de pe microcontroller (millis)
        can_id = int(parts[1], 16)  # ID-ul CAN în format Hex
        hex_data = parts[2]  # Datele efective

        now = time.time()

        # Calculăm frecvența
        freq = 0.0
        if can_id in dashboard_data:
            last_time = dashboard_data[can_id]['last_ts']
            delta = now - last_time
            if delta > 0:
                # Medie simplă instantanee
                freq = 1.0 / delta

            # Actualizăm contorul total
            count = dashboard_data[can_id]['count'] + 1
        else:
            count = 1

        # Salvăm în structura de date
        dashboard_data[can_id] = {
            'last_ts': now,
            'data': hex_data,
            'count': count,
            'freq': freq,
            'dev_ts': dev_ts
        }

    except Exception as e:
        # Ignorăm erorile de parsare pentru a nu bloca dashboard-ul
        pass


def print_dashboard():
    while True:
        # Curățăm ecranul (cls pentru Windows, clear pentru Linux/Mac)
        os.system('cls' if os.name == 'nt' else 'clear')

        print(f"📡 TELEMETRY DASHBOARD - {TOPIC} @ {BROKER}")
        print("=" * 85)
        print(
            f"{'ID (Hex)':<10} | {'NUME SENZOR':<20} | {'FREQ (Hz)':<10} | {'PACKETS':<8} | {'DEV_TS':<10} | {'DATA (Hex)'}")
        print("-" * 85)

        current_time = time.time()

        # Sortăm după ID pentru consistență vizuală
        sorted_ids = sorted(dashboard_data.keys())

        if not sorted_ids:
            print("⏳ Aștept pachete de date...")

        for can_id in sorted_ids:
            info = dashboard_data[can_id]

            # Resetăm frecvența la 0 dacă nu am primit date în ultimele 2 secunde
            if current_time - info['last_ts'] > 2.0:
                info['freq'] = 0.0

            name = CAN_MAP.get(can_id, "UNKNOWN_ID")

            print(
                f"0x{can_id:03X}      | {name:<20} | {info['freq']:6.1f} Hz | {info['count']:<8} | {info['dev_ts']:<10} | {info['data']}")

        print("=" * 85)
        print("Ctrl+C pentru a ieși.")
        time.sleep(0.2)  # Refresh rate al dashboard-ului (5Hz vizual)


# Setup MQTT Client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

print(f"Conectare la {BROKER}...")
try:
    client.connect(BROKER, PORT, 60)
    client.loop_start()  # Rulează MQTT într-un thread separat

    # Rulăm dashboard-ul în thread-ul principal
    print_dashboard()

except KeyboardInterrupt:
    print("\nDeconectare...")
    client.loop_stop()
    client.disconnect()
except Exception as e:
    print(f"Eroare: {e}")