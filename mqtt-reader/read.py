import paho.mqtt.client as mqtt
import json
import time

# Settings
BROKER = "broker.hivemq.com"
TOPIC = "tuiracing"


def on_connect(client, userdata, flags, rc, properties):
    print(f"Connected to Broker! (Result: {rc})")
    client.subscribe(TOPIC)


def on_message(client, userdata, msg):
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)

        # Get data with defaults
        lat = float(data.get('lat', 0))
        lon = float(data.get('lon', 0))
        speed_knots = float(data.get('spd', 0))

        # Convert Knots to Km/h (Standard conversion)
        speed_kmh = speed_knots * 1.852

        print(f"[{time.strftime('%H:%M:%S')}] 🚀 NEW PACKET")

        # Check if we have valid coordinates (A7670 returns 0.0 if no fix)
        if lat != 0.0 and lon != 0.0:
            print(f"   📍 Position: {lat:.6f}, {lon:.6f}")
            print(f"   🏎️  Speed:    {speed_kmh:.1f} km/h")
            print(f"   🌍 Maps:     http://maps.google.com/?q={lat},{lon}")
        else:
            print(f"   🛰️  GPS: No lock yet (Lat/Lon is 0)")

        print("-" * 30)

    except Exception as e:
        print(f"Error parsing: {msg.payload} | {e}")


# Setup Client
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message

print(f"Connecting to {BROKER}...")
client.connect(BROKER, 1883, 60)

try:
    client.loop_forever()
except KeyboardInterrupt:
    print("\nDisconnecting...")
    client.disconnect()