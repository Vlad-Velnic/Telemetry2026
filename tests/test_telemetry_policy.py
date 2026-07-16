import re
import unittest
from collections import deque
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REAR_INCLUDES = ROOT / "Rear-Module" / "include" / "includes.h"
REAR_LOOP = ROOT / "Rear-Module" / "src" / "loop.cpp"
TINYGSM_A76XX = (
    ROOT / "Rear-Module" / "lib" / "TinyGSM" / "src" / "TinyGsmClientA76xx.h"
)


def define(name):
    match = re.search(
        rf"^(?:#define\s+|static constexpr\s+\w+\s+){re.escape(name)}\s*(?:=\s*)?([^\s;/]+)",
        REAR_INCLUDES.read_text(encoding="utf-8"),
        re.MULTILINE,
    )
    if not match:
        raise AssertionError(f"Missing firmware constant {name}")
    return int(match.group(1))


def record(timestamp, can_id, byte_count):
    return f"{timestamp},{can_id:X},{'AB' * byte_count}"


class LatestSlot:
    def __init__(self):
        self.value = None
        self.generation = 0
        self.dirty = False

    def update(self, value):
        self.value = value
        self.generation += 1
        self.dirty = True

    def commit(self, sent_generation):
        if self.generation == sent_generation:
            self.dirty = False


class PolicyModel:
    priorities = ("lap", "gear", "gps", "imu")

    def __init__(self):
        self.gps = deque(maxlen=2)
        self.gear = deque(maxlen=4)
        self.current = {}
        self.lap = None
        self.overflow = 0

    def push(self, queue, value):
        if len(queue) == queue.maxlen:
            self.overflow += 1
        queue.append(value)

    def disconnect(self):
        self.gps.clear()
        self.gear.clear()

    def snapshot(self):
        return dict(self.current), self.lap


class TelemetryPolicyTests(unittest.TestCase):
    def test_firmware_rate_constants(self):
        self.assertEqual(define("SENSOR_FREQ_HZ"), 25)
        self.assertEqual(define("MQTT_PUBLISH_PERIOD_MS"), 200)
        self.assertEqual(define("MQTT_PACKET_BUFFER_SIZE"), 512)
        self.assertEqual(define("LAP_MAX_SAMPLE_GAP_MS"), 800)

    def test_gps_rate_prefers_five_then_two_hz(self):
        setup = (ROOT / "Rear-Module" / "src" / "setup.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("const uint8_t candidates[] = {5, 2, 1};", setup)

    def test_a76xx_parser_stops_after_documented_vdop(self):
        source = TINYGSM_A76XX.read_text(encoding="utf-8")
        parser_tail = source[source.index("// VDOP is the final documented field"):
                             source.index("if (status) { *status = fixMode; }")]
        self.assertEqual(parser_tail.count("streamSkipUntil(','"), 0)
        self.assertEqual(parser_tail.count("streamSkipUntil('\\n'"), 1)
        self.assertIn("getGPSWithMilliseconds", source)

    def test_analog_can_ids_are_not_part_of_mqtt_policy(self):
        accepted = {0x501, 0x502, 0x700, 0x750, 0x751, 0x777}
        self.assertNotIn(0x500, accepted)
        self.assertNotIn(0x701, accepted)

    def test_latest_imu_is_published_at_5_hz(self):
        inputs = list(range(0, 1000, 40))
        outputs = [max(t for t in inputs if t <= boundary)
                   for boundary in range(0, 1000, 200)]
        self.assertEqual(outputs, [0, 200, 400, 600, 800])

    def test_five_atomic_gps_pairs_per_second(self):
        epochs = [(t, (f"position-{t}", f"speed-{t}"))
                  for t in range(0, 1000, 200)]
        self.assertEqual(len(epochs), 5)
        self.assertTrue(all(position.rsplit("-", 1)[1] == speed.rsplit("-", 1)[1]
                            for _, (position, speed) in epochs))

    def test_canonical_gps_point_is_decimal_for_can_mqtt_and_lap(self):
        source = REAR_LOOP.read_text(encoding="utf-8")
        self.assertIn("point.lat = rawLatitude;", source)
        self.assertIn("point.lon = rawLongitude;", source)
        self.assertNotIn("gnssDegreesMinutesToDecimal", source)
        self.assertIn("xQueueSend(lapGpsQueue, &point", source)
        self.assertIn("submitGpsTelemetry(point);", source)
        self.assertIn("point.epochKey != lastEpochKey", source)

    def test_lap_gate_is_initialized_in_decimal_degrees(self):
        text = REAR_INCLUDES.read_text(encoding="utf-8")
        values = {}
        for name in (
            "LAP_GATE_LEFT_LAT", "LAP_GATE_LEFT_LON",
            "LAP_GATE_RIGHT_LAT", "LAP_GATE_RIGHT_LON",
        ):
            match = re.search(rf"{name}\s*=\s*([-0-9.]+)", text)
            self.assertIsNotNone(match)
            values[name] = float(match.group(1))
        self.assertTrue(47.0 < values["LAP_GATE_LEFT_LAT"] < 48.0)
        self.assertTrue(27.0 < values["LAP_GATE_LEFT_LON"] < 28.0)
        self.assertNotEqual(
            (values["LAP_GATE_LEFT_LAT"], values["LAP_GATE_LEFT_LON"]),
            (values["LAP_GATE_RIGHT_LAT"], values["LAP_GATE_RIGHT_LON"]),
        )

    def test_gear_transitions_and_one_hz_refresh(self):
        samples = [(t, 0 if t < 240 else 1) for t in range(0, 1280, 40)]
        sent = []
        previous = None
        last_sent = -1000
        for timestamp, gear in samples:
            if gear != previous or timestamp - last_sent >= 1000:
                sent.append((timestamp, gear))
                last_sent = timestamp
            previous = gear
        self.assertEqual(sent, [(0, 0), (240, 1), (1240, 1)])

    def test_normal_batch_stays_below_pubsubclient_buffer(self):
        records = [
            record(123456789, 0x777, 4),
            record(123456789, 0x700, 1),
            record(123456789, 0x750, 8),
            record(123456789, 0x751, 4),
            record(123456789, 0x501, 6),
            record(123456789, 0x502, 6),
        ]
        payload = ";".join(records).encode("ascii")
        topic_and_mqtt_overhead = len("tuiracing") + 8
        self.assertLess(len(payload) + topic_and_mqtt_overhead, 512)

    def test_generation_safe_dirty_clearing(self):
        slot = LatestSlot()
        slot.update("old")
        sent_generation = slot.generation
        slot.update("new")
        slot.commit(sent_generation)
        self.assertTrue(slot.dirty)
        slot.commit(slot.generation)
        self.assertFalse(slot.dirty)

    def test_overflow_drops_oldest_and_counts_loss(self):
        policy = PolicyModel()
        for value in range(3):
            policy.push(policy.gps, value)
        self.assertEqual(list(policy.gps), [1, 2])
        self.assertEqual(policy.overflow, 1)

    def test_priority_order(self):
        self.assertEqual(
            PolicyModel.priorities,
            ("lap", "gear", "gps", "imu"),
        )

    def test_disconnect_discards_history_but_preserves_current_and_lap(self):
        policy = PolicyModel()
        policy.gps.extend(["old-gps-1", "old-gps-2"])
        policy.gear.extend([1, 2])
        policy.current = {"gps": "fresh-gps", "gear": 2}
        policy.lap = 81234
        policy.disconnect()
        self.assertFalse(policy.gps or policy.gear)
        self.assertEqual(policy.snapshot(), (policy.current, 81234))

    def test_ecu_and_unknown_can_ids_are_ignored(self):
        accepted = {0x501, 0x502}
        self.assertFalse({0x500, 0x600, 0x601, 0x602, 0x701, 0x123} & accepted)

    def test_health_frames_do_not_exist(self):
        sources = [
            ROOT / "Front-Module" / "include" / "canIDs.h",
            ROOT / "Rear-Module" / "include" / "canIDs.h",
            ROOT / "Front-Module" / "src" / "loop.cpp",
            ROOT / "Rear-Module" / "src" / "loop.cpp",
        ]
        for path in sources:
            source = path.read_text(encoding="utf-8")
            self.assertNotIn("CAN_ID_SYSTEM_HEALTH", source)
            self.assertNotIn("sendHealthFrame", source)


if __name__ == "__main__":
    unittest.main()
