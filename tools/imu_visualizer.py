"""Live 3-D visualizer for the CSV stream produced by IMU_DEMO.c.

The serial worker only publishes the newest complete DATA row.  Ursina's
render thread reads that snapshot, so a slow frame never builds a backlog of
old IMU readings.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time as wall_time

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # Give a useful message when run before installation.
    print("Missing dependency: pySerial. Install with: pip install pyserial ursina")
    raise SystemExit(1)


BAUD_RATE = 115_200
DATA_FIELDS = 14
STALE_AFTER_S = 1.0
TERMINAL_PERIOD_S = 0.25
RECONNECT_DELAY_S = 2.0


@dataclass(frozen=True)
class ImuSample:
    """One parsed DATA row, with units matching the firmware header."""

    time_ms: int
    accel: tuple[float, float, float]
    gyro_raw: tuple[float, float, float]
    gyro_cal: tuple[float, float, float]
    roll: float
    pitch: float
    yaw_relative: float


class SharedState:
    """Small thread-safe mailbox shared by serial and rendering threads."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sample: ImuSample | None = None
        self._sample_received_at = 0.0
        self._connection = "Starting"
        self._connection_ok = False
        self._firmware_message = "Waiting for STM32 status..."
        self._calibration = "Waiting for calibration"
        self._valid_rows = 0
        self._bad_rows = 0

    def set_connection(self, message: str, ok: bool) -> None:
        with self._lock:
            changed = message != self._connection
            self._connection = message
            self._connection_ok = ok
            if not ok:
                self._calibration = "Waiting for calibration"
                self._sample = None
                self._sample_received_at = 0.0
        if changed:
            print(f"SERIAL STATUS | {message}", flush=True)

    def set_firmware_message(self, message: str) -> None:
        with self._lock:
            self._firmware_message = message
            if message.startswith(("Hold the IMU still", "Calibration:")):
                self._calibration = "Calibrating - hold still"
                self._sample = None
                self._sample_received_at = 0.0
            elif message.startswith("Motion detected"):
                self._calibration = "Motion detected - retrying"
            elif message.startswith("Calibration read failed"):
                self._calibration = "Sensor read failed - retrying"
            elif message.startswith("Calibration accepted"):
                self._calibration = "Calibration complete"
            elif message.startswith("ERROR:"):
                self._calibration = "Sensor error"
        print(f"STM32 STATUS  | {message}", flush=True)

    def publish(self, sample: ImuSample) -> None:
        with self._lock:
            self._sample = sample
            self._sample_received_at = wall_time.monotonic()
            self._valid_rows += 1
            # DATA is emitted only after successful startup calibration.
            self._calibration = "Calibration complete"

    def record_bad_row(self) -> None:
        with self._lock:
            self._bad_rows += 1

    def snapshot(self) -> tuple[ImuSample | None, float, str, bool, str]:
        with self._lock:
            return (
                self._sample,
                self._sample_received_at,
                self._connection,
                self._connection_ok,
                self._calibration,
            )


def parse_data_line(line: str) -> ImuSample:
    """Parse the exact DATA format emitted by the current firmware."""
    fields = [field.strip() for field in line.split(",")]
    if len(fields) != DATA_FIELDS or fields[0] != "DATA":
        raise ValueError(f"expected {DATA_FIELDS} DATA fields")

    time_ms = int(fields[1])
    values = tuple(float(field) for field in fields[2:])
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite number")

    return ImuSample(
        time_ms=time_ms,
        accel=values[0:3],
        gyro_raw=values[3:6],
        gyro_cal=values[6:9],
        roll=values[9],
        pitch=values[10],
        yaw_relative=values[11],
    )


class SerialReader(threading.Thread):
    """Reconnect-capable line reader; it never touches Ursina objects."""

    def __init__(self, port: str, baud: int, state: SharedState) -> None:
        super().__init__(name="imu-serial-reader", daemon=True)
        self.port = port
        self.baud = baud
        self.state = state
        self.stop_event = threading.Event()

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:
        while not self.stop_event.is_set():
            self.state.set_connection(f"Connecting to {self.port} at {self.baud} baud...", False)
            try:
                with serial.Serial(self.port, self.baud, timeout=0.25) as connection:
                    # Drop bytes buffered before connection (possibly old firmware).
                    connection.reset_input_buffer()
                    self.state.set_connection(f"Connected to {self.port}", True)
                    self._read_lines(connection)
            except (serial.SerialException, OSError) as exc:
                self.state.set_connection(f"Disconnected from {self.port}: {exc}", False)

            self.stop_event.wait(RECONNECT_DELAY_S)

    def _read_lines(self, connection: serial.Serial) -> None:
        while not self.stop_event.is_set():
            raw_line = connection.readline()
            if not raw_line:  # Normal serial timeout; allows a quick shutdown.
                continue
            try:
                line = raw_line.decode("ascii").strip()
            except UnicodeDecodeError:
                self.state.record_bad_row()
                continue

            if not line:
                continue
            if line.startswith("#"):
                self.state.set_firmware_message(line[1:].strip())
                continue
            if not line.startswith("DATA,"):
                self.state.record_bad_row()
                continue

            try:
                self.state.publish(parse_data_line(line))
            except (ValueError, IndexError):
                # A partial/corrupt row is discarded; the previous good row remains.
                self.state.record_bad_row()


def available_ports() -> list[tuple[str, str]]:
    return [(item.device, item.description) for item in list_ports.comports()]


def select_port(requested: str | None) -> str:
    """Use --port when supplied; otherwise offer detected ports interactively."""
    if requested:
        return requested

    ports = available_ports()
    if ports:
        print("Available serial ports:")
        for number, (device, description) in enumerate(ports, start=1):
            print(f"  {number}: {device}  ({description})")
        prompt = "Select a number or type a COM port: "
    else:
        print("No serial ports were detected automatically.")
        prompt = "Type the STM32 COM port (for example COM3): "

    while True:
        try:
            answer = input(prompt).strip()
        except EOFError:
            raise SystemExit("No interactive input available; run with --port COM3")
        if answer.isdigit() and 1 <= int(answer) <= len(ports):
            return ports[int(answer) - 1][0]
        if answer:
            return answer


def format_xyz(values: tuple[float, float, float]) -> str:
    return f"X={values[0]:7.2f}  Y={values[1]:7.2f}  Z={values[2]:7.2f}"


def print_live_sample(sample: ImuSample) -> None:
    """Clearly labeled terminal output, limited to four updates per second."""
    print(
        f"LIVE t={sample.time_ms:8d} ms | "
        f"ACCEL m/s^2 [{format_xyz(sample.accel)}] | "
        f"GYRO RAW deg/s [{format_xyz(sample.gyro_raw)}] | "
        f"GYRO CAL deg/s [{format_xyz(sample.gyro_cal)}] | "
        f"FUSED roll={sample.roll:7.2f} pitch={sample.pitch:7.2f} deg | "
        f"RELATIVE YAW (DRIFTING)={sample.yaw_relative:7.2f} deg",
        flush=True,
    )


def smooth_angle(current: float, target: float, blend: float) -> float:
    """Interpolate through the shortest path across the +/-180 boundary."""
    difference = (target - current + 180.0) % 360.0 - 180.0
    return current + difference * blend


def run_visualizer(port: str, state: SharedState, reader: SerialReader) -> None:
    try:
        from ursina import (
            AmbientLight,
            DirectionalLight,
            Entity,
            Text,
            Ursina,
            Vec3,
            camera,
            color,
            time as ursina_time,
            window,
        )
    except ImportError:
        reader.stop()
        raise SystemExit("Missing dependency: Ursina. Install with: pip install pyserial ursina")

    ursina_icon = Path(sys.modules["ursina"].__file__).parent / "textures" / "ursina.ico"
    app = Ursina(
        title=f"STM32 MPU-6050 Quadcopter Visualizer - {port}",
        # Panda3D expects forward slashes even for an absolute Windows path.
        icon=ursina_icon.as_posix(),
        borderless=False,
        fullscreen=False,
        size=(1280, 720),
        development_mode=False,
        editor_ui_enabled=True,
    )
    window.color = color.rgb(18, 22, 30)
    window.fps_counter.enabled = True

    # The drone is intentionally built only from Ursina's basic primitives.
    drone = Entity(position=(1.5, 0.2, 2.5))
    Entity(parent=drone, model="cube", color=color.azure, scale=(1.7, 0.28, 0.9))
    Entity(parent=drone, model="cube", color=color.dark_gray, scale=(4.2, 0.10, 0.16), rotation_y=35)
    Entity(parent=drone, model="cube", color=color.dark_gray, scale=(4.2, 0.10, 0.16), rotation_y=-35)
    # Bright nose marker makes relative yaw visible from a distance.
    Entity(parent=drone, model="cube", color=color.orange, scale=(0.30, 0.22, 0.70), z=0.70)

    rotor_positions = ((-1.7, 0, -1.2), (1.7, 0, -1.2), (-1.7, 0, 1.2), (1.7, 0, 1.2))
    rotor_colors = (color.red, color.red, color.lime, color.lime)
    for position, rotor_color in zip(rotor_positions, rotor_colors):
        rotor = Entity(parent=drone, position=position)
        Entity(parent=rotor, model="cube", color=rotor_color, scale=(0.95, 0.05, 0.12))
        Entity(parent=rotor, model="cube", color=rotor_color, scale=(0.12, 0.05, 0.95))

    Entity(model="plane", texture="white_cube", texture_scale=(12, 12),
           color=color.rgb(40, 45, 53), scale=24, y=-2.0, z=3)
    AmbientLight(color=color.rgba(120, 120, 120, 0.35))
    sun = DirectionalLight(color=color.rgba(255, 245, 225, 0.9))
    sun.look_at(Vec3(1, -1, 1))
    camera.position = (0, 3.4, -10)
    camera.look_at(Vec3(0.25, 0.2, 2.5))

    # One fixed panel keeps all demo values together and away from the drone.
    Entity(parent=camera.ui, model="quad", position=(-0.54, 0, 1),
           scale=(0.70, 0.88), color=color.rgba(12, 22, 35, 235))
    status_text = Text(parent=camera.ui, x=-0.83, y=0.39, origin=(-0.5, 0.5),
                       scale=0.85, color=color.lime)
    angles_text = Text(parent=camera.ui, x=-0.83, y=0.13, origin=(-0.5, 0.5),
                       scale=0.90, color=color.azure)

    class VisualizerController(Entity):
        def __init__(self) -> None:
            super().__init__()
            self.display_roll = 0.0
            self.display_pitch = 0.0
            self.display_yaw = 0.0
            self.have_sample = False
            self.next_terminal_print = 0.0
            self.started_at = wall_time.monotonic()
            self.next_no_data_notice = self.started_at + 3.0

        def update(self) -> None:
            (
                sample,
                received_at,
                connection,
                connection_ok,
                calibration,
            ) = state.snapshot()
            now = wall_time.monotonic()
            age = now - received_at if received_at else math.inf

            if not connection_ok:
                shown_status = f"{port}: disconnected/connecting"
                status_color = color.red
            elif sample is None:
                calibrating = calibration.startswith(("Calibrating", "Motion detected",
                                                       "Sensor read failed"))
                if now - self.started_at < 6.0 or calibrating:
                    shown_status = f"{port}: waiting for data"
                else:
                    shown_status = f"ALERT: {port} has no data"
                status_color = color.orange
            elif age > STALE_AFTER_S:
                shown_status = f"ALERT: {port} data stopped"
                status_color = color.red
            else:
                shown_status = f"{port} live ({age * 1000:.0f} ms old)"
                status_color = color.lime

            status_text.text = (
                "CONNECTION\n"
                f"{shown_status}\n\n"
                "CALIBRATION\n"
                f"{calibration}"
            )
            status_text.color = status_color

            if sample is None:
                angles_text.text = "ANGLES (deg)\nWaiting for STM32 data"
                if connection_ok and now >= self.next_no_data_notice:
                    print(
                        "NO DATA | COM port is open, but the STM32 sent no bytes. "
                        "Flash IMU_DEMO, press RESET, and keep the IMU still.",
                        flush=True,
                    )
                    self.next_no_data_notice = now + 5.0
                return

            if not self.have_sample:
                self.display_roll = sample.roll
                self.display_pitch = sample.pitch
                self.display_yaw = sample.yaw_relative
                self.have_sample = True

            blend = 1.0 - math.exp(-8.0 * max(ursina_time.dt, 0.0))
            self.display_roll = smooth_angle(self.display_roll, sample.roll, blend)
            self.display_pitch = smooth_angle(self.display_pitch, sample.pitch, blend)
            self.display_yaw = smooth_angle(self.display_yaw, sample.yaw_relative, blend)

            # Swap and reverse the drone's roll/pitch display; yaw stays unchanged.
            drone.rotation_x = -self.display_roll
            drone.rotation_y = -self.display_yaw
            drone.rotation_z = self.display_pitch

            angles_text.text = (
                "ANGLES (deg)\n"
                f"Roll X    {sample.roll:+8.2f}\n"
                f"Pitch Y   {sample.pitch:+8.2f}\n"
                f"Yaw Z     {sample.yaw_relative:+8.2f}\n"
                "Yaw is relative; drifts"
            )

            if now >= self.next_terminal_print:
                print_live_sample(sample)
                self.next_terminal_print = now + TERMINAL_PERIOD_S

    VisualizerController()
    try:
        app.run(info=False)
    finally:
        reader.stop()
        reader.join(timeout=1.0)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize STM32 MPU-6050 DATA rows in Ursina.")
    parser.add_argument("--port", help="Serial port, for example COM3. Omit for an interactive list.")
    parser.add_argument("--baud", type=int, default=BAUD_RATE, help="Serial baud rate (default: 115200).")
    parser.add_argument("--list-ports", action="store_true", help="List detected ports and exit.")
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    if args.list_ports:
        ports = available_ports()
        if not ports:
            print("No serial ports detected.")
        for device, description in ports:
            print(f"{device}: {description}")
        return

    port = select_port(args.port)
    state = SharedState()
    reader = SerialReader(port, args.baud, state)
    reader.start()
    run_visualizer(port, state, reader)


if __name__ == "__main__":
    main()
