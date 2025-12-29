import argparse
import json
import os
import queue
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
PROJECT_DIR = BASE_DIR / "GKFlasher-main"
sys.path.insert(0, str(PROJECT_DIR))

import yaml
from gkbus.hardware import TimeoutException
from gkbus.protocol import kwp2000
from gkbus.protocol.kwp2000.commands import StartDiagnosticSession
from gkbus.protocol.kwp2000.enums import DiagnosticSession

from ecu_definitions import BAUDRATES, AccessLevel
from flasher.ecu import DesiredBaudrate, calculate_key
from flasher.logging import data_sources, grab, convert
from gkflasher import initialize_bus, cli_identify_ecu


def load_config(config_path: str) -> dict:
    path = Path(config_path)
    if not path.is_absolute():
        path = PROJECT_DIR / path
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def build_config(args: argparse.Namespace) -> dict:
    config = load_config(args.config)
    if args.protocol:
        config["protocol"] = args.protocol
    if args.interface:
        config[config["protocol"]]["interface"] = args.interface
    if args.baudrate:
        config[config["protocol"]]["baudrate"] = args.baudrate
    return config


def parameter_labels() -> list[str]:
    labels = []
    for source in data_sources:
        for parameter in source["parameters"]:
            unit = parameter["unit"]
            if unit:
                labels.append(f"{parameter['name']} ({unit})")
            else:
                labels.append(parameter["name"])
    return labels


def poll_values(ecu) -> list[float]:
    values = []
    for source in data_sources:
        raw_data = ecu.bus.execute(source["payload"]).get_data()
        for parameter in source["parameters"]:
            value = grab(raw_data, parameter)
            values.append(convert(value, parameter))
    return values


def setup_ecu(config: dict, args: argparse.Namespace):
    print(f"[*] Selected protocol: {config['protocol']}. Initializing..")
    bus = initialize_bus(config["protocol"], config[config["protocol"]])

    bus.init(
        kwp2000.commands.StartCommunication(),
        keepalive_command=kwp2000.commands.TesterPresent(kwp2000.enums.ResponseType.REQUIRED),
        keepalive_delay=1.5,
    )
    bus.transport.set_buffer_size(20)

    if args.desired_baudrate is not None:
        try:
            desired_baudrate = DesiredBaudrate(
                index=args.desired_baudrate, baudrate=BAUDRATES[args.desired_baudrate]
            )
        except KeyError:
            raise ValueError("Selected baudrate is invalid.")

        print(f"[*] Trying to start diagnostic session with baudrate {desired_baudrate.baudrate}")
        try:
            bus.execute(
                kwp2000.commands.StartDiagnosticSession(
                    kwp2000.enums.DiagnosticSession.FLASH_REPROGRAMMING,
                    desired_baudrate.index,
                )
            )
            bus.transport.hardware.set_baudrate(desired_baudrate.baudrate)
        except TimeoutException:
            bus.transport.hardware.socket.reset_input_buffer()
            bus.transport.hardware.socket.reset_output_buffer()
            bus.transport.hardware.set_baudrate(desired_baudrate.baudrate)
            bus.execute(
                kwp2000.commands.StartDiagnosticSession(
                    kwp2000.enums.DiagnosticSession.FLASH_REPROGRAMMING,
                    desired_baudrate.index,
                )
            )
    else:
        desired_baudrate = DesiredBaudrate(index=None, baudrate=10400)
        print("[*] Trying to start diagnostic session")
        bus.execute(kwp2000.commands.StartDiagnosticSession(kwp2000.enums.DiagnosticSession.FLASH_REPROGRAMMING))

    bus.transport.hardware.set_timeout(12)

    print("[*] Set timing parameters to maximum")
    try:
        available_timing = bus.execute(
            kwp2000.commands.AccessTimingParameters().read_limits_of_possible_timing_parameters()
        ).get_data()

        bus.execute(
            kwp2000.commands.AccessTimingParameters().set_timing_parameters_to_given_values(
                *available_timing[1:]
            )
        )
    except kwp2000.Kwp2000NegativeResponseException:
        print("[!] Not supported on this ECU!")

    print("[*] Security Access")
    enable_security_access_compat(bus)

    ecu = cli_identify_ecu(bus)
    if not ecu:
        raise RuntimeError("ECU identification failed.")

    ecu.set_desired_baudrate(desired_baudrate)
    ecu.diagnostic_session_type = kwp2000.enums.DiagnosticSession.FLASH_REPROGRAMMING
    ecu.access_level = AccessLevel.HYUNDAI_0x1

    ecu.bus.execute(StartDiagnosticSession(DiagnosticSession.DEFAULT, ecu.get_desired_baudrate().index))
    return bus, ecu


def enable_security_access_compat(bus: kwp2000.Kwp2000Protocol) -> None:
    seed = bus.execute(kwp2000.commands.SecurityAccess().request_seed()).get_data()[1:]

    if sum(seed) == 0:
        return

    key_int = calculate_key(int.from_bytes(seed, "big"))
    key_bytes = key_int.to_bytes(2, "big")

    try:
        bus.execute(kwp2000.commands.SecurityAccess().send_key(key_bytes))
    except AttributeError as exc:
        if "to_bytes" not in str(exc):
            raise
        bus.execute(kwp2000.commands.SecurityAccess().send_key(key_int))


class LogState:
    def __init__(self, labels: list[str], interval_ms: int, config: dict, args: argparse.Namespace):
        self.labels = labels
        self.interval_ms = interval_ms
        self.config = config
        self.args = args
        self.last = None
        self.error = None
        self.running = True
        self.lock = threading.Lock()
        self.clients = set()
        self.connected = False
        self.connecting = False
        self.bus = None
        self.ecu = None
        self.log_thread = None


def log_loop(state: LogState, ecu):
    try:
        while state.running and state.connected:
            payload = {"ts": int(time.time() * 1000), "values": poll_values(ecu)}
            state.last = payload
            with state.lock:
                for client_queue in list(state.clients):
                    try:
                        client_queue.put_nowait(payload)
                    except queue.Full:
                        try:
                            client_queue.get_nowait()
                        except queue.Empty:
                            pass
                        try:
                            client_queue.put_nowait(payload)
                        except queue.Full:
                            pass
            if state.interval_ms > 0:
                time.sleep(state.interval_ms / 1000)
    except Exception as exc:
        with state.lock:
            state.error = str(exc)
            state.connected = False
            state.connecting = False
            if state.bus:
                try:
                    state.bus.close()
                except Exception:
                    pass
                state.bus = None
            state.ecu = None
    finally:
        with state.lock:
            state.log_thread = None


def start_logging(state: LogState) -> None:
    if state.log_thread and state.log_thread.is_alive():
        return
    state.log_thread = threading.Thread(target=log_loop, args=(state, state.ecu), daemon=True)
    state.log_thread.start()


def connect_ecu(state: LogState) -> None:
    with state.lock:
        if state.connecting or state.connected:
            return
        state.connecting = True
        state.error = None

    bus = None
    try:
        bus, ecu = setup_ecu(state.config, state.args)
    except Exception as exc:
        with state.lock:
            state.error = str(exc)
            state.connecting = False
        if bus is not None:
            try:
                bus.close()
            except Exception:
                pass
        return

    with state.lock:
        state.bus = bus
        state.ecu = ecu
        state.connected = True
        state.connecting = False
    start_logging(state)


def make_handler(state: LogState, index_path: Path):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.end_headers()
                with open(index_path, "rb") as handle:
                    self.wfile.write(handle.read())
                return

            if self.path == "/status":
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                body = {
                    "running": state.running,
                    "connected": state.connected,
                    "connecting": state.connecting,
                    "error": state.error,
                }
                self.wfile.write(json.dumps(body).encode("utf-8"))
                return

            if self.path == "/stream":
                client_queue = queue.Queue(maxsize=1)
                with state.lock:
                    state.clients.add(client_queue)

                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "keep-alive")
                self.end_headers()

                labels_payload = json.dumps(state.labels)
                self.wfile.write(f"event: labels\ndata: {labels_payload}\n\n".encode("utf-8"))
                self.wfile.flush()

                if state.last is not None:
                    self.wfile.write(f"event: sample\ndata: {json.dumps(state.last)}\n\n".encode("utf-8"))
                    self.wfile.flush()

                try:
                    while state.running:
                        try:
                            payload = client_queue.get(timeout=10)
                            self.wfile.write(f"event: sample\ndata: {json.dumps(payload)}\n\n".encode("utf-8"))
                        except queue.Empty:
                            self.wfile.write(b": ping\n\n")
                        self.wfile.flush()
                except (ConnectionResetError, BrokenPipeError):
                    pass
                finally:
                    with state.lock:
                        state.clients.discard(client_queue)
                return

            self.send_response(404)
            self.end_headers()

        def do_POST(self):
            if self.path == "/connect":
                threading.Thread(target=connect_ecu, args=(state,), daemon=True).start()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.end_headers()
                body = {
                    "connected": state.connected,
                    "connecting": state.connecting,
                    "error": state.error,
                }
                self.wfile.write(json.dumps(body).encode("utf-8"))
                return

            self.send_response(404)
            self.end_headers()

        def log_message(self, format, *args):
            return

    return Handler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="GKFlasher live logger server")
    parser.add_argument("-p", "--protocol", help="Protocol to use. canbus or kline")
    parser.add_argument("-i", "--interface")
    parser.add_argument("-b", "--baudrate", type=int)
    parser.add_argument("--desired-baudrate", type=lambda x: int(x, 0))
    parser.add_argument("-c", "--config", default="gkflasher.yml")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--interval", type=int, default=200, help="Polling interval in ms (0 = max speed)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = build_config(args)

    index_path = BASE_DIR / "web" / "index.html"
    if not index_path.exists():
        print(f"[!] Missing UI at {index_path}")
        return 1

    state = LogState(parameter_labels(), args.interval, config, args)

    server = ThreadingHTTPServer((args.host, args.port), make_handler(state, index_path))
    print(f"[*] Server running at http://{args.host}:{args.port}")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        state.running = False
        server.shutdown()
        if state.bus:
            state.bus.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
