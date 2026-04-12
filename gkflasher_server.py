import argparse
import ast
import json
import os
import queue
import re
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
from flasher.logging import data_sources
from gkflasher import initialize_bus, cli_identify_ecu


def load_config(config_path: str) -> dict:
    path = Path(config_path)
    if not path.is_absolute():
        path = PROJECT_DIR / path
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def load_optional_config(config_path: str) -> dict:
    path = Path(config_path)
    if not path.is_absolute():
        path = BASE_DIR / path
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def build_config(args: argparse.Namespace) -> dict:
    config = load_config(args.config)
    if args.protocol:
        config["protocol"] = args.protocol
    if args.interface:
        config[config["protocol"]]["interface"] = args.interface
    if args.baudrate and not should_treat_baudrate_as_desired(args, config):
        config[config["protocol"]]["baudrate"] = args.baudrate
    return config


def is_vehicle_speed_value(title: str, units: str) -> bool:
    title_lc = str(title or "").strip().lower()
    units_lc = str(units or "").strip().lower()
    return "vehicle speed" in title_lc and units_lc == "mph"


def normalize_label(title: str, units: str) -> str:
    display_units = "km/h" if is_vehicle_speed_value(title, units) else units
    return f"{title} ({display_units})" if display_units else title


class SafeExpression:
    ALLOWED_NODES = (
        ast.Expression,
        ast.BinOp,
        ast.UnaryOp,
        ast.Add,
        ast.Sub,
        ast.Mult,
        ast.Div,
        ast.Pow,
        ast.USub,
        ast.UAdd,
        ast.Load,
        ast.Name,
        ast.Constant,
    )

    def __init__(self, expression: str):
        self.expression = expression.strip()
        parsed = ast.parse(self.expression, mode="eval")
        for node in ast.walk(parsed):
            if not isinstance(node, self.ALLOWED_NODES):
                raise ValueError(f"Unsupported expression node: {type(node).__name__}")
        self.code = compile(parsed, "<adx-math>", "eval")

    def evaluate(self, variables: dict[str, float]) -> float:
        return eval(self.code, {"__builtins__": {}}, variables)


class AdxValueDefinition:
    def __init__(
        self,
        *,
        value_id: str,
        id_hash: str,
        title: str,
        units: str,
        packet_offset: int | None,
        size_bits: int,
        signed: bool,
        lsb_first: bool,
        equation: str | None,
        variables: list[dict[str, str]],
    ):
        self.value_id = value_id
        self.id_hash = id_hash
        self.title = title
        self.units = units
        self.label = normalize_label(title, units)
        self.convert_mph_to_kmh = is_vehicle_speed_value(title, units)
        self.packet_offset = packet_offset
        self.size_bits = size_bits
        self.signed = signed
        self.lsb_first = lsb_first
        self.variables = variables
        self.expression = SafeExpression(equation) if equation else None

    def extract_native(self, payload: bytes) -> int:
        if self.packet_offset is None:
            raise ValueError(f"{self.value_id} has no packet offset")
        byte_count = max(1, self.size_bits // 8)
        chunk = payload[self.packet_offset : self.packet_offset + byte_count]
        if len(chunk) < byte_count:
            raise ValueError(f"{self.value_id} payload too short")
        byteorder = "little" if self.lsb_first else "little"
        return int.from_bytes(chunk, byteorder=byteorder, signed=self.signed)


class AdxDecoder:
    def __init__(self, definitions: list[AdxValueDefinition], excluded_ids: list[str] | None = None):
        self.by_id = {definition.value_id: definition for definition in definitions}
        self.by_hash = {definition.id_hash.upper(): definition for definition in definitions if definition.id_hash}
        excluded = set(excluded_ids or [])
        self.selected_ids = [definition.value_id for definition in definitions if definition.value_id not in excluded]

    def labels(self) -> list[str]:
        return [self.by_id[value_id].label for value_id in self.selected_ids]

    def decode(self, payload: bytes) -> list[float | None]:
        cache: dict[str, float | None] = {}
        return [self._decode_value(self.by_id[value_id], payload, cache) for value_id in self.selected_ids]

    def _decode_value(
        self,
        definition: AdxValueDefinition,
        payload: bytes,
        cache: dict[str, float | None],
    ) -> float | None:
        if definition.value_id in cache:
            return cache[definition.value_id]

        try:
            if not definition.expression:
                result = float(definition.extract_native(payload))
            else:
                variables: dict[str, float] = {}
                for variable in definition.variables:
                    var_id = variable.get("varID", "")
                    var_type = variable.get("type", "native")
                    if var_type == "native":
                        variables[var_id] = float(definition.extract_native(payload))
                    elif var_type == "link":
                        link_hash = (variable.get("linkIDHash") or "").upper()
                        linked_definition = self.by_hash.get(link_hash)
                        if linked_definition is None:
                            raise ValueError(f"Missing link target {link_hash}")
                        linked_value = self._decode_value(linked_definition, payload, cache)
                        if linked_value is None:
                            raise ValueError(f"Linked value missing for {link_hash}")
                        variables[var_id] = float(linked_value)
                    else:
                        raise ValueError(f"Unsupported variable type {var_type}")
                result = float(definition.expression.evaluate(variables))
        except Exception:
            result = None

        if result is not None and definition.convert_mph_to_kmh:
            result *= 1.609344

        cache[definition.value_id] = result
        return result


def load_adx_decoder(adx_path: str, excluded_ids: list[str] | None = None) -> AdxDecoder:
    path = Path(adx_path)
    if not path.is_absolute():
        path = BASE_DIR / path
    tree = yaml = None
    import xml.etree.ElementTree as ET

    tree = ET.parse(path)
    root = tree.getroot()
    defaults = root.find("ADXHEADER/DEFAULTS")
    default_size_bits = int(defaults.get("datasizeinbits", "8")) if defaults is not None else 8
    default_signed = (defaults.get("signed", "0") == "1") if defaults is not None else False
    default_lsb_first = (defaults.get("lsbfirst", "0") == "1") if defaults is not None else False

    definitions: list[AdxValueDefinition] = []
    for value in root.findall(".//ADXVALUE"):
        value_id = value.get("id")
        if not value_id:
            continue
        math_node = value.find("MATH")
        equation = math_node.get("equation") if math_node is not None else None
        variables = [node.attrib for node in math_node.findall("VAR")] if math_node is not None else []
        packet_offset_text = value.findtext("packetoffset")
        packet_offset = int(packet_offset_text, 16) if packet_offset_text else None
        size_bits = int(value.findtext("sizeinbits") or default_size_bits)
        signed_text = value.findtext("signed")
        signed = default_signed if signed_text is None else signed_text == "1"
        lsb_first_text = value.findtext("lsbfirst")
        lsb_first = default_lsb_first if lsb_first_text is None else lsb_first_text == "1"
        units = value.findtext("units") or ""
        definitions.append(
            AdxValueDefinition(
                value_id=value_id,
                id_hash=(value.get("idhash") or "").upper(),
                title=value.get("title") or value_id,
                units=units,
                packet_offset=packet_offset,
                size_bits=size_bits,
                signed=signed,
                lsb_first=lsb_first,
                equation=equation,
                variables=variables,
            )
        )
    return AdxDecoder(definitions, excluded_ids)


def should_treat_baudrate_as_desired(args: argparse.Namespace, config: dict) -> bool:
    if args.desired_baudrate is not None:
        return False
    if config.get("protocol") != "kline":
        return False
    return args.baudrate in BAUDRATES.values()


def resolve_desired_baudrate_index(args: argparse.Namespace, config: dict) -> int | None:
    if args.desired_baudrate is not None:
        return args.desired_baudrate
    if not should_treat_baudrate_as_desired(args, config):
        return None
    for index, baudrate in BAUDRATES.items():
        if baudrate == args.baudrate:
            return index
    return None


def poll_sample(ecu, decoder: AdxDecoder) -> dict:
    raw_packets = []
    for index, source in enumerate(data_sources):
        raw_data = ecu.bus.execute(source["payload"]).get_data()
        raw_packets.append(
            {
                "source_index": index,
                "hex": raw_data.hex().upper(),
            }
        )
    primary_payload = bytes.fromhex(raw_packets[0]["hex"]) if raw_packets else b""
    values = decoder.decode(primary_payload) if raw_packets else []
    return {"values": values, "raw_packets": raw_packets}


def setup_ecu(config: dict, args: argparse.Namespace):
    print(f"[*] Selected protocol: {config['protocol']}. Initializing..")
    bus = initialize_bus(config["protocol"], config[config["protocol"]])

    bus.init(
        kwp2000.commands.StartCommunication(),
        keepalive_command=kwp2000.commands.TesterPresent(kwp2000.enums.ResponseType.REQUIRED),
        keepalive_delay=1.5,
    )
    bus.transport.set_buffer_size(20)

    desired_baudrate_index = resolve_desired_baudrate_index(args, config)

    if desired_baudrate_index is not None:
        try:
            desired_baudrate = DesiredBaudrate(
                index=desired_baudrate_index, baudrate=BAUDRATES[desired_baudrate_index]
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
    def __init__(self, labels: list[str], interval_ms: int, config: dict, args: argparse.Namespace, decoder: AdxDecoder):
        self.labels = labels
        self.interval_ms = interval_ms
        self.config = config
        self.args = args
        self.decoder = decoder
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
            sample = poll_sample(ecu, state.decoder)
            payload = {
                "ts": int(time.time() * 1000),
                "values": sample["values"],
                "raw_packets": sample["raw_packets"],
            }
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
    web_root = index_path.parent

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

            if self.path.startswith("/"):
                rel_path = self.path.lstrip("/")
                file_path = (web_root / rel_path).resolve()
                try:
                    file_path.relative_to(web_root)
                except ValueError:
                    file_path = None

                if file_path and file_path.is_file():
                    content_type = "application/octet-stream"
                    suffix = file_path.suffix.lower()
                    if suffix == ".js":
                        content_type = "application/javascript; charset=utf-8"
                    elif suffix == ".css":
                        content_type = "text/css; charset=utf-8"
                    elif suffix == ".html":
                        content_type = "text/html; charset=utf-8"
                    elif suffix == ".svg":
                        content_type = "image/svg+xml"
                    elif suffix in (".png", ".jpg", ".jpeg", ".ico"):
                        content_type = f"image/{suffix.lstrip('.')}"
                    elif suffix == ".map":
                        content_type = "application/json; charset=utf-8"

                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.end_headers()
                    with open(file_path, "rb") as handle:
                        self.wfile.write(handle.read())
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

            if self.path.startswith("/"):
                rel_path = self.path.lstrip("/")
                file_path = (web_root / rel_path).resolve()
                try:
                    file_path.relative_to(web_root)
                except ValueError:
                    file_path = None

                if file_path and file_path.is_file():
                    content_type = "application/octet-stream"
                    suffix = file_path.suffix.lower()
                    if suffix == ".js":
                        content_type = "application/javascript; charset=utf-8"
                    elif suffix == ".css":
                        content_type = "text/css; charset=utf-8"
                    elif suffix == ".html":
                        content_type = "text/html; charset=utf-8"
                    elif suffix == ".svg":
                        content_type = "image/svg+xml"
                    elif suffix in (".png", ".jpg", ".jpeg", ".ico"):
                        content_type = f"image/{suffix.lstrip('.')}"
                    elif suffix == ".map":
                        content_type = "application/json; charset=utf-8"

                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.end_headers()
                    with open(file_path, "rb") as handle:
                        self.wfile.write(handle.read())
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
    parser.add_argument("--logger-config", default="excluded_pids.yml")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--interval", type=int, default=0, help="Polling interval in ms (0 = max speed)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = build_config(args)
    logger_config = load_optional_config(args.logger_config)
    adx_path = logger_config.get("adx_path")
    if not adx_path:
        print("[!] Missing 'adx_path' in logger config")
        return 1
    excluded_ids = logger_config.get("excluded_pids") or None
    decoder = load_adx_decoder(adx_path, excluded_ids)

    index_path = BASE_DIR / "web" / "index.html"
    if not index_path.exists():
        print(f"[!] Missing UI at {index_path}")
        return 1

    state = LogState(decoder.labels(), args.interval, config, args, decoder)

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
