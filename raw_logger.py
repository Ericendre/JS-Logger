import csv
import time
from datetime import datetime

from gkbus.protocol.kwp2000.commands import StartDiagnosticSession
from gkbus.protocol.kwp2000.enums import DiagnosticSession

from flasher.logging import poll_raw


def collect_raw_logs(ecu, duration_s=None, max_frames=None):
	"""
	Collect raw KWP frames for a given duration or number of frames.
	Returns a list of (timestamp_ms, bytes) tuples.
	"""
	ecu.bus.execute(
		StartDiagnosticSession(DiagnosticSession.DEFAULT, ecu.get_desired_baudrate().index)
	)

	frames = []
	start_time = time.time()

	while True:
		if duration_s is not None and (time.time() - start_time) >= duration_s:
			break
		if max_frames is not None and len(frames) >= max_frames:
			break

		raw = poll_raw(ecu)[0]
		frames.append((int(time.time() * 1000), raw))

	return frames


def save_raw_logs_csv(frames, output_filename=None):
	if output_filename is None:
		output_filename = "log_raw_{}.csv".format(datetime.now().strftime("%Y-%m-%d_%H%M"))

	with open(output_filename, "w", newline="") as csvfile:
		logwriter = csv.writer(csvfile)
		for ts_ms, payload in frames:
			data_hex = " ".join([hex(x) for x in list(payload)])
			logwriter.writerow([ts_ms, data_hex])

	return output_filename


if __name__ == "__main__":
	print("This module provides collect_raw_logs() and save_raw_logs_csv().")
