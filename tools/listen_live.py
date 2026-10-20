#!/usr/bin/env python3
"""Play the live microphone stream of the delivery box (micro-ROS) on this computer.

  python3 listen_live.py                 # switch the mic on, play it, switch it off on Ctrl+C
  python3 listen_live.py --wav talk.wav  # also save what was heard
  python3 listen_live.py --no-play       # only statistics (and --wav)
  python3 listen_live.py --no-enable     # somebody else switches the mic on/off

Topics (see README.md): <prefix>/enable (Bool, in), <prefix>/status (Bool, out),
<prefix>/audio (Int16MultiArray, 160 samples = 10 ms of 16 kHz mono; layout.data_offset
is the index of the first sample, used to fill lost chunks with silence).

Run it from a shell with ROS 2 sourced (NOT the ESP-IDF build shell):
  source /opt/ros/jazzy/setup.bash
Playback needs `aplay` (alsa-utils); nothing else is installed.
"""
import argparse
import math
import queue
import signal
import subprocess
import sys
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int16MultiArray

RATE = 16000
CHUNK = 160                      # samples per message (10 ms)
UNDERRUN_FILL = bytes(int(RATE * 0.03) * 2)   # 30 ms, the queue wait below
MAX_FILL_SAMPLES = RATE          # a gap longer than 1 s is a restart, not loss


def dbfs(rms):
    return 20 * math.log10(rms / 32768) if rms > 0 else -120.0


class Listener(Node):
    def __init__(self, args):
        super().__init__("mic_listener")
        self.args = args
        self.q = queue.Queue()
        self.max_chunks = max(2, int(args.max_latency * RATE / CHUNK))
        self.expected = None          # next data_offset we expect
        self.status = None
        self.enable_wanted = not args.no_enable
        self.t_start = time.monotonic()

        # per-second statistics
        self.rx = self.lost = self.dropped = self.restarts = 0
        self.sumsq = 0.0
        self.nsamp = 0
        self.last_rx = None
        self.max_gap_ms = 0.0
        self.total_rx = self.total_lost = 0

        p = args.topic_prefix.rstrip("/")
        audio_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               history=HistoryPolicy.KEEP_LAST, depth=200)
        self.create_subscription(Int16MultiArray, p + "/audio", self.on_audio, audio_qos)
        self.create_subscription(Bool, p + "/status", self.on_status, 10)
        self.pub_enable = self.create_publisher(Bool, p + "/enable", 10)

        self.create_timer(1.0, self.report)
        self.enable_timer = None
        if self.enable_wanted:
            self.enable_timer = self.create_timer(0.5, self.ask_enable)

    # ---- mic on/off ----------------------------------------------------------
    def set_enable(self, value):
        m = Bool()
        m.data = value
        self.pub_enable.publish(m)

    def ask_enable(self):
        # Ask until the board reports ON for the first time (it creates its
        # subscription only after connecting to the agent), then stop for good:
        # a later OFF from anybody else must stay OFF.
        if self.status is True:
            self.enable_timer.cancel()
            return
        self.set_enable(True)
        if time.monotonic() - self.t_start > 15 and self.status is None:
            self.get_logger().warn("no /status from the board: is the agent running "
                                   "and the board connected?")

    def on_status(self, msg):
        if self.status != msg.data:
            self.get_logger().info("microphone is %s" % ("ON" if msg.data else "OFF"))
        self.status = msg.data
        if msg.data and self.enable_timer is not None:
            self.enable_timer.cancel()          # it is on: stop asking

    # ---- audio ---------------------------------------------------------------
    def on_audio(self, msg):
        now = time.monotonic()
        data = msg.data
        raw = data.tobytes() if hasattr(data, "tobytes") else \
            b"".join(int(v).to_bytes(2, "little", signed=True) for v in data)
        n = len(raw) // 2
        offset = msg.layout.data_offset

        if self.last_rx is not None:
            self.max_gap_ms = max(self.max_gap_ms, (now - self.last_rx) * 1000)
        self.last_rx = now

        if self.expected is not None:
            if offset > self.expected:
                gap = offset - self.expected
                if gap <= MAX_FILL_SAMPLES:
                    self.lost += gap // CHUNK
                    self.total_lost += gap // CHUNK
                    self.q.put(bytes(gap * 2))          # silence keeps the timing right
                else:
                    self.restarts += 1
            elif offset < self.expected:
                self.restarts += 1                       # stream restarted on the board
        self.expected = offset + n

        self.rx += 1
        self.total_rx += 1
        samples = memoryview(raw).cast("h")
        self.sumsq += sum(s * s for s in samples)
        self.nsamp += n
        self.q.put(raw)

        # Never let the delay grow: drop the oldest audio when too far behind.
        while self.q.qsize() > self.max_chunks:
            try:
                self.q.get_nowait()
                self.dropped += 1
            except queue.Empty:
                break

    def report(self):
        rms = math.sqrt(self.sumsq / self.nsamp) if self.nsamp else 0.0
        total = self.rx + self.lost
        loss = 100.0 * self.lost / total if total else 0.0
        print("rx %3d chunks/s | lost %3d (%4.1f%%) | dropped(late) %d | restarts %d | "
              "level %6.1f dBFS | max gap %5.1f ms | queue %3d ms | mic %s" % (
                  self.rx, self.lost, loss, self.dropped, self.restarts, dbfs(rms),
                  self.max_gap_ms, self.q.qsize() * 10,
                  {True: "ON", False: "OFF", None: "?"}[self.status]), flush=True)
        self.rx = self.lost = self.dropped = self.restarts = 0
        self.sumsq = 0.0
        self.nsamp = 0
        self.max_gap_ms = 0.0


def player(q, stop, args):
    """Feed aplay / the wav file from the queue, in order, with a small prebuffer."""
    proc = None
    if not args.no_play:
        try:
            proc = subprocess.Popen(
                ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(RATE), "-c", "1",
                 "-B", str(int(args.buffer_ms * 1000))], stdin=subprocess.PIPE)
        except FileNotFoundError:
            print("aplay not found (sudo apt install alsa-utils); continuing without sound")
    wav = None
    if args.wav:
        wav = wave.open(args.wav, "wb")
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(RATE)

    prebuffer = max(1, int(args.prebuffer_ms / 10))
    while not stop.is_set() and q.qsize() < prebuffer:
        time.sleep(0.005)
    started = False
    while not stop.is_set():
        fill = False
        try:
            chunk = q.get(timeout=0.03)
            started = True
        except queue.Empty:
            if not started:
                continue
            chunk, fill = UNDERRUN_FILL, True           # underrun: keep aplay fed
        try:
            if proc and proc.stdin:
                proc.stdin.write(chunk)
                proc.stdin.flush()
            if wav and not fill:                        # the file holds real audio only
                wav.writeframes(chunk)
        except BrokenPipeError:
            break
    if proc and proc.stdin:
        try:
            proc.stdin.close()
        except OSError:
            pass
        proc.wait(timeout=2)
    if wav:
        wav.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--topic-prefix", default="/delivery_box/mic")
    ap.add_argument("--no-enable", action="store_true",
                    help="do not switch the mic on/off, only listen")
    ap.add_argument("--no-play", action="store_true", help="no sound, statistics only")
    ap.add_argument("--wav", help="also save the audio to this .wav file")
    ap.add_argument("--prebuffer-ms", type=int, default=100)
    ap.add_argument("--max-latency", type=float, default=0.3,
                    help="seconds of audio we may fall behind before dropping (default 0.3)")
    ap.add_argument("--buffer-ms", type=int, default=60, help="aplay buffer (default 60)")
    args = ap.parse_args()

    # Keep the ROS context alive on Ctrl+C so the "mic off" message can still be sent.
    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = Listener(args)
    stop = threading.Event()
    th = threading.Thread(target=player, args=(node.q, stop, args), daemon=True)
    th.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not args.no_enable:
            for _ in range(3):                  # make sure the mic goes off
                try:
                    node.set_enable(False)
                except Exception:               # context already gone
                    break
                time.sleep(0.05)
        stop.set()
        th.join(timeout=3)
        print("\nreceived %d chunks, %d lost" % (node.total_rx, node.total_lost))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
