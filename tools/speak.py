#!/usr/bin/env python3
"""Send audio from this computer to the delivery box speaker (micro-ROS).

  python3 speak.py --wav hello.wav          # play a WAV file on the box
  python3 speak.py --live                   # talk into this computer's microphone
  python3 speak.py --tone 440 --seconds 2   # a test tone
  python3 speak.py --wav hello.wav --volume 70

The audio goes out in small pieces: <prefix>/audio (std_msgs/Int16MultiArray,
best effort), 160 samples = 10 ms of 16 kHz mono int16 per message, sent in
real time. layout.data_offset is the index of the first sample (0 = a new
stream): the box plays a lost piece as silence and keeps the timing. The box
upsamples to its speaker rate. Mirror image of listen_live.py.

Any WAV works (8/16/24/32-bit PCM, mono or stereo, any rate): it is converted
to 16 kHz mono here. --live needs `arecord` (alsa-utils).

Run it from a shell with ROS 2 sourced (NOT the ESP-IDF build shell):
  source /opt/ros/jazzy/setup.bash
"""
import argparse
import array
import math
import signal
import subprocess
import sys
import threading
import time
import wave

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int16MultiArray, String

RATE = 16000
CHUNK = 160                       # samples per message (10 ms)


# ---- audio sources: each yields lists/arrays of int16 samples at RATE -------------

def resample(samples, rate_in, rate_out):
    """Linear-interpolation resampler, with a simple low-pass when going down."""
    if rate_in == rate_out:
        return samples
    if rate_out < rate_in:
        # moving average over ~ the ratio: crude anti-aliasing for speech
        k = max(1, int(round(rate_in / rate_out)))
        if k > 1:
            acc, out = 0, array.array("h")
            window = []
            for s in samples:
                window.append(s)
                acc += s
                if len(window) > k:
                    acc -= window.pop(0)
                out.append(int(acc / len(window)))
            samples = out
    n_out = int(len(samples) * rate_out / rate_in)
    out = array.array("h", bytes(2 * n_out))
    step = rate_in / rate_out
    last = len(samples) - 1
    for i in range(n_out):
        x = i * step
        j = int(x)
        f = x - j
        a = samples[min(j, last)]
        b = samples[min(j + 1, last)]
        out[i] = int(a + (b - a) * f)
    return out


def read_wav(path, gain):
    with wave.open(path, "rb") as w:
        if w.getcomptype() != "NONE":
            sys.exit("only uncompressed PCM WAV files are supported")
        ch, width, rate = w.getnchannels(), w.getsampwidth(), w.getframerate()
        raw = w.readframes(w.getnframes())
    # to int16 (take the top 16 bits of wider samples; 8-bit WAV is unsigned)
    if width == 2:
        data = array.array("h", raw)
    elif width == 1:
        data = array.array("h", ((b - 128) << 8 for b in raw))
    elif width in (3, 4):
        data = array.array("h", (int.from_bytes(raw[i + width - 2:i + width], "little", signed=True)
                                 for i in range(0, len(raw), width)))
    else:
        sys.exit("unsupported sample width: %d bytes" % width)
    if sys.byteorder == "big":
        data.byteswap()
    if ch > 1:                                     # mono = average of the channels
        data = array.array("h", (sum(data[i:i + ch]) // ch for i in range(0, len(data), ch)))
    data = resample(data, rate, RATE)
    if gain != 1.0:
        data = array.array("h", (max(-32768, min(32767, int(s * gain))) for s in data))
    print("%s: %d ch, %d Hz, %d-bit, %.1f s -> 16 kHz mono" % (
        path, ch, rate, 8 * width, len(data) / RATE), flush=True)
    for i in range(0, len(data), CHUNK):
        yield data[i:i + CHUNK]


def tone(hz, seconds, level):
    amp = level * 32767
    n = int(seconds * RATE)
    for start in range(0, n, CHUNK):
        yield array.array("h", (int(amp * math.sin(2 * math.pi * hz * i / RATE))
                                for i in range(start, min(start + CHUNK, n))))


def live(device, gain):
    cmd = ["arecord", "-q", "-t", "raw", "-f", "S16_LE", "-r", str(RATE), "-c", "1"]
    if device:
        cmd += ["-D", device]
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE)
    except FileNotFoundError:
        sys.exit("arecord not found (sudo apt install alsa-utils)")
    print("live: talk now, Ctrl+C to stop", flush=True)
    try:
        while True:
            raw = proc.stdout.read(CHUNK * 2)
            if not raw:
                break
            data = array.array("h", raw)
            if gain != 1.0:
                data = array.array("h", (max(-32768, min(32767, int(s * gain))) for s in data))
            yield data
    finally:
        proc.terminate()


# ---- ROS -------------------------------------------------------------------------

class Speaker(Node):
    def __init__(self, prefix):
        super().__init__("box_speaker")
        p = prefix.rstrip("/")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=50)
        self.pub = self.create_publisher(Int16MultiArray, p + "/speaker/audio", qos)
        self.pub_sound = self.create_publisher(String, p + "/sound", 10)
        self.status = None
        self.create_subscription(String, p + "/speaker/status", self.on_status, 10)

    def on_status(self, msg):
        self.status = msg.data

    def send(self, samples, offset):
        m = Int16MultiArray()
        m.layout.data_offset = offset
        m.data = samples
        self.pub.publish(m)

    def sound_cmd(self, text):
        m = String()
        m.data = text
        self.pub_sound.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--wav", help="WAV file to play on the box")
    src.add_argument("--live", action="store_true", help="stream this computer's microphone")
    src.add_argument("--tone", type=float, metavar="HZ", help="test tone")
    ap.add_argument("--seconds", type=float, default=2.0, help="--tone length (default 2)")
    ap.add_argument("--level", type=float, default=0.3, help="--tone level 0..1 (default 0.3)")
    ap.add_argument("--gain", type=float, default=1.0, help="multiply the audio (default 1)")
    ap.add_argument("--device", help="--live: ALSA capture device (arecord -L)")
    ap.add_argument("--volume", type=int, help="set the box speaker volume 0..100 first")
    ap.add_argument("--lead-ms", type=int, default=80,
                    help="send this much ahead of real time (default 80; the box "
                         "buffers up to 500 ms)")
    ap.add_argument("--topic-prefix", default="/delivery_box")
    args = ap.parse_args()

    rclpy.init(signal_handler_options=rclpy.SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    node = Speaker(args.topic_prefix)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    # Give discovery a moment, so the first pieces are not lost.
    time.sleep(1.0)
    if node.status is None:
        print("note: no speaker/status from the box yet (agent running? board connected?)",
              flush=True)
    if args.volume is not None:
        node.sound_cmd("volume %d" % args.volume)

    if args.wav:
        source = read_wav(args.wav, args.gain)
    elif args.tone:
        source = tone(args.tone, args.seconds, args.level)
    else:
        source = live(args.device, args.gain)

    sent = 0
    t0 = time.monotonic()
    lead = args.lead_ms / 1000.0
    try:
        for samples in source:
            if not args.live:
                # pace to real time (live audio is paced by the microphone)
                due = t0 + sent / RATE - lead
                delay = due - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
            node.send(samples, sent)
            sent += len(samples)
            if sent % RATE < CHUNK:
                print("sent %5.1f s | box: %s" % (sent / RATE, node.status or "?"), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(lead + 0.3)                     # let the box play the tail
        print("sent %.1f s of audio | box: %s" % (sent / RATE, node.status or "?"))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
