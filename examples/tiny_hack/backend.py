#!/usr/bin/env python3
import os, time, json, threading
from datetime import datetime
from io import BytesIO

from flask import Flask, jsonify, request, send_file, url_for
from flask_cors import CORS
from PIL import Image

import serial
from serial.tools import list_ports

# ---------------- Config ----------------
BAUD = 115200
PORT_HINTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyUSB0"]
SAVE_DIR = os.path.join(os.path.dirname(__file__), "images")
os.makedirs(SAVE_DIR, exist_ok=True)

# ---------------- Serial client ----------------
class NiclaClient:
    def __init__(self):
        self._ser = None
        self._lock = threading.Lock()

    def _find_port(self):
        for p in PORT_HINTS:
            if os.path.exists(p):
                return p
        for p in list_ports.comports():
            desc = (p.manufacturer or "") + " " + (p.description or "")
            if any(k in desc for k in ["Arduino", "Nicla", "H7", "STM"]):
                return p.device
        return None

    def _open(self):
        if self._ser and self._ser.is_open:
            return
        port = self._find_port()
        if not port:
            raise RuntimeError("Nicla serial port not found")
        self._ser = serial.Serial(port, BAUD, timeout=2)
        time.sleep(0.2)
        self._ser.reset_input_buffer()

    def _send(self, cmd: str):
        self._ser.write((cmd + "\n").encode("utf-8"))
        self._ser.flush()

    def _read_line(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                return line
        return ""

    def _read_json_line(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self._read_line(timeout=deadline - time.time())
            if not line:
                continue
            if not (line.startswith("{") and line.endswith("}")):
                # ignore non-JSON chatter
                continue
            try:
                return json.loads(line)
            except Exception:
                continue
        return None

    def _read_until(self, expect: str, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._read_line(timeout=deadline - time.time()) == expect:
                return True
        return False

    def _read_image_ppm(self, timeout=5.0):
        if not self._read_until("BEGIN_IMAGE", timeout=timeout):
            return None
        magic = self._read_line(timeout=1.0)     # P6
        dims  = self._read_line(timeout=1.0)     # "W H"
        maxv  = self._read_line(timeout=1.0)     # "255"
        try:
            w, h = map(int, dims.split())
        except Exception:
            return None
        nbytes = w * h * 3
        raw = self._ser.read(nbytes)
        _ = self._ser.readline()                 # trailing newline after raw
        end = self._read_line(timeout=1.0)       # END_IMAGE (or blank + END_IMAGE)
        if end == "":
            end = self._read_line(timeout=1.0)
        # not fatal if end != END_IMAGE
        return (w, h, raw)

    def hello(self):
        with self._lock:
            self._open()
            self._send("HELLO")
            return self._read_json_line(timeout=1.5) or {"type":"hello","status":"unknown"}

    def info(self):
        with self._lock:
            self._open()
            self._send("INFO")
            return self._read_json_line(timeout=1.5) or {"type":"info","status":"unknown"}

    def infer(self, with_image: bool):
        with self._lock:
            self._open()
            self._send("INFER+IMAGE" if with_image else "INFER")
            result = self._read_json_line(timeout=5.0)
            if not isinstance(result, dict):
                return None, None

            filename = None
            if with_image:
                img = self._read_image_ppm(timeout=5.0)
                if img:
                    w, h, raw = img
                    # Convert raw RGB to PNG
                    im = Image.frombytes("RGB", (w, h), raw)
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                    filename = f"nicla_{ts}.png"
                    path = os.path.join(SAVE_DIR, filename)
                    im.save(path, format="PNG", optimize=True)
                # if image failed, we still return result
            return result, filename

nicla = NiclaClient()

# ---------------- Flask app ----------------
app = Flask(__name__)
CORS(app)  # allow your frontend origin in production

@app.get("/health")
def health():
    return jsonify({"ok": True, "time_ms": int(time.time()*1000)})

@app.get("/hello")
def hello():
    try:
        return jsonify(nicla.hello())
    except Exception as e:
        return jsonify({"error": str(e)}), 503

@app.get("/info")
def info():
    try:
        return jsonify(nicla.info())
    except Exception as e:
        return jsonify({"error": str(e)}), 503

@app.post("/infer")
def infer():
    """
    Trigger a single classification.
    Query params:
      image=true|false  (default true)
    Response:
      { class, score, latency_us, image_url? }
    """
    with_image = request.args.get("image", "true").lower() != "false"
    try:
        res, fname = nicla.infer(with_image=with_image)
        if not isinstance(res, dict) or res.get("type") != "result":
            # Handle busy or error gracefully
            if isinstance(res, dict) and res.get("type") in ("busy","error"):
                return jsonify(res), 429 if res.get("type")=="busy" else 500
            return jsonify({"error":"no_result"}), 504

        payload = {
            "class": res.get("class"),
            "score": res.get("score"),
            "latency_us": res.get("latency_us"),
            "timestamp": int(time.time()*1000)
        }
        if fname:
            payload["image_url"] = url_for("get_image", filename=fname, _external=True)
        return jsonify(payload)
    except Exception as e:
        return jsonify({"error": str(e)}), 503

@app.get("/images/<path:filename>")
def get_image(filename):
    path = os.path.join(SAVE_DIR, filename)
    if not os.path.isfile(path):
        return jsonify({"error":"not_found"}), 404
    # Disable caching so the frontend always gets the latest
    return send_file(path, mimetype="image/png", as_attachment=False,
                     download_name=filename, max_age=0, conditional=False)

if __name__ == "__main__":
    port = int(os.environ.get("PORT", "5000"))
    # threaded=True is fine because we hold a lock during serial access
    app.run(host="0.0.0.0", port=port, debug=False, threaded=True)
