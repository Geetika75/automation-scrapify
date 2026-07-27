"""
stereo_autonomous_combined.py
-------------------------------
Autonomous waste collection using stereo cameras.
Communicates with ESP32 over WiFi TCP (ESP hosts the AP + TCP server).

Combines:
  - Full ROI drawing UI, DisparityWorker, RTSPReader, calibration, drawing
    from stereo_autonomous_updated.py (the longer file)
  - Navigation logic (stabilise -> turn -> post-turn stabilise -> move),
    send_turn_interruptible, _nav_lock thread-safety, and scan logic
    from stereo_autonomous.py (the shorter file)

Navigation flow:
  1. Detect all waste inside ROI (LEFT camera)
  2. Pick closest (sorted_targets[0])
  3. Stabilise dist+angle 1.5s (median sampling, fresh last_seen check)
  4. If |angle| > ANGLE_THRESH: turn, settle 1.5s, re-stabilise,
     optional correction turn, pre-move stabilise 1.0s, move
  5. Else: pre-move stabilise 1.0s, move
  6. Mark collected, pick next
  7. ROI empty: scan 15deg steps up to 360deg (interruptible via
     send_turn_interruptible + settle loop)
  8. Full 360deg scan finds nothing: run the stepped-serpentine search
     pattern (see search_pattern() inside main()) until a target is found.

Controls:
    q       quit
    d       toggle disparity window
    e       toggle epipolar lines
    i       toggle debug
    +/-     confidence threshold
    S       draw left ROI
    R       draw right ROI
    c       clear ROI
"""

import cv2
import numpy as np
import math
import threading
import queue
import time
import os
import sys
import json
import socket
import subprocess
from datetime import datetime
from collections import deque

try:
    from ultralytics import YOLO
except ImportError:
    print("ERROR: pip install ultralytics"); sys.exit(1)

try:
    import cv2.ximgproc as ximgproc
    HAS_WLS = True
except ImportError:
    HAS_WLS = False
    print("INFO: WLS disabled")


# ----------------------------------------------------------------------------
#  CONFIGURATION
# ----------------------------------------------------------------------------

RTSP_LEFT  = ("rtsp://admin:Scrapify%40123@192.168.1.102:554/"
              "cam/realmonitor?channel=1&subtype=1&unicast=true&proto=Onvif")
RTSP_RIGHT = ("rtsp://admin:Scrapify%40123@192.168.1.103:554/"
              "cam/realmonitor?channel=1&subtype=1&unicast=true&proto=Onvif")

CALIB_FILE   = "stereo_calib_cl11.npz"
YOLO_MODEL   = "yolo11x.pt"
BOTTLE_CLASS = 39
CONF_THRESH  = 0.25

# SGBM tuned for cl5 (fx=419, baseline=14.95cm)
NUM_DISP   = 128
BLOCK_SIZE = 9
MIN_DISP   = 0
USE_WLS    = True
WLS_LAMBDA = 8000
WLS_SIGMA  = 1.5

# Navigation
ANGLE_THRESH    = 5.0   # deg — turn only if angle beyond this
CLOSE_RANGE_NO_TURN_M = 0.15  # m  — skip alignment turn within this distance;
                              #       at close range the large bounding-box shifts
                              #       outside the ROI after a rotation and the
                              #       small lateral offset is negligible.
CLOSE_RANGE_DIRECT_MOVE_M = 0.40  # m  — if stabilise gets n=0 (YOLO can't see
                              #       the target because it's right under the
                              #       camera), trust the last-known dist and
                              #       proceed rather than aborting the maneuver.
DONE_TIMEOUT_S  = 20.0   # max wait for ESP DONE response

# WiFi / TCP
ESP_HOST               = "192.168.4.1"
ESP_PORT               = 4210
ESP_RECONNECT_INTERVAL = 3.0

# WiFi auto-connect
WIFI_SSID     = "WaterBot"
WIFI_PASSWORD = "jetson"

# Display
DISPLAY_SCALE  = 0.65
SHOW_DISPARITY = True
EPIPOLAR_LINES = True
MAX_DIST_M     = 10.0

# ROI
USE_ROI          = True
ROI_NORM_POLYGON = [(0.10, 0.15), (0.90, 0.15), (0.90, 0.95), (0.10, 0.95)]
ROI_FILE         = "roi_config.json"

# Scan
# NOTE: With the CUAV NEO 3 (IST8310 compass) now wired to the ESP32 over I2C,
# scan turns are compass-verified (closed-loop, ESP stops at the true target
# heading) instead of open-loop timed turns. Because the achieved heading is
# now accurate, the old 450 deg "overscan" (added to compensate for timed-turn
# drift) is no longer needed — a scan is a true 360 deg full turn.
SCAN_STEP_DEG       = 30.0
# SCAN_MAX_DEG = 0.0 skips the preliminary full-circle look-around scan
# entirely and goes straight into the stepped-serpentine lane patrol (whose
# Scan Dance already covers 360 deg of coverage at every lane stop). Set
# this back to 360.0 to restore the old "spin in place once before starting
# the lane patrol" behaviour.
SCAN_MAX_DEG        = 0.0    # was 360.0 — pre-scan skipped, dance covers it
SCAN_STARTUP_GRACE  = 5.0
TARGET_TIMEOUT_S    = 5.0
RAW_SEEN_HOLD_S     = 0.8
MAX_DEPTH_WAIT_TRIES = 3

# Search pattern (runs after a full 360 deg scan finds nothing)
# "Stepped Serpentine" patrol over a SEARCH_LANE_LENGTH_M x
# (SEARCH_LANE_SHIFT_M * (SEARCH_TOTAL_LANES-1)) region.
#
# Each lane is walked in STEP_DISTANCE_M increments up to SEARCH_LANE_LENGTH_M.
# At 0.0m and at every step short of the lane boundary, the robot performs a
# "Scan Dance": note the lane's centre heading (cheading), then
#   - turn right 90deg via 3 consecutive 30deg compass-verified splits
#   - return to centre with a SINGLE full 90deg compass turn, verify the
#     achieved heading against the noted cheading, and settle/stabilise
#   - turn left 90deg via 3 consecutive 30deg compass-verified splits
#   - return to centre with a SINGLE full 90deg compass turn, verify the
#     achieved heading against the noted cheading, and settle/stabilise
# At the lane boundary (>= SEARCH_LANE_LENGTH_M) it does NOT scan — instead:
#   - turn 90deg toward the shift direction (this heading is the "lheading")
#     and note it via CMD:SETCENTRE
#   - move sideways by SEARCH_LANE_SHIFT_M using a centre-corrected move,
#     stabilising against the lheading the whole way
#   - turn 90deg again (same direction) so the boat faces down the new lane
#   - note the new lane's forward heading (cheading) via CMD:SETCENTRE, reset
#     in-lane distance to 0.0m, and immediately run the 0.0m Scan Dance
# Shift direction alternates every lane (LEFT, RIGHT, LEFT, ... — true
# serpentine).
#
# NOTE: backward moves require the ESP firmware to handle CMD:MOVE with a
#       negative distance value (e.g.  CMD:MOVE -0.30  → reverse 0.30 m).
# NOTE: Every turn in this routine (scan-dance edges/returns, lane-shift
#       turns, and heading restore after a collection interrupt) is
#       compass-verified via CMD:CTURN. Only the navigate()/stabilise/
#       restabilise turns during target approach remain timed (unchanged)
#       — the compass is intentionally NOT used there.
# NOTE: SCAN_STEP_DEG (30.0, defined above) is reused for the Scan Dance's
#       30deg outward splits — no separate constant needed.
SEARCH_LANE_LENGTH_M   = 1.5    # m   — length of each lane before a lane shift
STEP_DISTANCE_M        = 0.5    # m   — forward step between scan-dance stops
SEARCH_LANE_SHIFT_M    = 1.0    # m   — sideways shift between lanes
SEARCH_TOTAL_LANES     = 3      # lanes — 2 shifts of SEARCH_LANE_SHIFT_M -> 3.0m width
SCAN_EDGE_DEG          = 90.0   # deg — scan-dance return-to-centre turn & lane-shift turn angle
# Collect-vs-ignore gate for detections that fire DURING the scan dance.
# For now hardcoded to match STEP_DISTANCE_M (mid-stops are 0.5m apart, so
# anything farther than that is presumably closer to the NEXT stop / a
# neighbouring lane and shouldn't be chased mid-dance). TODO: once the patrol
# region becomes configurable, derive this from the region dimensions instead
# of hardcoding it.
COLLECT_MAX_DIST_M     = STEP_DISTANCE_M   # m — 0.5m: collect if <=, else ignore
SEARCH_SETTLE_S        = 0.4    # s   — pause after each sub-step to check for targets (was 0.8s)
SCAN_EDGE_VERIFY_TOL_DEG = 5.0   # deg — tolerance for the return-to-centre heading check
SCAN_STEP_COARSE_TOL_DEG = 12.0  # deg — widened accept tolerance for the first two 30deg
                                 # splits per side; skips the ESP's reach-verify coast-settle
                                 # since these don't need to be exact — the final split to
                                 # the edge (an absolute-target turn) corrects any drift


def heading_diff_deg(frm, to):
    """Shortest signed angular difference to->frm, wrapped to (-180, 180].
    Mirrors the ESP firmware's headingDiffDeg() so Jetson-side verification
    uses the exact same wrap-around convention."""
    d = (to - frm) % 360.0
    if d > 180.0:
        d -= 360.0
    return d


# Stabilisation
INITIAL_STABILIZE_S        = 1.0   # was 1.5s — n≈10 samples still well above minimum
PRE_MOVE_STABILIZE_S       = 0.6   # was 1.0s — n≈6 samples > STABILISE_MIN_SAMPLES
PRE_MOVE_SAMPLE_INTERVAL_S = 0.10   # how often to sample inside stabilise()
PRE_MOVE_MAX_JUMP_M        = 0.35
STABILISE_MIN_SAMPLES      = 3      # minimum accepted samples; fewer = unreliable
                                    # median and the bot must abort the maneuver

# Settle durations (all in seconds) — centralised for easy tuning
NAV_POST_TURN_SETTLE_S     = 0.8   # was 1.5s — motor stops in ~0.3s; 5s cache covers depth
NAV_POST_MOVE_SETTLE_S     = 1.0   # was 1.5s — keep slightly longer for collection check
SCAN_STEP_SETTLE_S         = 0.8   # was 1.5s — each scan step was burning ~2s total
# Two separate timers after a scan interrupt / abort:
#   NAV_SETTLE: how long the bot needs to physically stop before nav starts
#   RESTART_BLOCK: how long scan is blocked regardless of YOLO / depth state.
#   RESTART_BLOCK must be >= depth recovery time (cache + anon fixes reduced this).
POST_SCAN_NAV_SETTLE_S     = 1.8   # was 3.0s — faster with improved depth handling
POST_SCAN_RESTART_BLOCK_S  = 4.0   # was 7.0s — cache + anon seeding cuts recovery time

DEBUG_DETECTION_REASONS = True

# Depth / detection reliability
DIST_CACHE_GRACE_S   = 5.0    # s  — reuse last valid depth within this window.
                              #       Must exceed turn_time + POST_TURN_SETTLE so cached
                              #       depth survives through a post-turn re-stabilise.
                              #       For a 90° turn (~2.5s) + 0.8s settle + 0.6s gather
                              #       = 3.9s — 5.0s gives comfortable headroom.
DIST_CACHE_HISTORY   = 8      # per-track samples kept for outlier detection
DIST_OUTLIER_RATIO   = 2.2    # reject depth if > ratio × recent median
DIST_MIN_COVERAGE    = 12.0   # % — minimum bbox-pixel coverage to trust depth

# Navigation detection-loss watchdog  →  STOP ESP if target disappears
NAV_LOSS_GRACE_FRAMES = 3     # frames of no detection before logging a warning
NAV_LOSS_STOP_FRAMES  = 4     # was 8 — stop quickly when target leaves frame

# How long to wait for depth to recover when object is visible but depth fails.
# After this duration of persistent depth failure the hold is released so scan
# can continue (object may be too close / outside stereo range).
DEPTH_FAIL_TIMEOUT_S = 1.5   # was 2.5s

COCO_NAMES = [
    'person','bicycle','car','motorcycle','airplane','bus','train','truck','boat',
    'traffic light','fire hydrant','stop sign','parking meter','bench','bird','cat',
    'dog','horse','sheep','cow','elephant','bear','zebra','giraffe','backpack',
    'umbrella','handbag','tie','suitcase','frisbee','skis','snowboard','sports ball',
    'kite','baseball bat','baseball glove','skateboard','surfboard','tennis racket',
    'bottle','wine glass','cup','fork','knife','spoon','bowl','banana','apple',
    'sandwich','orange','broccoli','carrot','hot dog','pizza','donut','cake','chair',
    'couch','potted plant','bed','dining table','toilet','tv','laptop','mouse',
    'remote','keyboard','cell phone','microwave','oven','toaster','sink',
    'refrigerator','book','clock','vase','scissors','teddy bear','hair drier',
    'toothbrush'
]


# ----------------------------------------------------------------------------
#  TIMESTAMPED FILE LOGGER  (tees stdout → log_DD-MM-YY_HH-MM-SS.txt)
# ----------------------------------------------------------------------------

class TeeLogger:
    """Redirects sys.stdout so every printed line is also written to a log file
    with a [DD-MM-YY/HH-MM-SS] timestamp prefix."""

    def __init__(self, filepath):
        self._terminal = sys.__stdout__
        self._file     = open(filepath, 'w', buffering=1, encoding='utf-8')
        self._buf      = ''
        self._lock     = threading.Lock()
        # Write session header
        ts = datetime.now().strftime('%d-%m-%y/%H-%M-%S')
        self._file.write(f"[{ts}] === SESSION START  log={filepath} ===\n")

    def write(self, msg):
        self._terminal.write(msg)
        with self._lock:
            self._buf += msg
            while '\n' in self._buf:
                line, self._buf = self._buf.split('\n', 1)
                ts = datetime.now().strftime('%d-%m-%y/%H-%M-%S')
                self._file.write(f"[{ts}] {line}\n")

    def flush(self):
        self._terminal.flush()
        self._file.flush()

    def close(self):
        with self._lock:
            if self._buf:
                ts = datetime.now().strftime('%d-%m-%y/%H-%M-%S')
                self._file.write(f"[{ts}] {self._buf}\n")
                self._buf = ''
        ts = datetime.now().strftime('%d-%m-%y/%H-%M-%S')
        self._file.write(f"[{ts}] === SESSION END ===\n")
        self._file.close()
        sys.stdout = self._terminal


# ----------------------------------------------------------------------------
#  WiFi AUTO-CONNECT
# ----------------------------------------------------------------------------

def ensure_wifi():
    print(f"[WiFi] Connecting to '{WIFI_SSID}' ...")

    iface = None
    try:
        result = subprocess.run(
            ["nmcli", "-t", "-f", "DEVICE,TYPE", "device"],
            capture_output=True, text=True, timeout=5
        )
        for line in result.stdout.splitlines():
            if ":wifi" in line:
                iface = line.split(":")[0]
                break
    except Exception:
        pass

    if iface is None:
        try:
            for name in os.listdir("/sys/class/net"):
                if os.path.exists(f"/sys/class/net/{name}/wireless"):
                    iface = name
                    break
        except Exception:
            pass

    if iface is None:
        print("[WiFi] Could not detect WiFi interface — skipping auto-connect")
        return

    print(f"[WiFi] Using interface: {iface}")
    print("[WiFi] Scanning for networks...")
    try:
        subprocess.run(
            ["nmcli", "dev", "wifi", "rescan", "ifname", iface],
            capture_output=True, timeout=10
        )
        time.sleep(3)
    except Exception:
        pass

    for attempt in range(10):
        try:
            result = subprocess.run(
                ["nmcli", "dev", "wifi", "connect", WIFI_SSID,
                 "ifname", iface] +
                (["password", WIFI_PASSWORD] if WIFI_PASSWORD else []),
                capture_output=True, text=True, timeout=15
            )
            if result.returncode == 0:
                print("[WiFi] Connected"); return
            if "already" in result.stdout.lower() or "already" in result.stderr.lower():
                print("[WiFi] Already connected"); return
            print(f"[WiFi] Attempt {attempt+1}/10  {result.stderr.strip()[:60]}")
            subprocess.run(["nmcli", "dev", "wifi", "rescan", "ifname", iface],
                           capture_output=True, timeout=5)
            time.sleep(2)
        except FileNotFoundError:
            break
        except Exception as e:
            print(f"[WiFi] nmcli error: {e}"); break

    print("[WiFi] Could not connect via nmcli — continuing anyway")


# ----------------------------------------------------------------------------
#  ANGLE
# ----------------------------------------------------------------------------

def compute_angle(cx, cx0, fx):
    # cx0 is the rectified principal-point x from P1[0,2].
    # Using the true principal point instead of frame_w/2 removes a systematic
    # angular bias (typically 1-3 deg for these cameras).
    # Negated because the RTSP stream is horizontally mirrored: without the
    # negation a physical left-side object appears at high cx and the bot
    # would turn the wrong way.
    return -math.degrees(math.atan2(cx - cx0, fx))


# ----------------------------------------------------------------------------
#  ESP WiFi TCP COMMANDER
#  Includes send_turn_interruptible from stereo_autonomous.py
# ----------------------------------------------------------------------------

class ESPCommander:
    def __init__(self, host=ESP_HOST, port=ESP_PORT, verbose=True):
        self.host       = host
        self.port       = port
        self.verbose    = verbose
        self._sock      = None
        self._sock_lock = threading.Lock()
        self._cmd_lock  = threading.Lock()
        self._done_ev   = threading.Event()
        self._heading_ev = threading.Event()
        self._stop      = threading.Event()
        self._connected = False

        # Last heading the ESP reported. Populated two ways:
        #   - "DONE HEADING=xx.x" riding on every ordinary completion ack
        #     (updated by whatever CMD:CTURN/TURN/MOVE last ran)
        #   - "HEADING:xx.x" from an explicit CMD:GETHEADING query (see
        #     get_heading()), used to snapshot the center heading before a
        #     scan-dance begins, independent of whatever the last motion was.
        self.last_reported_heading = None
        self._last_queried_heading = None
        # True if the most recent DONE reported a stall-escalation branch
        # fired during that command (bump/floor obstruction) — reset to
        # False on every DONE, including bare ones (e.g. from CMD:STOP), so
        # a stale True never leaks into an unrelated later command's result.
        self.last_turn_stalled = False

        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="ESP-reader")
        self._reader.start()
        self._try_connect()

    def send_turn(self, angle_deg):
        cmd       = f"CMD:TURN {angle_deg:.1f}"
        direction = "left" if angle_deg < 0 else "right"
        print(f"  -> Turn {direction}  {abs(angle_deg):.1f}deg")
        self._send_and_wait(cmd)

    def send_turn_interruptible(self, angle_deg, interrupt_event):
        """Send a turn that can be interrupted mid-execution by interrupt_event."""
        cmd       = f"CMD:TURN {angle_deg:.1f}"
        direction = "left" if angle_deg < 0 else "right"
        print(f"  -> Scan {direction}  {abs(angle_deg):.1f}deg")
        return self._send_and_wait(cmd,
                                   interrupt_event=interrupt_event,
                                   stop_on_interrupt=True)

    def send_turn_compass(self, angle_deg):
        """Compass-verified (closed-loop) turn.

        Used ONLY for scan / search-pattern turns (30deg split-scan steps,
        pose turns, undo-turns, and post-collection heading restore). The
        ESP32 uses the IST8310 heading over I2C to stop at the true target
        heading instead of a fixed timed duration. navigate()/stabilise()
        turns during target approach intentionally do NOT use this — they
        keep the original timed CMD:TURN behaviour.
        """
        cmd       = f"CMD:CTURN {angle_deg:.1f}"
        direction = "left" if angle_deg < 0 else "right"
        print(f"  -> CTurn {direction}  {abs(angle_deg):.1f}deg (compass)")
        self._send_and_wait(cmd)

    def send_turn_compass_interruptible(self, angle_deg, interrupt_event, tol_deg=None):
        """Compass-verified turn that can be interrupted mid-execution.

        tol_deg, if given, is passed to the ESP as a widened accept
        tolerance (CMD:CTURN <deg> <tol>). Above the ESP's default
        COMPASS_TURN_TOL_DEG this skips its reach-verify coast-settle,
        so intermediate stops don't need to nail an exact heading."""
        cmd = (f"CMD:CTURN {angle_deg:.1f} {tol_deg:.1f}"
               if tol_deg is not None else f"CMD:CTURN {angle_deg:.1f}")
        direction = "left" if angle_deg < 0 else "right"
        print(f"  -> CScan {direction}  {abs(angle_deg):.1f}deg (compass)")
        return self._send_and_wait(cmd,
                                   interrupt_event=interrupt_event,
                                   stop_on_interrupt=True)

    def send_move(self, dist_m):
        cmd = f"CMD:MOVE {dist_m:.2f}"
        print(f"  -> Move  {dist_m:.2f} m")
        self._send_and_wait(cmd)

    def send_cmove(self, dist_m):
        """Compass-verified move: before driving forward the ESP checks the
        current heading against the last CMD:SETCENTRE reference and issues
        an internal CTURN correction if drift exceeds CENTRE_CORRECT_TOL_DEG
        (5 deg). All correction logic runs on the ESP side — no extra
        GETHEADING round-trips are needed from the Jetson."""
        cmd = f"CMD:CMOVE {dist_m:.2f}"
        print(f"  -> CMove {dist_m:.2f} m (compass-corrected)")
        self._send_and_wait(cmd)

    def send_set_centre(self):
        """Tell the ESP to read the compass now (median-of-3, glitch-safe)
        and store the result as the reference heading for CMD:CMOVE
        corrections. Should be called at the start of each patrol lane and
        again after every lane shift so the reference tracks the actual lane
        direction. Returns the stored heading as a float, or None on failure."""
        with self._cmd_lock:
            self._heading_ev.clear()
            if not self._connected:
                return None
            print("  -> SetCentre (reading compass median-of-3 on ESP)")
            self._send_line("CMD:SETCENTRE")
            if self._heading_ev.wait(timeout=2.0):
                h = self._last_queried_heading
                if h is not None:
                    print(f"  -> Centre heading stored: {h:.1f}deg")
                else:
                    print("  -> WARNING: CMD:SETCENTRE compass read failed on ESP")
                return h
            print("[ESP] WARNING: timeout waiting HEADING for CMD:SETCENTRE "
                  "— forcing reconnect")
            self._close_socket()
            return None

    def send_backward(self, dist_m):
        """Reverse by dist_m metres.
        Requires ESP firmware to handle CMD:MOVE with a negative value.
        e.g. add 'if (dist < 0) { move_backward(-dist); }' to the ESP handler."""
        cmd = f"CMD:MOVE {-abs(dist_m):.2f}"
        print(f"  -> Back  {abs(dist_m):.2f} m")
        self._send_and_wait(cmd)

    def send_stop(self):
        self._send_line("CMD:STOP")

    def get_heading(self, timeout=2.0):
        """Query the ESP's current compass heading right now, independent of
        any turn/move — used to snapshot the "center" heading before a
        scan-dance starts. Returns float degrees, or None on timeout/failure
        (e.g. compass disabled) — callers should skip verification in that case
        rather than treat None as 0.0."""
        with self._cmd_lock:
            self._heading_ev.clear()
            if not self._connected:
                return None
            self._send_line("CMD:GETHEADING")
            if self._heading_ev.wait(timeout=timeout):
                return self._last_queried_heading
            print("[ESP] WARNING: timeout waiting HEADING for CMD:GETHEADING "
                  "— forcing reconnect (silence could mean a frozen/crashed ESP, "
                  "not just a slow reply)")
            self._close_socket()
            return None

    def stop(self):
        self._stop.set()
        self._close_socket()

    @property
    def connected(self):
        return self._connected

    def _try_connect(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((self.host, self.port))
            # A bare settimeout(None) here means recv() blocks FOREVER with no
            # exception if the ESP silently hangs/reboots without a clean TCP
            # close (very plausible after sustained high motor current) — the
            # reader thread would get stuck for the rest of the session with
            # no reconnect ever triggering. Use a bounded recv timeout plus
            # OS-level TCP keepalive so a dead peer gets detected and errors
            # out recv() within a bounded time instead of hanging silently.
            s.settimeout(8.0)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 3)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 2)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)
            except (AttributeError, OSError):
                pass  # TCP_KEEPIDLE/INTVL/CNT are Linux-only; harmless if absent
            with self._sock_lock:
                self._sock = s
            self._connected = True
            self._connect_attempts = 0
            print(f"[ESP] Connected  {self.host}:{self.port}")
        except Exception:
            self._connected = False
            self._connect_attempts = getattr(self, '_connect_attempts', 0) + 1
            if self._connect_attempts == 1:
                print(f"[ESP] Not reachable — retrying every "
                      f"{ESP_RECONNECT_INTERVAL:.0f}s (WiFi connected?)")

    def _close_socket(self):
        with self._sock_lock:
            if self._sock:
                try: self._sock.close()
                except: pass
                self._sock = None
        self._connected = False

    def _send_and_wait(self, cmd, timeout=DONE_TIMEOUT_S,
                       interrupt_event=None, stop_on_interrupt=False):
        with self._cmd_lock:
            self._done_ev.clear()
            if not self._connected:
                print(f"[ESP] Not connected — dry-run: {cmd}")
                self._done_ev.set()
                return True

            self._send_line(cmd)
            deadline   = time.time() + timeout
            stop_sent  = False

            while time.time() < deadline:
                if self._done_ev.wait(timeout=0.03):
                    if stop_on_interrupt and interrupt_event is not None:
                        return not interrupt_event.is_set()
                    return True

                if not self._connected:
                    # Reader thread already detected a dead connection
                    # (keepalive/recv-timeout error) and is reconnecting —
                    # no point waiting out the rest of the timeout.
                    print(f"[ESP] WARNING: connection dropped mid-wait for '{cmd}'")
                    return False

                if (stop_on_interrupt and interrupt_event is not None
                        and interrupt_event.is_set() and not stop_sent):
                    print(f"[ESP] Interrupting '{cmd}' with STOP")
                    self._send_line("CMD:STOP")
                    stop_sent = True

            print(f"[ESP] WARNING: timeout waiting DONE after '{cmd}' — forcing "
                  f"reconnect (full {timeout:.0f}s of silence with no error at all "
                  f"points to a hung/crashed ESP, not just a slow reply)")
            self._close_socket()
            return False

    def _send_line(self, cmd):
        line = (cmd + "\n").encode()
        with self._sock_lock:
            sock = self._sock
        if sock:
            try:
                sock.sendall(line)
                if self.verbose:
                    print(f"[ESP->] {cmd}")
            except Exception as e:
                print(f"[ESP] Send error: {e}")
                self._close_socket()
        else:
            print(f"[DRY-RUN] {cmd}")
            self._done_ev.set()

    def _read_loop(self):
        buf = ""
        while not self._stop.is_set():
            with self._sock_lock:
                sock = self._sock

            if sock is None:
                time.sleep(ESP_RECONNECT_INTERVAL)
                self._try_connect()
                buf = ""
                continue

            try:
                chunk = sock.recv(256).decode(errors='ignore')
                if not chunk:
                    print("[ESP] Connection closed by peer — reconnecting...")
                    self._close_socket()
                    buf = ""
                    continue
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    resp = line.strip()
                    if resp:
                        if self.verbose:
                            print(f"[ESP<-] {resp}")
                        if resp == "DONE":
                            self.last_turn_stalled = False
                            self._done_ev.set()
                        elif resp.startswith("DONE"):
                            # "DONE HEADING=xx.x" or "DONE HEADING=xx.x STALLED=1"
                            # — ordinary completion ack with the ESP's last-known
                            # heading riding along, and an optional stall flag
                            # if a bump-recovery branch fired during this turn.
                            self.last_turn_stalled = "STALLED=1" in resp
                            try:
                                val = resp.split("HEADING=", 1)[1]
                                self.last_reported_heading = float(val.split()[0])
                            except (IndexError, ValueError):
                                pass
                            self._done_ev.set()
                        elif resp.startswith("HEADING:"):
                            # Reply to an explicit CMD:GETHEADING query.
                            val = resp.split("HEADING:", 1)[1]
                            try:
                                self._last_queried_heading = float(val)
                            except ValueError:
                                self._last_queried_heading = None   # "ERR"
                            self._heading_ev.set()
                        elif resp == "HELLO":
                            print("[ESP] Handshake OK")
            except socket.timeout:
                # Expected periodically now that the socket has a bounded
                # recv timeout (see _try_connect) — just means no data
                # arrived in that window, not that the connection is dead.
                # TCP keepalive will make a genuinely dead peer raise a
                # real error here instead of just timing out silently.
                continue
            except Exception as e:
                print(f"[ESP] Read error: {e} — reconnecting...")
                self._close_socket()
                buf = ""


# ----------------------------------------------------------------------------
#  TARGET TRACKER
# ----------------------------------------------------------------------------

class TargetTracker:
    def __init__(self):
        self.targets   = {}
        self.collected = []
        self._lock     = threading.Lock()

    def update(self, detections):
        now = time.time()
        with self._lock:
            for d in detections:
                tid = d['id']
                if tid in self.collected:
                    continue
                self.targets[tid] = {
                    'dist':      d['dist'],
                    'angle':     d['angle'],
                    'conf':      d['conf'],
                    'last_seen': now,
                }
            for tid in list(self.targets):
                if tid in self.collected:
                    del self.targets[tid]
                elif now - self.targets[tid]['last_seen'] > TARGET_TIMEOUT_S:
                    del self.targets[tid]
            return sorted(self.targets.items(), key=lambda x: x[1]['dist'])

    def mark_collected(self, tid):
        with self._lock:
            self.collected.append(tid)
            if tid in self.targets:
                del self.targets[tid]
        print(f"  -> Collected ID={tid}  total={len(self.collected)}")


# ----------------------------------------------------------------------------
#  RTSP READER
# ----------------------------------------------------------------------------

class RTSPReader:
    def __init__(self, url, name, map1=None, map2=None, target_shape=None):
        self.name          = name
        self._url          = url
        self._stop         = threading.Event()
        self.map1          = map1
        self.map2          = map2
        self.target_shape  = target_shape
        self._latest_frame = None
        self._frame_id     = 0
        self._lock         = threading.Lock()
        self._t = threading.Thread(target=self._run, daemon=True,
                                   name=f"RTSP-{name}")

    def start(self):
        cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        if not cap.isOpened():
            print(f"[{self.name}] ERROR: cannot open {self._url}")
            return False
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap
        self._t.start()
        print(f"[{self.name}] Connected.")
        return True

    def read(self):
        with self._lock:
            if self._latest_frame is not None:
                return True, self._latest_frame, self._frame_id
            return False, None, -1

    def read_blocking(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            with self._lock:
                if self._latest_frame is not None:
                    return True, self._latest_frame
            time.sleep(0.05)
        return False, None

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)
        if hasattr(self, '_cap'):
            self._cap.release()

    def _run(self):
        while not self._stop.is_set():
            ok, frame = self._cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            if (self.target_shape is not None and
                    frame.shape[:2] != (self.target_shape[1], self.target_shape[0])):
                frame = cv2.resize(frame, self.target_shape)
            if self.map1 is not None and self.map2 is not None:
                frame = cv2.remap(frame, self.map1, self.map2, cv2.INTER_LINEAR)
            with self._lock:
                self._latest_frame = frame
                self._frame_id    += 1


# ----------------------------------------------------------------------------
#  DISPARITY WORKER
# ----------------------------------------------------------------------------

class DisparityWorker:
    def __init__(self, matchers, Q):
        self._matchers = matchers
        self._Q        = Q
        self._in_q     = queue.Queue(maxsize=1)
        self._result   = (None, None)
        self._lock     = threading.Lock()
        self._stop     = threading.Event()
        self._t        = threading.Thread(target=self._run, daemon=True,
                                          name="DisparityWorker")
        self._t.start()

    def submit(self, gl_full, gr_full):
        if self._in_q.full():
            try: self._in_q.get_nowait()
            except queue.Empty: pass
        self._in_q.put((gl_full.copy(), gr_full.copy()))

    def get(self):
        with self._lock:
            return self._result

    def stop(self):
        self._stop.set()
        self._t.join(timeout=3.0)

    def _run(self):
        while not self._stop.is_set():
            try:
                gl_full, gr_full = self._in_q.get(timeout=0.1)
            except queue.Empty:
                continue

            H, W = gl_full.shape
            half_W, half_H = W // 2, H // 2

            gl_small = cv2.resize(gl_full, (half_W, half_H),
                                  interpolation=cv2.INTER_LINEAR)
            gr_small = cv2.resize(gr_full, (half_W, half_H),
                                  interpolation=cv2.INTER_LINEAR)

            lm, rm, wls = self._matchers
            dL_small = lm.compute(gl_small, gr_small)

            if wls is not None and rm is not None:
                dR_small       = rm.compute(gr_small, gl_small)
                disp_small_f32 = wls.filter(
                    dL_small, gl_small,
                    disparity_map_right=dR_small
                ).astype(np.float32) / 16.0
            else:
                disp_small_f32 = dL_small.astype(np.float32) / 16.0

            scale    = W / half_W
            disp_f32 = cv2.resize(disp_small_f32, (W, H),
                                  interpolation=cv2.INTER_LINEAR) * scale

            depth = _disp_to_depth(disp_f32, self._Q)

            with self._lock:
                self._result = (disp_f32, depth)


# ----------------------------------------------------------------------------
#  CALIBRATION
# ----------------------------------------------------------------------------

def load_calibration(path):
    if not os.path.exists(path):
        print(f"ERROR: {path} not found"); sys.exit(1)
    raw = np.load(path)

    def get(a, b=None):
        if a in raw: return raw[a]
        if b and b in raw: return raw[b]
        raise KeyError(a)

    K0 = get('K0', 'mtx_left');  d0 = get('d0', 'dist_left')
    K1 = get('K1', 'mtx_right'); d1 = get('d1', 'dist_right')
    R1 = get('R1'); R2 = get('R2')
    P1 = get('P1'); P2 = get('P2')
    Q  = get('Q')
    W, H = tuple(int(x) for x in get('img_shape'))

    map1L, map2L = cv2.initUndistortRectifyMap(K0, d0, R1, P1, (W, H), cv2.CV_32F)
    map1R, map2R = cv2.initUndistortRectifyMap(K1, d1, R2, P2, (W, H), cv2.CV_32F)

    baseline = abs(float(P2[0, 3])) / float(P1[0, 0])
    f_rect   = float(P1[0, 0])
    cx0      = float(P1[0, 2])   # true rectified principal-point x

    print(f"\n-- Calibration: {W}x{H}  fx={f_rect:.1f}  baseline={baseline*100:.1f}cm  cx0={cx0:.1f}px")
    print(f"   Disp@1m={f_rect*baseline:.1f}px  Disp@3m={f_rect*baseline/3:.1f}px\n")

    return dict(W=W, H=H, Q=Q,
                map1L=map1L, map2L=map2L,
                map1R=map1R, map2R=map2R,
                baseline=baseline, f_rect=f_rect, fx=f_rect, cx0=cx0)


# ----------------------------------------------------------------------------
#  STEREO MATCHER
# ----------------------------------------------------------------------------

def build_matcher():
    lm = cv2.StereoSGBM_create(
        minDisparity=MIN_DISP, numDisparities=NUM_DISP, blockSize=BLOCK_SIZE,
        P1=8*3*BLOCK_SIZE**2, P2=32*3*BLOCK_SIZE**2,
        disp12MaxDiff=2, uniquenessRatio=8,
        speckleWindowSize=150, speckleRange=2,
        preFilterCap=31, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    if USE_WLS and HAS_WLS:
        rm  = ximgproc.createRightMatcher(lm)
        wls = ximgproc.createDisparityWLSFilter(lm)
        wls.setLambda(WLS_LAMBDA)
        wls.setSigmaColor(WLS_SIGMA)
        print("Disparity: SGBM + WLS")
        return lm, rm, wls
    print("Disparity: SGBM only")
    return lm, None, None


# ----------------------------------------------------------------------------
#  DEPTH
# ----------------------------------------------------------------------------

def _disp_to_depth(disp, Q):
    pts   = cv2.reprojectImageTo3D(disp, Q)
    depth = pts[:, :, 2].copy()
    depth[disp <= 1.0]         = 0.0
    depth[~np.isfinite(depth)] = 0.0
    depth[depth <= 0]          = 0.0
    depth[depth > 50.0]        = 0.0
    return depth


def bbox_dist(depth, x1, y1, x2, y2):
    H, W = depth.shape
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(W, int(x2)), min(H, int(y2))
    if x2 <= x1 or y2 <= y1:
        return None, 0.0
    bh   = y2 - y1
    y1c  = y1 + int(bh * 0.20)
    y2c  = y1 + int(bh * 0.80)
    roi  = depth[y1c:y2c, x1:x2]
    valid = roi[(roi > 0.2) & (roi < 40.0)]
    cov  = 100.0 * valid.size / max(roi.size, 1)
    if valid.size < 8:
        return None, cov
    sv = np.sort(valid.ravel())
    n  = max(1, int(len(sv) * 0.30))
    return float(np.median(sv[:n])), cov


# ----------------------------------------------------------------------------
#  DRAWING
# ----------------------------------------------------------------------------

def dist_color(dist_m):
    ratio = min(dist_m / MAX_DIST_M, 1.0)
    return (0, int(255 * ratio), int(255 * (1.0 - ratio)))


def draw_detections(frame, results, depth, cam_label, is_main_view=True,
                    track_history=None, roi_poly=None,
                    active_id=None, sorted_targets=None, fx=395.34):
    out   = frame.copy()
    H_f   = out.shape[0]
    W_f   = out.shape[1]
    count = 0

    if roi_poly is not None:
        cv2.polylines(out, [roi_poly], True, (0, 255, 255), 2)

    if not is_main_view:
        cv2.putText(out, f"{cam_label}  |  Inference on LEFT only",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 220, 255), 2)
        return out

    if results is None:
        cv2.putText(out, f"{cam_label}  |  0 det",
                    (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 220, 255), 2)
        return out

    for box in results.boxes:
        cls      = int(box.cls[0])
        conf_val = float(box.conf[0])
        track_id = int(box.id[0]) if box.id is not None else None

        if cls != BOTTLE_CLASS:
            continue

        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

        if roi_poly is not None:
            px, py = (x1 + x2) / 2, y2
            if cv2.pointPolygonTest(roi_poly, (px, py), False) < 0:
                continue

        count += 1
        class_label = COCO_NAMES[cls] if 0 <= cls < len(COCO_NAMES) else f"cls{cls}"

        if depth is not None:
            dist, cov = bbox_dist(depth, x1, y1, x2, y2)
            if dist is not None and track_id is not None and track_history is not None:
                if track_id not in track_history:
                    track_history[track_id] = []
                track_history[track_id].append(dist)
                if len(track_history[track_id]) > 8:
                    track_history[track_id].pop(0)
                smoothed = float(np.median(track_history[track_id]))
            else:
                smoothed = dist

            is_active = (track_id == active_id)
            if smoothed is not None:
                color = (0, 255, 255) if is_active else dist_color(smoothed)
                thick = 3             if is_active else 2
                cx_box = (x1 + x2) / 2.0
                ang    = compute_angle(cx_box, W_f / 2.0, fx)  # display only — approx cx0
                if is_active:
                    d   = "L" if ang < 0 else "R"
                    tag = (f">>>{class_label} {conf_val:.0%}  "
                           f"{smoothed:.2f}m  {abs(ang):.1f}d{d}")
                else:
                    tag = f"{class_label} {conf_val:.0%}  {smoothed:.2f}m"
            else:
                color = (80, 80, 80); thick = 2
                tag   = f"{class_label} {conf_val:.0%}  N/A ({cov:.0f}%)"
        else:
            color = (0, 165, 255); thick = 2
            tag   = f"{class_label} {conf_val:.0%} (no depth)"

        cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), color, thick)
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
        ty = max(int(y1) - 5, th + 5)
        cv2.rectangle(out, (int(x1), ty - th - 3),
                      (int(x1) + tw + 4, ty + 2), color, -1)
        cv2.putText(out, tag, (int(x1) + 2, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 1, cv2.LINE_AA)

    if sorted_targets:
        for rank, (tid, t) in enumerate(sorted_targets[:5]):
            marker = ">>>" if tid == active_id else f"#{rank+1}"
            col    = (0, 255, 255) if tid == active_id else (180, 180, 180)
            line   = f"{marker} ID={tid} {t['dist']:.2f}m {t['angle']:+.1f}d"
            cv2.putText(out, line, (8, H_f - 60 - rank * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.40, col, 1)

    cv2.putText(out, f"{cam_label}  |  {count} det",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 220, 255), 2)
    return out


def colorize_disp(disp):
    valid = disp[disp > 0]
    lo = float(np.percentile(valid, 2))  if valid.size else 0.0
    hi = float(np.percentile(valid, 98)) if valid.size else 1.0
    d  = np.clip((disp - lo) / max(hi - lo, 1e-6), 0, 1)
    return cv2.applyColorMap((d * 255).astype(np.uint8), cv2.COLORMAP_TURBO)


# ----------------------------------------------------------------------------
#  ROI
# ----------------------------------------------------------------------------

def save_roi_for_side(polygon_norm, side="left", filename=ROI_FILE):
    data = {"left_roi": None, "right_roi": None}
    if os.path.exists(filename):
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
        except Exception:
            pass
    data[f"{side}_roi"] = polygon_norm
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"[ROI] Saved {side}")


def load_roi_for_side(side="left", filename=ROI_FILE):
    if not os.path.exists(filename):
        return None
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        poly = data.get(f"{side}_roi")
        if poly:
            print(f"[ROI] Loaded {side}")
            return poly
    except Exception:
        pass
    return None


def delete_roi(filename=ROI_FILE):
    if os.path.exists(filename):
        os.remove(filename)
        print("[ROI] Deleted")


class ROIDrawer:
    def __init__(self, fw, fh, total_width=None):
        self.frame_W      = fw
        self.H            = fh
        self.total_W      = total_width or fw
        self.points       = []
        self.drawing      = False
        self.polygon_norm = None
        self.scale_factor = 1.0
        self.current_side = None

    def set_scale_factor(self, s):
        self.scale_factor = s

    def mouse_callback(self, event, x, y, flags, param):
        xs = int(x / self.scale_factor)
        ys = int(y / self.scale_factor)
        xr = xs - self.frame_W if self.current_side == 'right' else xs
        xr = max(0, min(self.frame_W - 1, xr))
        yr = max(0, min(self.H - 1, ys))

        if event == cv2.EVENT_LBUTTONDOWN:
            self.points.append((xr, yr))
            print(f"[ROI] ({xr},{yr}) count:{len(self.points)}")
        elif event == cv2.EVENT_LBUTTONDBLCLK:
            if len(self.points) >= 3:
                self.polygon_norm = [
                    (px / self.frame_W, py / self.H)
                    for px, py in self.points
                ]
                self.drawing = False
                print("[ROI] Polygon complete")
            else:
                print("[ROI] Need >=3 points")

    def reset(self):
        self.points       = []
        self.drawing      = True
        self.polygon_norm = None

    def draw_on_frame(self, frame):
        out = frame.copy()
        for i, pt in enumerate(self.points):
            cv2.circle(out, pt, 6, (0, 255, 0), -1)
            cv2.putText(out, str(i + 1), (pt[0] + 10, pt[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        if len(self.points) > 1:
            cv2.polylines(out,
                          [np.array(self.points, np.int32)],
                          False, (0, 255, 0), 2)
        return out


# ----------------------------------------------------------------------------
#  MAIN
# ----------------------------------------------------------------------------

def main():
    # -- Logging setup (must be first so all prints go to file) ------------
    _log_fname = datetime.now().strftime("log_%d-%m-%y_%H-%M-%S.txt")
    _tee = TeeLogger(_log_fname)
    sys.stdout = _tee

    print("=" * 62)
    print("  Stereo Autonomous Waste Collection  [COMBINED]")
    print("=" * 62)
    print(f"[Logger] Logging session to: {_log_fname}")

    ensure_wifi()

    calib    = load_calibration(CALIB_FILE)
    W, H     = calib['W'], calib['H']
    matchers = build_matcher()

    disp_worker = DisparityWorker(matchers, calib['Q'])
    print("Disparity worker started.")

    print(f"Loading YOLO: {YOLO_MODEL} ...")
    model = YOLO(YOLO_MODEL, task='detect')
    print("YOLO ready.")

    esp     = ESPCommander(host=ESP_HOST, port=ESP_PORT)
    tracker = TargetTracker()

    rl = RTSPReader(RTSP_LEFT,  "Left",
                    calib['map1L'], calib['map2L'], (W, H))
    rr = RTSPReader(RTSP_RIGHT, "Right",
                    calib['map1R'], calib['map2R'], (W, H))

    if not rl.start() or not rr.start():
        rl.stop(); rr.stop(); disp_worker.stop(); sys.exit(1)

    time.sleep(1.5)

    print("Waiting for first frames ...")
    for _ in range(20):
        ok_l, _ = rl.read_blocking(timeout=0.5)
        ok_r, _ = rr.read_blocking(timeout=0.5)
        if ok_l and ok_r:
            break
    else:
        print("ERROR: timed out waiting for camera frames.")
        rl.stop(); rr.stop(); disp_worker.stop(); sys.exit(1)

    print("Cameras live — starting main loop.")

    # -- Display / UI state ------------------------------------------------
    show_disp  = SHOW_DISPARITY
    show_epi   = EPIPOLAR_LINES
    debug      = False
    conf       = CONF_THRESH
    fps = 0.0; t_fps = time.time(); fc = 0
    start_time = time.time()

    if USE_ROI:
        roi_norm_left  = load_roi_for_side("left")  or ROI_NORM_POLYGON
        roi_norm_right = load_roi_for_side("right") or ROI_NORM_POLYGON
    else:
        roi_norm_left = roi_norm_right = None

    def make_poly(norm):
        if norm is None: return None
        return np.array([[int(x * W), int(y * H)] for x, y in norm], np.int32)

    roi_poly_left  = make_poly(roi_norm_left)
    roi_poly_right = make_poly(roi_norm_right)

    roi_drawer    = ROIDrawer(W, H, total_width=2 * W)
    roi_draw_mode = False
    roi_draw_side = None
    last_r = last_c = last_s = False
    poly_done = False

    track_history_l = {}

    # -- Navigation state --------------------------------------------------
    # Use a lock for nav_busy / active_id — written from background thread,
    # read from main thread (pattern from stereo_autonomous.py)
    _nav_lock = threading.Lock()
    active_id = None
    nav_busy            = False
    scan_busy           = False
    search_pattern_busy = False
    _last_ignore_print_time = 0.0   # throttle for "target too far — ignoring" log
    search_paused_for_nav = False
    _nav_total_turn_deg = 0.0
    sorted_cache = []
    # Pre-initialized so the "if sorted_targets: ... if _nav_busy_snapshot:"
    # check earlier in the loop body has a value to read on the very FIRST
    # iteration — that check runs before the "with _nav_lock: _nav_busy_snapshot
    # = nav_busy" assignment further down in the same iteration, so without
    # this a bottle already in frame at startup causes an UnboundLocalError.
    _nav_busy_snapshot     = False
    _active_id_for_filter  = None

    nav_interrupt_event = threading.Event()
    last_raw_seen_time       = 0.0
    last_detection_debug_print = 0.0
    last_left_frame_id       = -1

    # per-track depth cache: tid -> {'ts': float, 'dist': float, 'samples': deque}
    _dist_cache: dict = {}
    # consecutive frames of no detection during active navigation
    nav_detection_loss_count = 0
    # True only when the current navigation started by interrupting scan.
    # The detection-loss stop rule is limited to this case.
    _nav_started_from_scan = False
    # True while navigate() is inside a planned post-turn/post-move settle sleep
    # — suppresses the watchdog to prevent redundant STOP commands
    _nav_in_planned_settle = False

    # depth-fail hold: timestamp of first frame where object was in ROI but
    # depth failed; reset when we get a valid detection or nothing is in ROI.
    depth_fail_since = 0.0

    # -- Scan state --------------------------------------------------------
    scan_total_deg         = 0.0
    scan_active            = False
    scan_direction         = 1
    _scan_resume_pending   = False
    _post_scan_settle_until = 0.0   # non-blocking settle after scan abort
    _post_scan_restart_ok_after = 0.0  # scan cannot restart before this time
    _post_scan_best_target  = None   # last valid target seen; used to dispatch nav
                                     # at settle expiry even if depth cache expired
    _last_nav_target_angle  = None   # angle of the most recently dispatched nav target;
                                     # survives _post_scan_best_target clears so scan
                                     # direction is correct even after repeated nav failures
    _anon_target            = None   # last depth seen on an id_fail frame
                                     # — used by stabilise() when tracker has no
                                     #   fresh entry (e.g. post-turn ID reassignment)

    win_name = "Stereo Autonomous  [q d e i +/- S R c]"
    print(f"\nControls: q=quit  d=disp  e=epi  i=debug  +/-=conf")
    print(f"          S=left ROI  R=right ROI  c=clear ROI")
    print(f"ESP WiFi: {ESP_HOST}:{ESP_PORT}")
    print(f"Initial stabilise: {INITIAL_STABILIZE_S:.1f}s  "
          f"Pre-move stabilise: {PRE_MOVE_STABILIZE_S:.1f}s")
    print(f"Scan step: {SCAN_STEP_DEG:.1f}deg | Target timeout: {TARGET_TIMEOUT_S:.1f}s")
    print("-" * 62)

    # -- Main loop ---------------------------------------------------------
    while True:
        ok_l, rect_l, left_frame_id = rl.read()
        ok_r, rect_r, _             = rr.read()

        if not ok_l or not ok_r or rect_l is None or rect_r is None:
            time.sleep(0.005)
            continue

        # Skip duplicate frames (from stereo_autonomous.py)
        if left_frame_id == last_left_frame_id:
            time.sleep(0.001)
            continue
        last_left_frame_id = left_frame_id

        gl = cv2.cvtColor(rect_l, cv2.COLOR_BGR2GRAY)
        gr = cv2.cvtColor(rect_r, cv2.COLOR_BGR2GRAY)
        disp_worker.submit(gl, gr)

        disp, dep = disp_worker.get()

        if debug and dep is not None:
            vd = disp[disp > 0]; vz = dep[dep > 0]
            if vd.size: print(f"[disp] med={np.median(vd):.1f}px")
            if vz.size: print(f"[dep]  med={np.median(vz):.2f}m")

        raw = model.track(rect_l, conf=conf, classes=[BOTTLE_CLASS],
                          persist=True, verbose=False)
        result_l = raw[0] if isinstance(raw, list) else raw

        # -- Extract detections --------------------------------------------
        now_loop = time.time()   # moved here so depth cache can use it
        detections = []
        raw_det_count   = 0
        raw_seen_in_roi = False
        roi_fail    = 0
        id_fail     = 0
        depth_fail  = 0
        valid_count = 0
        # _anon_target is NOT reset here — it persists across frames so that
        # a valid depth captured on an id_fail=1 frame can still be used on
        # the very next frame when YOLO assigns a fresh tracking ID.

        if result_l.boxes is not None:
            for box in result_l.boxes:
                cls_val  = int(box.cls[0])
                conf_val = float(box.conf[0])

                if cls_val != BOTTLE_CLASS: continue
                if conf_val < conf:         continue

                raw_det_count += 1

                # ── ROI check FIRST so raw_seen_in_roi is correct even
                #    when the track ID hasn't been assigned yet (id_fail).
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                if roi_poly_left is not None:
                    px, py = (x1 + x2) / 2, y2
                    if cv2.pointPolygonTest(roi_poly_left, (px, py), False) < 0:
                        roi_fail += 1; continue

                # Object is inside ROI — record visibility regardless of ID/depth
                raw_seen_in_roi = True

                tid = int(box.id[0]) if box.id is not None else None
                if tid is None:
                    id_fail += 1
                    # Depth may still be valid even without a track ID (common
                    # on the first 1-2 frames after a new detection or after a
                    # turn when YOLO re-assigns IDs).  Compute it and save so
                    # stabilise() and nav dispatch can use it as a fallback.
                    if dep is not None:
                        _d, _cov = bbox_dist(dep, x1, y1, x2, y2)
                        if _d is not None and _cov >= DIST_MIN_COVERAGE:
                            _cx  = (x1 + x2) / 2.0
                            _ang = compute_angle(_cx, calib['cx0'], calib['fx'])
                            _anon_target = {
                                'dist': _d, 'angle': _ang, 'ts': now_loop,
                            }
                    continue

                if dep is None:
                    depth_fail += 1; continue

                dist, cov = bbox_dist(dep, x1, y1, x2, y2)

                # ---- Depth reliability: cache, outlier rejection, continuity ----
                if tid not in _dist_cache:
                    _dist_cache[tid] = {
                        'ts': 0.0, 'dist': 0.0,
                        'samples': deque(maxlen=DIST_CACHE_HISTORY),
                    }
                _cache = _dist_cache[tid]

                if dist is not None and cov >= DIST_MIN_COVERAGE:
                    # Outlier rejection: only reject UPWARD jumps (sudden far reading = noise).
                    # Do NOT reject downward readings — the bot may be approaching the target
                    # and the distance naturally decreases.  Rejecting closer readings froze
                    # the history at the initial distance and caused a 30+ frame log spam.
                    if len(_cache['samples']) >= 3:
                        hist_med = float(np.median(list(_cache['samples'])))
                        if dist > hist_med * DIST_OUTLIER_RATIO:
                            print(f"  [depth] ID={tid} outlier {dist:.2f}m "
                                  f"vs hist_med {hist_med:.2f}m — using history")
                            dist = hist_med
                    _cache['ts']   = now_loop
                    _cache['dist'] = dist
                    _cache['samples'].append(dist)
                else:
                    # Depth failed or coverage too low — try recent cache
                    age = now_loop - _cache['ts']
                    if _cache['dist'] > 0 and age < DIST_CACHE_GRACE_S:
                        print(f"  [depth] ID={tid} depth N/A (cov={cov:.0f}%) "
                              f"— using cached {_cache['dist']:.2f}m  age={age:.2f}s")
                        dist = _cache['dist']
                    else:
                        # Cache empty for this ID — seed from a very recent
                        # anonymous depth (id_fail frame that immediately
                        # preceded this ID assignment after a turn).
                        if (_anon_target is not None
                                and _cache['dist'] == 0
                                and (now_loop - _anon_target['ts']) < 0.5):
                            dist = _anon_target['dist']
                            _cache['ts']   = now_loop
                            _cache['dist'] = dist
                            _cache['samples'].append(dist)
                            # No print — this is a silent seed from adjacent frame
                        else:
                            depth_fail += 1
                            continue
                # ---- end depth reliability block ----

                cx  = (x1 + x2) / 2.0
                ang = compute_angle(cx, calib['cx0'], calib['fx'])
                detections.append({
                    'id': tid, 'dist': dist,
                    'angle': ang, 'conf': conf_val, 'cov': cov,
                })
                valid_count += 1

        sorted_targets = tracker.update(detections)
        sorted_cache   = sorted_targets

        # -- Interrupt / scan-pause logic ----------------------------------
        # (now_loop already set above)

        # Only ever dispatch navigate() / hold scan for a target within collect
        # range (COLLECT_MAX_DIST_M). Compute _within_collect_range FIRST so we
        # can gate last_raw_seen_time — an out-of-range target with valid depth
        # must NOT freeze the scan; the bot needs to keep looking for targets
        # that are actually close enough to collect.
        if sorted_targets:
            _closest_dist = min(t_data['dist'] for _, t_data in sorted_targets)
            _within_collect_range = _closest_dist <= COLLECT_MAX_DIST_M
            if not _within_collect_range and now_loop - _last_ignore_print_time > 1.0:
                _ctx = "scan-dance" if search_pattern_busy else "idle"
                print(f"  [{_ctx}] target at {_closest_dist:.2f}m > "
                      f"{COLLECT_MAX_DIST_M:.2f}m collect range — ignoring")
                _last_ignore_print_time = now_loop
        else:
            # No valid-depth target yet — raw_hold_active is allowed to
            # hold briefly so depth can resolve; the check above will re-run
            # once depth appears and can still reject it.
            _within_collect_range = True

        if sorted_targets:
            depth_fail_since = 0.0
            # FIX: only update last_raw_seen_time for in-range targets.
            # If the target is outside COLLECT_MAX_DIST_M it has valid depth
            # but we are ignoring it — do NOT let it hold raw_hold_active True
            # and freeze the scan indefinitely.
            if _within_collect_range:
                last_raw_seen_time = now_loop
            # Keep a rolling record of the best target with valid depth.
            # During active navigation the bot is already heading for the active
            # target, so track the NEXT best target (closest non-active,
            # non-collected) so post-collection dispatch finds the right bottle.
            # When idle, simply track sorted_targets[0] as before.
            if _nav_busy_snapshot:
                _next_candidates = [
                    (t_id, t_data) for t_id, t_data in sorted_targets
                    if t_id != _active_id_for_filter
                    and t_id not in tracker.collected
                ]
                if _next_candidates:
                    _nid, _nd = _next_candidates[0]
                    _post_scan_best_target = {
                        'tid': _nid, 'dist': _nd['dist'],
                        'angle': _nd['angle'], 'ts': now_loop,
                    }
                # else: no secondary target visible — leave _post_scan_best_target unchanged
            else:
                _tid0, _t0 = sorted_targets[0]
                _post_scan_best_target = {
                    'tid': _tid0, 'dist': _t0['dist'],
                    'angle': _t0['angle'], 'ts': now_loop,
                }
        elif raw_seen_in_roi:
            # Object visible in ROI but ID not yet assigned or depth not ready.
            # Record the time of first persistent depth failure for timeout.
            last_raw_seen_time = now_loop
            if depth_fail > 0 and depth_fail_since == 0.0:
                depth_fail_since = now_loop
            # If this frame produced an anonymous depth (id_fail but dep valid),
            # save it as a best-target candidate so nav can dispatch with it
            # if a real track ID is never assigned during the settle window.
            if (_anon_target is not None
                    and (now_loop - _anon_target['ts']) < 0.1
                    and (_post_scan_best_target is None
                         or now_loop - _post_scan_best_target['ts'] > 1.0)):
                _post_scan_best_target = {
                    'tid': -1,
                    'dist': _anon_target['dist'],
                    'angle': _anon_target['angle'],
                    'ts': now_loop,
                }
        else:
            # Nothing in ROI — clear both counters.
            depth_fail_since = 0.0

        # raw_hold_active: pause scan while object is transiently invisible or
        # depth is still catching up, but release after DEPTH_FAIL_TIMEOUT_S of
        # persistent depth failure (object may be outside stereo range).
        _depth_timed_out = (
            depth_fail_since > 0.0
            and (now_loop - depth_fail_since) > DEPTH_FAIL_TIMEOUT_S
        )
        raw_hold_active = (
            (now_loop - last_raw_seen_time) <= RAW_SEEN_HOLD_S
            and not _depth_timed_out
        )

        if (sorted_targets or raw_hold_active) and _within_collect_range:
            nav_interrupt_event.set()
        else:
            nav_interrupt_event.clear()

        if (DEBUG_DETECTION_REASONS and raw_det_count > 0 and valid_count == 0
                and now_loop - last_detection_debug_print > 0.7):
            print(f"[detect-but-no-target] raw={raw_det_count} "
                  f"roi_fail={roi_fail} id_fail={id_fail} "
                  f"depth_fail={depth_fail} raw_in_roi={raw_seen_in_roi}")
            last_detection_debug_print = now_loop

        # ------------------------------------------------------------------
        #  NAVIGATION LOGIC
        #  nav_busy / active_id protected by _nav_lock (from short file).
        #  navigate() uses stabilise() with fresh last_seen check (short file).
        #  scan uses send_turn_interruptible (short file).
        # ------------------------------------------------------------------
        with _nav_lock:
            _nav_busy_snapshot       = nav_busy
            _active_id_for_filter   = active_id   # captured for next-target filtering below

        # -- Detection-loss watchdog (active during navigation) -----------
        if _nav_busy_snapshot and _nav_started_from_scan:
            if sorted_targets or raw_seen_in_roi:
                nav_detection_loss_count = 0
            elif _nav_in_planned_settle:
                # Planned settle window (post-turn / post-move): the bot is
                # stationary and the camera may not see the target at this
                # angle yet.  Suppress the watchdog entirely — stabilise()
                # will handle the no-target case when it runs.
                nav_detection_loss_count = 0
            else:
                nav_detection_loss_count += 1
                if nav_detection_loss_count == NAV_LOSS_GRACE_FRAMES:
                    print(f"  [watchdog] No detections for "
                          f"{nav_detection_loss_count} frames during navigation — monitoring...")
                elif nav_detection_loss_count >= NAV_LOSS_STOP_FRAMES:
                    print(f"  [watchdog] Detection lost {nav_detection_loss_count} "
                          f"consecutive frames — sending STOP to ESP")
                    esp.send_stop()
                    nav_detection_loss_count = 0
        else:
            nav_detection_loss_count = 0

        # Guard: non-blocking post-scan settle — two independent timers:
        #   _post_scan_settle_until  : blocks nav dispatch (physical stop, 2s)
        #   _post_scan_restart_ok_after: blocks scan restart (depth recovery, 5.5s)
        _scan_settling = (now_loop < _post_scan_settle_until) or scan_busy or search_pattern_busy

        # If the settle just expired and live sorted_targets is empty, fall back
        # to the last target that had valid depth (saved above).  This handles the
        # common case where depth flickered in during the settle window but the
        # 0.7 s depth cache has since expired before the 3 s settle finished.
        _PEND_GRACE = POST_SCAN_NAV_SETTLE_S + 2.0
        _effective_targets = sorted_targets
        if (not _effective_targets and not _scan_settling
                and not _nav_busy_snapshot            # never fire while nav thread is live
                and _post_scan_best_target is not None
                and (now_loop - _post_scan_best_target['ts']) < _PEND_GRACE
                and _post_scan_best_target['tid'] not in tracker.collected):
            pt = _post_scan_best_target
            _effective_targets = [(pt['tid'], {
                'id': pt['tid'], 'dist': pt['dist'],
                'angle': pt['angle'], 'conf': 0.0, 'cov': 0.0,
            })]
            print(f"  [pending-target] Depth was valid {now_loop - pt['ts']:.1f}s ago "
                  f"— dispatching nav to ID={pt['tid']}  dist={pt['dist']:.2f}m")

        # Final gate: whatever _effective_targets ended up being (live
        # detection or the pending-target fallback above), drop it entirely
        # if it's beyond COLLECT_MAX_DIST_M. This must happen HERE, after
        # _effective_targets is fully resolved, not earlier against
        # sorted_targets alone — otherwise the pending-target fallback path
        # could still slip an out-of-range target through to the dispatch
        # block below, which does not otherwise check distance at all (only
        # nav_interrupt_event did, which is a different, narrower gate used
        # solely to abort an in-progress scan-dance step).
        if _effective_targets and _effective_targets[0][1]['dist'] > COLLECT_MAX_DIST_M:
            if now_loop - _last_ignore_print_time > 1.0:
                _ctx = "scan-dance" if search_pattern_busy else "idle"
                print(f"  [{_ctx}] target at {_effective_targets[0][1]['dist']:.2f}m > "
                      f"{COLLECT_MAX_DIST_M:.2f}m collect range — ignoring")
                _last_ignore_print_time = now_loop
            _effective_targets = []

        if not _nav_busy_snapshot and not _scan_settling and _effective_targets:
            tid, t = _effective_targets[0]

            # Stop scan state so no new scan steps are spawned.
            # NOTE: we do NOT sleep here — just set the non-blocking guard.
            if scan_active:
                _nav_started_from_scan = True
                scan_active    = False
                _scan_resume_pending = True
                nav_interrupt_event.set()   # tell scan_step thread to abort
                print(f"\n  Target found — stopping scan at {scan_total_deg:.0f}deg")
                esp.send_stop()
                _post_scan_settle_until     = now_loop + POST_SCAN_NAV_SETTLE_S
                _post_scan_restart_ok_after = now_loop + POST_SCAN_RESTART_BLOCK_S
                print(f"  [brakes] Nav settle {POST_SCAN_NAV_SETTLE_S:.1f}s, "
                      f"scan blocked {POST_SCAN_RESTART_BLOCK_S:.1f}s (non-blocking)...")
                # Do NOT start nav this frame — the guard above will block it
                # until the settle expires and scan_busy clears.

            else:
                with _nav_lock:
                    if active_id != tid:
                        active_id = tid
                        print(f"\n  Target: ID={tid}  dist={t['dist']:.2f}m  "
                              f"angle={t['angle']:+.1f}deg")

                def navigate(tid=tid):
                    nonlocal nav_busy, active_id, _nav_in_planned_settle, _nav_total_turn_deg
                    _nav_total_turn_deg = 0.0

                    def planned_settle(wait_s, label):
                        """Sleep in the background thread while signalling the
                        main loop that this is a *planned* settle window so the
                        watchdog does not fire spurious CMD:STOP commands."""
                        nonlocal _nav_in_planned_settle
                        print(f"  [settle/{label}] {wait_s:.1f}s...")
                        _nav_in_planned_settle = True
                        deadline = time.time() + wait_s
                        while time.time() < deadline:
                            time.sleep(0.05)   # yield in small slices
                        _nav_in_planned_settle = False

                    def stabilise(wait_s, label="stabilise",
                                  fallback_dist=None, fallback_angle=None):
                        """Collect median of fresh tracker readings over wait_s seconds.
                           Falls back to anonymous depth (id_fail frames) when the tracker
                           has no fresh entry — handles post-turn ID reassignment.
                           If n=0 and the target is very close (≤ CLOSE_RANGE_DIRECT_MOVE_M),
                           falls back to fallback_dist/angle so the bot doesn't abort on a
                           target it can no longer see because it's right under the camera."""
                        print(f"  [stabilise/{label}] gathering frames for {wait_s:.1f}s...")
                        samples_d, samples_a = [], []
                        deadline = time.time() + wait_s
                        while time.time() < deadline:
                            time.sleep(PRE_MOVE_SAMPLE_INTERVAL_S)
                            _added = False
                            with tracker._lock:
                                if tracker.targets:
                                    # ALWAYS pick the closest target at this microsecond
                                    st = sorted(tracker.targets.items(),
                                                key=lambda x: x[1]['dist'])
                                    fresh = st[0][1]
                                    # Check if frame is fresh (< 0.5s old)
                                    if time.time() - fresh['last_seen'] <= 0.5:
                                        d = float(fresh['dist'])
                                        a = float(fresh['angle'])
                                        if (not samples_d or
                                                abs(d - samples_d[-1]) <= PRE_MOVE_MAX_JUMP_M):
                                            samples_d.append(d)
                                            samples_a.append(a)
                                            _added = True
                            # No fresh tracker data — fall back to anonymous depth
                            # captured from id_fail frames in the main loop.
                            if not _added and _anon_target is not None:
                                if (time.time() - _anon_target['ts']) < 2.0:
                                    d = float(_anon_target['dist'])
                                    a = float(_anon_target['angle'])
                                    if (not samples_d or
                                            abs(d - samples_d[-1]) <= PRE_MOVE_MAX_JUMP_M):
                                        samples_d.append(d)
                                        samples_a.append(a)
                        if len(samples_d) < STABILISE_MIN_SAMPLES:
                            # Special case: target invisible because the bot is right
                            # on top of it (< CLOSE_RANGE_DIRECT_MOVE_M) and YOLO
                            # simply can't see it anymore.  Trust the last-known
                            # measurement passed in by the caller.
                            if (len(samples_d) == 0
                                    and fallback_dist is not None
                                    and fallback_dist <= CLOSE_RANGE_DIRECT_MOVE_M):
                                fb_a = fallback_angle if fallback_angle is not None else 0.0
                                print(f"  [stabilise/{label}] n=0 — target very close "
                                      f"({fallback_dist:.2f}m ≤ {CLOSE_RANGE_DIRECT_MOVE_M:.2f}m)"
                                      f", using last-known dist")
                                return fallback_dist, fb_a
                            print(f"  [stabilise/{label}] Only n={len(samples_d)} sample(s) "
                                  f"— too few to trust, aborting maneuver")
                            return None, None
                        if not samples_d:
                            print(f"  [stabilise/{label}] Target lost or unstable "
                                  f"— aborting maneuver")
                            return None, None
                        sd = float(np.median(samples_d))
                        sa = float(np.median(samples_a))
                        print(f"  [stabilise/{label}] n={len(samples_d)}  "
                              f"dist={sd:.2f}m  angle={sa:+.1f}deg")
                        return sd, sa

                    def abort_nav():
                        nonlocal _nav_in_planned_settle
                        _nav_in_planned_settle = False

                    # Phase A: initial stabilise
                    # Fallback: dispatch values — used when target dips below FOV
                    # (< CLOSE_RANGE_DIRECT_MOVE_M) and YOLO can't see it anymore.
                    dist, angle = stabilise(INITIAL_STABILIZE_S, label="initial",
                                            fallback_dist=t['dist'],
                                            fallback_angle=t['angle'])
                    if dist is None:
                        _nav_in_planned_settle = False
                        with _nav_lock:
                            active_id = None
                            nav_busy  = False
                        return

                    # Remember the confirmed initial dist/angle so post-turn
                    # and subsequent phases can fall back to it.
                    _confirmed_dist  = dist
                    _confirmed_angle = angle

                    # Phase B: turn to face target
                    # Skip for close targets — rotating when very close shifts
                    # the large bounding box outside the ROI polygon and the
                    # small lateral offset is acceptable.
                    #
                    # NOTE (NEO 3 / compass integration): this turn intentionally
                    # keeps using the ORIGINAL timed CMD:TURN (esp.send_turn), not
                    # the compass-verified esp.send_turn_compass(). The compass is
                    # only used for scan / search-pattern turns — detection,
                    # stabilise, and re-stabilise logic below are unchanged.
                    if abs(angle) > ANGLE_THRESH and dist > CLOSE_RANGE_NO_TURN_M:
                        esp.send_turn(angle)
                        _nav_total_turn_deg += angle
                        planned_settle(NAV_POST_TURN_SETTLE_S, label="post-turn")

                        # Phase C: re-stabilise; optional correction turn.
                        # After the turn the bot should be roughly facing the target
                        # so angle fallback is 0.0 (straight ahead).
                        dist, angle = stabilise(PRE_MOVE_STABILIZE_S, label="post-turn",
                                                fallback_dist=_confirmed_dist,
                                                fallback_angle=0.0)
                        if dist is None:
                            _nav_in_planned_settle = False
                            with _nav_lock:
                                active_id = None
                                nav_busy  = False
                            return

                        if abs(angle) > ANGLE_THRESH:
                            # Adaptive overshoot compensation: if the initial turn
                            # reversed the sign of the angle (bot turned past the
                            # target), the hardware overshoot factor k is:
                            #   k = (|initial| + |post_turn|) / |initial|
                            # Correct by initial / (initial + post_turn) so the
                            # physical movement cancels the residual error exactly.
                            if (_confirmed_angle * angle) < 0:  # sign flip → overshot
                                _c_gain = (abs(_confirmed_angle)
                                           / (abs(_confirmed_angle) + abs(angle)))
                                print(f"  [correction turn] {angle:+.1f}deg  "
                                      f"(overshoot — gain={_c_gain:.2f}  "
                                      f"cmd={angle * _c_gain:+.1f}deg)")
                            else:
                                _c_gain = 1.0
                                print(f"  [correction turn] {angle:+.1f}deg")
                            esp.send_turn(angle * _c_gain)
                            _nav_total_turn_deg += angle * _c_gain
                            planned_settle(NAV_POST_TURN_SETTLE_S, label="post-correction")
                            dist, angle = stabilise(PRE_MOVE_STABILIZE_S,
                                                    label="post-correction",
                                                    fallback_dist=_confirmed_dist,
                                                    fallback_angle=0.0)
                            if dist is None:
                                _nav_in_planned_settle = False
                                with _nav_lock:
                                    active_id = None
                                    nav_busy  = False
                                return

                    # Phase D: pre-move stabilise then move
                    dist, angle = stabilise(PRE_MOVE_STABILIZE_S, label="pre-move",
                                            fallback_dist=_confirmed_dist,
                                            fallback_angle=0.0)
                    if dist is None:
                        _nav_in_planned_settle = False
                        with _nav_lock:
                            active_id = None
                            nav_busy  = False
                        return

                    esp.send_move(dist)
                    planned_settle(NAV_POST_MOVE_SETTLE_S, label="post-move")
                    if tid is not None and tid >= 0:
                        tracker.mark_collected(tid)

                    # Return to the pre-approach position so the scan can
                    # continue from the paused point instead of the pickup spot.
                    esp.send_backward(dist)
                    planned_settle(NAV_POST_MOVE_SETTLE_S, label="return-home")

                    _nav_in_planned_settle = False
                    with _nav_lock:
                        active_id = None
                        nav_busy  = False

                with _nav_lock:
                    _nav_started_from_scan = False
                    nav_busy = True
                _last_nav_target_angle  = t['angle']   # remember direction in case nav fails
                _post_scan_best_target  = None   # nav is live — discard pending
                threading.Thread(target=navigate, daemon=True).start()

        elif not _effective_targets and not _nav_busy_snapshot:
            # -- Scan logic ------------------------------------------------
            # Two independent reasons to hold off:
            #   1. raw_hold_active — YOLO sees target but ID/depth not ready
            #   2. _post_scan_restart_ok_after — hard block after scan interrupt,
            #      ensures depth has time to recover after a turn regardless of
            #      whether YOLO detects anything in every individual frame.
            _scan_restart_blocked = now_loop < _post_scan_restart_ok_after
            if raw_hold_active or _scan_restart_blocked:
                if DEBUG_DETECTION_REASONS and now_loop - last_detection_debug_print > 0.7:
                    depth_age = (now_loop - depth_fail_since) if depth_fail_since > 0 else 0.0
                    restart_hold = max(0.0, _post_scan_restart_ok_after - now_loop)
                    print(f"  [scan-paused] depth_fail_age={depth_age:.1f}s "
                          f"restart_block_remaining={restart_hold:.1f}s")
                    last_detection_debug_print = now_loop
            else:
                elapsed = time.time() - start_time

                if elapsed >= SCAN_STARTUP_GRACE and not search_pattern_busy and not search_paused_for_nav:
                    with _nav_lock:
                        _active_id_snap = active_id
                    # Only restart scan when BOTH hold-offs have expired.
                    if ((_active_id_snap is not None or not scan_active)
                            and now_loop >= _post_scan_restart_ok_after):
                        with _nav_lock:
                            active_id = None
                        scan_active    = True
                        if not _scan_resume_pending:
                            scan_total_deg = 0.0
                        if _scan_resume_pending:
                            _scan_resume_pending = False
                            print(f"  Resuming scan from {scan_total_deg:.0f}deg "
                                  f"({scan_direction:+d} direction)")
                        else:
                            # Intelligently choose scan direction based on the last
                            # known angle of any un-collected target.  Priority:
                            #   1. _post_scan_best_target  (fresh depth from a different track)
                            #   2. _last_nav_target_angle  (angle of the last nav attempt,
                            #      survives _post_scan_best_target clears after nav failures)
                            #   3. default sweep right
                            if (_post_scan_best_target is not None
                                    and (now_loop - _post_scan_best_target['ts']) < 15.0
                                    and _post_scan_best_target['tid'] not in tracker.collected):
                                scan_direction = 1 if _post_scan_best_target['angle'] >= 0 else -1
                                _sd_label = 'left' if scan_direction > 0 else 'right'
                                print(f"  Scan direction: {_sd_label} "
                                      f"(best-target angle "
                                      f"{_post_scan_best_target['angle']:+.1f}deg)")
                            elif (_last_nav_target_angle is not None
                                    and abs(_last_nav_target_angle) >= ANGLE_THRESH):
                                # Only use the saved angle when it is meaningfully
                                # directional (>= ANGLE_THRESH).  Angles below the
                                # turn threshold are essentially noise and would send
                                # the scan the wrong way.
                                scan_direction = 1 if _last_nav_target_angle >= 0 else -1
                                _sd_label = 'left' if scan_direction > 0 else 'right'
                                print(f"  Scan direction: {_sd_label} "
                                      f"(last nav angle "
                                      f"{_last_nav_target_angle:+.1f}deg)")
                            else:
                                scan_direction = 1   # default: sweep left
                            _post_scan_best_target = None   # fresh scan cycle
                            print("\n  No targets in ROI — starting scan...")

            if scan_active and not scan_busy and not search_pattern_busy and not (raw_hold_active or _scan_restart_blocked):
                if scan_total_deg >= SCAN_MAX_DEG:
                    print("  Full scan complete — no targets found. Starting search pattern...")
                    esp.send_stop()
                    scan_active             = False
                    scan_total_deg          = 0.0
                    scan_busy               = False
                    _scan_resume_pending    = False
                    _last_nav_target_angle  = None    # stale after full sweep

                    def search_pattern():
                        """Stepped Serpentine search pattern.

                        Walks SEARCH_TOTAL_LANES lanes of length
                        SEARCH_LANE_LENGTH_M, pausing every STEP_DISTANCE_M to
                        run a compass-verified "Scan Dance"
                        (right 90 -> center -> left 90 -> center). At the end
                        of a lane the robot performs a sideways lane-shift
                        (alternating LEFT / RIGHT — true serpentine) and
                        resets its in-lane distance to 0.0m before the next
                        lane's 0.0m Scan Dance.

                        Detection interrupts (nav_interrupt_event) are checked
                        before every atomic turn/move via _sp_turn()/_sp_move().
                        On interrupt the routine stops, hands off to
                        navigate() for collection via _sp_pause_for_collection(),
                        restores the exact heading via
                        esp.send_turn_compass(-_nav_total_turn_deg), and then
                        retries the very step that was interrupted — so the
                        lane_index / distance_travelled_m state (held in the
                        loop below) is never lost and the patrol resumes
                        exactly where it left off.
                        """
                        nonlocal search_pattern_busy, scan_active, scan_total_deg, \
                                 _post_scan_settle_until, _post_scan_restart_ok_after, \
                                 search_paused_for_nav, _nav_total_turn_deg

                        def _sp_pause_for_collection(label):
                            nonlocal search_pattern_busy, search_paused_for_nav, _post_scan_settle_until
                            print(f"  [search] {label} — target detected, pausing search pattern for collection...")
                            esp.send_stop()
                            search_paused_for_nav = True
                            search_pattern_busy = False

                            # Clear settle guard so nav can start immediately
                            _post_scan_settle_until = time.time()

                            # Wait for nav to start (i.e., nav_busy = True)
                            start_wait = time.time()
                            nav_started = False
                            while time.time() - start_wait < 3.0:
                                with _nav_lock:
                                    if nav_busy:
                                        nav_started = True
                                        break
                                time.sleep(0.05)

                            if nav_started:
                                print("  [search] Navigation started, waiting for collection completion...")
                                while True:
                                    with _nav_lock:
                                        if not nav_busy:
                                            break
                                    time.sleep(0.1)
                                print("  [search] Collection completed, waiting for recovery settle...")
                                time.sleep(POST_SCAN_RESTART_BLOCK_S)
                            else:
                                print("  [search] Navigation did not start (transient detection?), resuming search...")
                                time.sleep(1.0)

                            # Undo any navigation turns that occurred, restoring
                            # absolute heading for the search pattern. Compass-
                            # verified so the restored heading is exact regardless
                            # of accumulated timed-turn drift from navigate()'s turns.
                            if _nav_total_turn_deg != 0.0:
                                print(f"  [search] Undoing navigation turn of {_nav_total_turn_deg:.1f}deg to restore search heading")
                                esp.send_turn_compass(-_nav_total_turn_deg)
                                _sp_wait("restore-heading")

                            search_pattern_busy = True
                            search_paused_for_nav = False
                            print("  [search] Search pattern resumed.")

                        def _sp_wait(label="settle"):
                            """Brief settle after a move/turn. Handles pause/resume on interrupt."""
                            while True:
                                deadline = time.time() + SEARCH_SETTLE_S
                                interrupted = False
                                while time.time() < deadline:
                                    if nav_interrupt_event.is_set():
                                        interrupted = True
                                        break
                                    time.sleep(0.05)
                                if interrupted:
                                    _sp_pause_for_collection(label)
                                    # Repeat settle after resuming
                                    continue
                                else:
                                    break

                        def _sp_turn(angle_deg, label, tol_deg=None):
                            """Compass-verified, interruptible turn. If a
                            collection interrupt fires before or during the
                            turn, pause for collection then retry toward the
                            ORIGINAL intended absolute heading — not a fresh
                            angle_deg relative to wherever the boat has
                            drifted to. A boat that was already partway
                            through the turn (or coasted further on momentum
                            while stopped for the pause) would otherwise
                            accumulate that drift PLUS a full fresh angle_deg
                            every time it gets interrupted, which is exactly
                            what the scan-dance edge double-check is there
                            to catch (and did).

                            tol_deg, if given, widens the ESP's accept
                            tolerance so it skips its reach-verify coast-settle
                            — used for intermediate splits that don't need to
                            be exact."""
                            absolute_target = None
                            remaining_delta = angle_deg
                            while True:
                                if nav_interrupt_event.is_set():
                                    if absolute_target is None:
                                        cur = esp.get_heading()
                                        if cur is not None:
                                            absolute_target = (cur + angle_deg) % 360.0
                                    _sp_pause_for_collection(f"{label} (pre-turn)")
                                    if absolute_target is not None:
                                        cur = esp.get_heading()
                                        if cur is not None:
                                            remaining_delta = heading_diff_deg(cur, absolute_target)
                                    continue
                                if absolute_target is None:
                                    cur = esp.get_heading()
                                    if cur is not None:
                                        absolute_target = (cur + angle_deg) % 360.0
                                completed = esp.send_turn_compass_interruptible(
                                    remaining_delta, nav_interrupt_event, tol_deg=tol_deg)
                                if not completed or nav_interrupt_event.is_set():
                                    _sp_pause_for_collection(f"{label} (interrupted)")
                                    if absolute_target is not None:
                                        cur = esp.get_heading()
                                        if cur is not None:
                                            remaining_delta = heading_diff_deg(cur, absolute_target)
                                    continue
                                break
                            if esp.last_turn_stalled:
                                print(f"  [search] {label}: completed via bump/stall "
                                      f"recovery on the ESP (see its serial log)")
                            _sp_wait(label)
                        def _sp_turn_to(target_heading, label):
                            """Like _sp_turn, but targets a KNOWN ABSOLUTE heading (the center
                            noted at the start of a scan-dance) rather than a relative delta
                            from wherever the boat currently is. The dance's return-to-center
                            steps used to be blind -90/+90 relative turns — if the 3 preceding
                            30deg steps had drifted even a little (compass tolerance + coast
                            during settle), turning -90 from the DRIFTED heading compounds
                            that drift instead of correcting it, so the boat never actually
                            returns to the true noted center. Returns False (caller should
                            fall back to a relative turn) if heading can't be read."""
                            absolute_target = target_heading
                            while True:
                                cur = esp.get_heading()
                                if cur is None:
                                    return False
                                remaining_delta = heading_diff_deg(cur, absolute_target)
                                if nav_interrupt_event.is_set():
                                    _sp_pause_for_collection(f"{label} (pre-turn)")
                                    continue
                                completed = esp.send_turn_compass_interruptible(
                                    remaining_delta, nav_interrupt_event)
                                if not completed or nav_interrupt_event.is_set():
                                    _sp_pause_for_collection(f"{label} (interrupted)")
                                    continue
                                break
                            if esp.last_turn_stalled:
                                print(f"  [search] {label}: completed via bump/stall "
                                      f"recovery on the ESP (see its serial log)")
                            _sp_wait(label)
                            return True
                            
                        def _sp_move(dist_m, label, centre_corrected=True):
                            """Forward/backward move. The ESP's CMD:CMOVE
                            automatically corrects heading drift before moving
                            (compass median-of-3, glitch-safe, interruptible
                            via the ESP's non-blocking state machine). The
                            pre-move interrupt check and post-move settle
                            window remain the only places a detection can
                            pause this function."""
                            while True:
                                if nav_interrupt_event.is_set():
                                    _sp_pause_for_collection(f"{label} (pre-move)")
                                    continue
                                if centre_corrected:
                                    esp.send_cmove(dist_m)
                                else:
                                    esp.send_move(dist_m)
                                break
                            _sp_wait(label)

                        def _sp_verify_center(label, center_heading):
                            """After a return-to-centre turn, verify the achieved
                            heading actually matches the noted centre heading
                            (within tolerance) — i.e. check the RETURN, not the
                            outward edge. Verification-only — logs a clear
                            match/mismatch, does not itself correct the heading
                            (the stabilise/settle wait already happened as part
                            of the return turn via _sp_turn_to's _sp_wait call)."""
                            if center_heading is None:
                                print(f"  [verify] {label}: center heading unknown "
                                      f"(GETHEADING failed) — skipping check")
                                return
                            achieved = esp.get_heading()
                            if achieved is None:
                                print(f"  [verify] {label}: could not read heading "
                                      f"for check")
                                return
                            err = heading_diff_deg(achieved, center_heading)
                            if abs(err) <= SCAN_EDGE_VERIFY_TOL_DEG:
                                print(f"  [verify] {label}: OK — heading={achieved:.1f} "
                                      f"centre={center_heading:.1f} err={err:+.1f}")
                            else:
                                print(f"  [verify] {label}: MISMATCH — heading={achieved:.1f} "
                                      f"centre={center_heading:.1f} err={err:+.1f} "
                                      f"(tol={SCAN_EDGE_VERIFY_TOL_DEG:.1f})")


                        def _scan_dance(label):
                            """Note the centre heading (cheading), then per side:
                              - 2x coarse 30deg splits out (no reach-verify —
                                these don't need to be exact)
                              - one precise turn to the absolute cheading+-90
                                edge, correcting any drift from the two splits
                              - one precise turn back to cheading, verified
                            """
                            print(f"  [search] scan-dance @ {label}")
                            center_heading = esp.get_heading()
                            if center_heading is not None:
                                print(f"  [search] {label}: cheading noted = "
                                      f"{center_heading:.1f}deg")
                            else:
                                print(f"  [search] {label}: could not note cheading "
                                      f"(GETHEADING failed) — edge/return-to-centre "
                                      f"checks will be skipped for this stop")

                            # Right side
                            _sp_turn(+SCAN_STEP_DEG, f"{label} right-step-1/3",
                                     tol_deg=SCAN_STEP_COARSE_TOL_DEG)
                            _sp_turn(+SCAN_STEP_DEG, f"{label} right-step-2/3",
                                     tol_deg=SCAN_STEP_COARSE_TOL_DEG)
                            if center_heading is None or not _sp_turn_to(
                                    (center_heading + SCAN_EDGE_DEG) % 360.0, f"{label} right-edge"):
                                _sp_turn(+SCAN_STEP_DEG, f"{label} right-step-3/3")
                            if center_heading is None or not _sp_turn_to(center_heading, f"{label} return-to-center-from-right"):
                                _sp_turn(-SCAN_EDGE_DEG, f"{label} return-to-center-from-right")
                            _sp_verify_center(f"{label} right-return", center_heading)

                            # Left side
                            _sp_turn(-SCAN_STEP_DEG, f"{label} left-step-1/3",
                                     tol_deg=SCAN_STEP_COARSE_TOL_DEG)
                            _sp_turn(-SCAN_STEP_DEG, f"{label} left-step-2/3",
                                     tol_deg=SCAN_STEP_COARSE_TOL_DEG)
                            if center_heading is None or not _sp_turn_to(
                                    (center_heading - SCAN_EDGE_DEG) % 360.0, f"{label} left-edge"):
                                _sp_turn(-SCAN_STEP_DEG, f"{label} left-step-3/3")
                            if center_heading is None or not _sp_turn_to(center_heading, f"{label} return-to-center-from-left"):
                                _sp_turn(+SCAN_EDGE_DEG, f"{label} return-to-center-from-left")
                            _sp_verify_center(f"{label} left-return", center_heading)

                        print(f"  [search] Starting stepped-serpentine patrol: "
                              f"{SEARCH_TOTAL_LANES} lanes x {SEARCH_LANE_LENGTH_M:.1f}m, "
                              f"shift {SEARCH_LANE_SHIFT_M:.1f}m")

                        # Store the lane's forward heading on the ESP (median-of-3,
                        # glitch-safe). CMD:CMOVE will automatically correct back
                        # to this heading before every lane step, handling drift
                        # and I2C re-init glitches from motor EMI.
                        esp.send_set_centre()

                        for lane_index in range(SEARCH_TOTAL_LANES):
                            distance_travelled_m = 0.0
                            print(f"  [search] Lane {lane_index}: start (0.00m)")
                            _scan_dance(f"lane{lane_index}@0.00m")

                            while distance_travelled_m < SEARCH_LANE_LENGTH_M - 1e-6:
                                _sp_move(STEP_DISTANCE_M, f"lane{lane_index} step-forward")
                                distance_travelled_m += STEP_DISTANCE_M
                                scan_total_deg = distance_travelled_m  # reuse field for progress logging

                                if distance_travelled_m >= SEARCH_LANE_LENGTH_M - 1e-6:
                                    print(f"  [search] Lane {lane_index}: boundary reached "
                                          f"({distance_travelled_m:.2f}m) — no scan")
                                    break

                                print(f"  [search] Lane {lane_index}: {distance_travelled_m:.2f}m")
                                _scan_dance(f"lane{lane_index}@{distance_travelled_m:.2f}m")

                            if lane_index < SEARCH_TOTAL_LANES - 1:
                                # Alternate shift direction: lane0->1 LEFT,
                                # lane1->2 RIGHT, lane2->3 LEFT, ... (serpentine)
                                shift_left   = (lane_index % 2 == 0)
                                shift_angle  = -SCAN_EDGE_DEG if shift_left else +SCAN_EDGE_DEG
                                shift_label  = "LEFT" if shift_left else "RIGHT"
                                print(f"  [search] Lane shift {lane_index}->{lane_index+1}: {shift_label}")

                                # Turn 90deg to face the shift direction and note
                                # this heading as the "lheading" — the reference
                                # the sideways move stabilises against.
                                _sp_turn(shift_angle, f"lane-shift-{lane_index}-turn1-{shift_label}")
                                lheading = esp.send_set_centre()
                                if lheading is not None:
                                    print(f"  [search] Lane shift {lane_index}: "
                                          f"lheading noted = {lheading:.1f}deg")
                                else:
                                    print(f"  [search] Lane shift {lane_index}: "
                                          f"lheading read failed — shift move will "
                                          f"run without centre correction")

                                # Sideways shift move, compass-corrected against
                                # lheading the whole way (previously this move ran
                                # with no heading correction at all).
                                _sp_move(SEARCH_LANE_SHIFT_M, f"lane-shift-{lane_index}-move", centre_corrected=True)

                                # Turn 90deg again to face down the new lane. Target the
                                # absolute heading derived from lheading, not a relative
                                # turn from wherever the move left the boat pointing —
                                # CMOVE only corrects heading once before the move starts,
                                # so any drift during the move must not carry into the
                                # new lane's centre.
                                turn2_label = f"lane-shift-{lane_index}-turn2-{shift_label}"
                                if lheading is None or not _sp_turn_to((lheading + shift_angle) % 360.0, turn2_label):
                                    _sp_turn(shift_angle, turn2_label)

                                # Note the new lane's forward heading (cheading) —
                                # this is the first point of the new lane, and the
                                # patrol loop continues from distance_travelled_m=0.0
                                # with a fresh Scan Dance next iteration.
                                esp.send_set_centre()

                        print("  [search] Serpentine patrol complete — search finished")
                        search_pattern_busy = False
                        scan_active         = False

                    search_pattern_busy = True
                    threading.Thread(target=search_pattern, daemon=True,
                                     name="SearchPattern").start()
                else:
                    step_deg = SCAN_STEP_DEG * scan_direction

                    def scan_step(deg=step_deg):
                        nonlocal scan_busy, scan_total_deg, scan_active, \
                                 _post_scan_settle_until, _post_scan_restart_ok_after

                        def _interrupt_exit(reason):
                            """Immediately stop ESP, set both post-scan timers, clean up."""
                            nonlocal scan_active, scan_busy, _scan_resume_pending, \
                                     _post_scan_settle_until, _post_scan_restart_ok_after
                            print(f"  [scan] {reason}")
                            # Always send an explicit STOP so the ESP is commanded
                            # regardless of which scan phase was interrupted
                            # (before-turn / mid-turn / during-settle).
                            esp.send_stop()
                            now = time.time()
                            _post_scan_settle_until     = now + POST_SCAN_NAV_SETTLE_S
                            _post_scan_restart_ok_after = now + POST_SCAN_RESTART_BLOCK_S
                            _scan_resume_pending = True
                            scan_active = False
                            scan_busy   = False

                        # Abort immediately if target already visible
                        if nav_interrupt_event.is_set():
                            _interrupt_exit("Target visible before turn — aborting")
                            return

                        # Interruptible, compass-verified turn: ESP sends STOP mid-turn
                        # if event fires, and stops at the true target heading rather
                        # than after a fixed timed duration.
                        completed = esp.send_turn_compass_interruptible(deg, nav_interrupt_event)
                        if not completed:
                            _interrupt_exit("Turn interrupted — target visible during turn")
                            return

                        scan_total_deg += SCAN_STEP_DEG
                        print(f"  Scanning... {scan_total_deg:.0f}/{SCAN_MAX_DEG:.0f}deg")

                        # Settle in short slices so detection can break in at any time
                        settle_deadline = time.time() + SCAN_STEP_SETTLE_S
                        while time.time() < settle_deadline:
                            if nav_interrupt_event.is_set():
                                _interrupt_exit("Target appeared during settle — exiting")
                                return
                            time.sleep(0.05)   # 50ms slices for fast reaction

                        scan_busy = False

                    scan_busy = True
                    threading.Thread(target=scan_step, daemon=True).start()

        # -- Draw ----------------------------------------------------------
        with _nav_lock:
            _active_id_draw = active_id
        ann_l = draw_detections(
            rect_l, result_l, dep, "LEFT",
            is_main_view=True,
            track_history=track_history_l,
            roi_poly=roi_poly_left,
            active_id=_active_id_draw,
            sorted_targets=sorted_cache,
            fx=calib['fx']
        )
        ann_r = draw_detections(
            rect_r, None, None, "RIGHT",
            is_main_view=False,
            roi_poly=roi_poly_right,
            fx=calib['fx']
        )

        if roi_draw_mode:
            roi_drawer.current_side = roi_draw_side
            if roi_draw_side == 'right':
                ann_r = roi_drawer.draw_on_frame(ann_r)
                cv2.putText(ann_r,
                            f"[ROI RIGHT] {len(roi_drawer.points)}pts "
                            f"| dbl-click finish",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 255, 0), 2)
            else:
                ann_l = roi_drawer.draw_on_frame(ann_l)
                cv2.putText(ann_l,
                            f"[ROI LEFT] {len(roi_drawer.points)}pts "
                            f"| dbl-click finish",
                            (10, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 255, 0), 2)
        elif roi_norm_left or roi_norm_right:
            cv2.putText(ann_l, "[ROI ACTIVE] S=left  R=right  c=clear",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 255), 1)

        if show_epi:
            for y_ep in range(0, H, 40):
                cv2.line(ann_l, (0, y_ep), (W, y_ep), (0, 180, 0), 1)
                cv2.line(ann_r, (0, y_ep), (W, y_ep), (0, 180, 0), 1)

        fc += 1
        if fc % 10 == 0:
            fps   = 10.0 / (time.time() - t_fps)
            t_fps = time.time()

        wls_tag = "SGBM+WLS(50%)" if (USE_WLS and HAS_WLS) else "SGBM(50%)"
        esp_tag = "WiFi-OK" if esp.connected else "WiFi-X"
        with _nav_lock:
            _nb = nav_busy; _ai = active_id
        if _nb:
            status = f"NAVIGATING ID={_ai}"
        elif scan_active:
            status = f"SCANNING {scan_total_deg:.0f}deg"
        else:
            status = "IDLE"

        cv2.putText(ann_l,
                    f"FPS:{fps:.1f}  {wls_tag}  {esp_tag}  {status}",
                    (6, H - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (160, 160, 0), 1)

        combined = np.hstack([ann_l, ann_r])
        dw = int(combined.shape[1] * DISPLAY_SCALE)
        dh = int(combined.shape[0] * DISPLAY_SCALE)
        roi_drawer.set_scale_factor(dw / combined.shape[1])

        cv2.imshow(win_name, cv2.resize(combined, (dw, dh)))

        if roi_draw_mode:
            cv2.setMouseCallback(win_name, roi_drawer.mouse_callback)

        if show_disp and disp is not None:
            dc = colorize_disp(disp)
            cv2.imshow("Disparity",
                       cv2.resize(dc, (int(W * DISPLAY_SCALE),
                                       int(H * DISPLAY_SCALE))))

        key  = cv2.waitKey(1) & 0xFF
        r_p  = (key == ord('r') or key == ord('R'))
        s_p  = (key == ord('s') or key == ord('S'))
        c_p  = (key == ord('c') or key == ord('C'))

        if roi_draw_mode and roi_drawer.polygon_norm is not None and not poly_done:
            poly_done = True
            if roi_draw_side == "left":
                roi_norm_left = roi_drawer.polygon_norm
                save_roi_for_side(roi_norm_left, "left")
            else:
                roi_norm_right = roi_drawer.polygon_norm
                save_roi_for_side(roi_norm_right, "right")
            roi_draw_mode = False
            roi_draw_side = None
            print("[ROI] Draw mode OFF — polygon saved")

        if s_p and not last_s:
            if roi_draw_mode and roi_draw_side == "left":
                roi_draw_mode = False; roi_draw_side = None
                roi_drawer.reset(); poly_done = False
                print("[ROI] Draw mode OFF (left discarded)")
            else:
                roi_draw_mode = True; roi_draw_side = "left"
                roi_drawer.reset(); poly_done = False
                print("[ROI] Draw mode ON (LEFT)")
        last_s = s_p

        if r_p and not last_r:
            if roi_draw_mode and roi_draw_side == "right":
                roi_draw_mode = False; roi_draw_side = None
                roi_drawer.reset(); poly_done = False
                print("[ROI] Draw mode OFF (right discarded)")
            else:
                roi_draw_mode = True; roi_draw_side = "right"
                roi_drawer.reset(); poly_done = False
                print("[ROI] Draw mode ON (RIGHT)")
        last_r = r_p

        if c_p and not last_c:
            roi_draw_mode = False; roi_draw_side = None
            roi_norm_left = None; roi_norm_right = None
            roi_drawer.reset(); delete_roi()
            print("[ROI] Cleared both ROIs")
        last_c = c_p

        roi_poly_left  = make_poly(roi_norm_left)
        roi_poly_right = make_poly(roi_norm_right)

        if key == ord('q'):
            break
        elif key == ord('d'):
            show_disp = not show_disp
            if not show_disp:
                cv2.destroyWindow("Disparity")
        elif key == ord('e'):
            show_epi = not show_epi
        elif key == ord('i'):
            debug = not debug
            print(f"Debug: {'ON' if debug else 'OFF'}")
        elif key in (ord('+'), ord('=')):
            conf = min(0.95, conf + 0.05)
            print(f"conf:{conf:.2f}")
        elif key == ord('-'):
            conf = max(0.10, conf - 0.05)
            print(f"conf:{conf:.2f}")

    # -- Cleanup -----------------------------------------------------------
    rl.stop()
    rr.stop()
    disp_worker.stop()
    esp.stop()
    cv2.destroyAllWindows()
    print("Done.")
    _tee.close()


if __name__ == "__main__":
    if len(sys.argv) == 4:
        RTSP_LEFT, RTSP_RIGHT, CALIB_FILE = sys.argv[1:4]
    elif len(sys.argv) != 1:
        print("Usage: python stereo_autonomous_combined.py "
              "[left_url right_url calib_file]")
        sys.exit(1)
    main()