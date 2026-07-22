/*
 * esp32_waterbot.ino
 * ──────────────────────────────────────────────────────────────────────────
 * Water-surface waste-collection bot — ESP32-WROOM-32 firmware
 * [UPDATED: Non-blocking motion + Explicit ledcAttach v3.x API]
 * [FIX: Non-blocking settle, priority CMD:STOP, double-DONE prevention]
 * [NEO 3 / IST8310: compass-verified closed-loop turns for scan commands]
 *
 * [FIX — POLARITY] Compass turns were driving the motors backwards relative
 * to the heading convention: commanding deltaDeg > 0 ("clockwise", heading
 * should INCREASE) was actually spinning the boat the other way, so the
 * heading DECREASED every poll. Combined with the reactive per-poll
 * direction-flip (see next fix), this meant the error only ever grew,
 * wrapped past 0/360, and the correction chased itself around the far side
 * of the compass rose until the 8s timeout. Motor direction for CMD:CTURN
 * is now swapped so deltaDeg > 0 genuinely increases heading.
 *
 * [FIX — NO MORE REACTIVE DIRECTION FLIP, EXCEPT IN NEAR MODE] Direction is
 * chosen ONCE in doTurnCompass() from the sign of deltaDeg and held fixed for
 * the whole FAR-mode (full-speed) phase — see compassTurnDir. FAR-mode
 * momentum can carry the hull past the target before the motor stop takes
 * effect though, so once in NEAR mode (short pulse/brake cycles) the pulse
 * direction IS recomputed each cycle from the live sign of err. This is safe
 * now that driveCompassDir()'s polarity itself is corrected — the old bug
 * this comment used to warn about was direction-flipping combined with
 * BACKWARDS polarity, which chased itself around the compass forever.
 * The final approach (within COMPASS_NEAR_THRESHOLD_DEG) uses short
 * full-stop-then-pulse cycles at a speed strong enough to actually move the
 * boat (COMPASS_NEAR_SPEED). Pulse duration is now adaptive
 * (compassPulseDurMs): it lengthens when a pulse makes negligible progress
 * against water drag/slosh, and resets to the base duration once progress
 * resumes, instead of a single fixed 150ms that could stall indefinitely.
 *
 * [FIX — FAR-MODE "REACHED" COULD BE A MOMENTUM ILLUSION] A single 250ms FAR
 * poll can, on a fast boat, jump the heading by 20+ deg — occasionally
 * landing directly inside COMPASS_TURN_TOL_DEG without ever slowing into
 * NEAR mode. motorsStop() was fired immediately and the turn reported DONE,
 * but at full FAR speed the hull keeps coasting well past that point before
 * it actually stops. NEAR-mode pulses can also leave a few degrees of
 * residual coast during STOP_SETTLE_MS, which showed up as right turns
 * undershooting and left turns overshooting by a few degrees. Every
 * "reached" event is now provisional: motors stop immediately, but the turn
 * is not finalized until an extra coast-settle window
 * (COMPASS_REACHED_VERIFY_MS) has passed and a fresh reading confirms the
 * hull actually settled within tolerance. If it coasted past, the turn
 * resumes closed-loop instead of reporting a false DONE.
 *
 * [FIX — UNCHECKED FINAL SETTLE READ] The one-shot compass read taken at
 * settle-complete (used for the "DONE HEADING=xx.x" report) had no
 * plausibility check, unlike every in-turn poll. In practice the first read
 * right after enableI2C()'s re-setup is frequently a glitchy 0.00 (I2C not
 * fully settled yet), which was being reported to the Jetson as a real
 * heading. This read now goes through the same
 * COMPASS_MAX_PLAUSIBLE_JUMP_DEG check as in-turn polls, falling back to the
 * last known good heading instead of reporting a glitch as fact.
 *
 * [FIX — CMOVE DID NOT CORRECT DURING THE MOVE] CMD:CMOVE only checked
 * heading once before starting; the actual drive was open-loop with I2C
 * disabled (to avoid motor EMI on the bus), so a motor imbalance could
 * drift the heading freely for the whole move with no correction at all.
 * CMD:CMOVE now walks the distance in short segments (CMOVE_SEGMENT_M),
 * pausing briefly between each to re-enable I2C, recheck heading against
 * the held target, and issue a compass-turn correction if it drifted
 * beyond CENTRE_CORRECT_TOL_DEG before continuing.
 *
 * ── CUAV NEO 3 (IST8310 compass) wiring ─────────────────────────────────
 *   Only the I2C lines are used (SDA/SCL) to read heading from the IST8310
 *   magnetometer on the NEO 3 module. The GPS UART (RX/TX) is NOT used.
 *     NEO 3 SDA -> ESP32 GPIO21
 *     NEO 3 SCL -> ESP32 GPIO22
 *     NEO 3 VCC -> 3.3V   NEO 3 GND -> GND
 *   Requires the IST8310 Arduino library (Wire-based), e.g.
 *   https://github.com/dayaftereh/rocket
 *
 * ── Jetson-side "Stepped Serpentine" search pattern ─────────────────────
 *   The Jetson's search_pattern() / _scan_dance() / lane-shift logic only
 *   ever issues CMD:CTURN (any signed angle — dance uses ±90/±30, lane
 *   shifts use ±90), CMD:CMOVE / CMD:MOVE (signed distance), CMD:SETCENTRE,
 *   CMD:GETHEADING, and CMD:STOP. No protocol changes were needed here for
 *   that; this file only needed the polarity, direction-flip, and
 *   reached-from-FAR / unchecked-settle-read fixes above.
 */

#include <WiFi.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include "IST8310.h"

// ═══════════════════════════════════════════════════════════════════════════
//  BLUETOOTH DEBUG SERIAL
// ═══════════════════════════════════════════════════════════════════════════
BluetoothSerial BT;

inline void BLOG(const char* msg) {
    Serial.print(msg);
    if (BT.hasClient()) BT.print(msg);
}
inline void BLOGln(const char* msg) {
    Serial.println(msg);
    if (BT.hasClient()) { BT.print(msg); BT.print("\r\n"); }
}
void BLOGf(const char* fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    Serial.print(buf);
    if (BT.hasClient()) BT.print(buf);
}

// ═══════════════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS
// ═══════════════════════════════════════════════════════════════════════════
#define PIN_ENA   32
#define PIN_IN1   33
#define PIN_IN2   25
#define PIN_ENB   26
#define PIN_IN3   27
#define PIN_IN4   14

#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// ═══════════════════════════════════════════════════════════════════════════
//  LEDC (PWM) CONFIG
// ═══════════════════════════════════════════════════════════════════════════
#define LEDC_FREQ      1000
#define LEDC_BITS      8

// ═══════════════════════════════════════════════════════════════════════════
//  SPEED SETTINGS  (0–255)
// ═══════════════════════════════════════════════════════════════════════════
#define SPEED_MOVE  240
#define SPEED_TURN  230  // calibrated — used for CMD:TURN and FAR-mode compass turns

float TURN_DEG_PER_SEC = 77.0;
float MOVE_M_PER_SEC   = 0.1429;
#define STOP_SETTLE_MS  150

// ═══════════════════════════════════════════════════════════════════════════
//  COMPASS (IST8310 via CUAV NEO 3) CONFIG
// ═══════════════════════════════════════════════════════════════════════════
#define COMPASS_TURN_TOL_DEG          3.0f
#define COMPASS_TURN_MAX_MS           10000UL // headroom for the near-mode pulse phase
#define COMPASS_POLL_MIN_MS           250UL
#define COMPASS_FAIL_ABORT_STREAK     15
#define COMPASS_DISABLE_AFTER_N_FAILS 10

// ── NEAR-target approach (replaces the reactive direction-flip) ────────────
// COMPASS_NEAR_SPEED must be strong enough to actually move the boat once
// it's this close — too weak (previously 145) and it just stalls in place.
#define COMPASS_NEAR_THRESHOLD_DEG    20.0f
#define COMPASS_NEAR_SPEED            190
#define COMPASS_PULSE_MS              150UL
#define COMPASS_PULSE_MS_MAX          400UL  // cap so a stalled pulse can't run away
#define COMPASS_PULSE_PROGRESS_MIN    1.0f   // deg — below this, count as "stalled" vs drag
#define COMPASS_BRAKE_MS              200UL
// If NEAR-mode pulse direction flips sign this many cycles in a row, the hull
// has enough momentum that pulses are overshooting the target both ways
// (hunting), not just being soaked up by drag — growing the pulse further
// would make that worse. Force a short, gentle "fine tap" instead.
#define COMPASS_OSCILLATION_LIMIT     2
#define COMPASS_PULSE_MS_FINE         80UL
// At TURN_DEG_PER_SEC=77 and a 250ms poll, a genuine reading can't move more
// than ~19-25deg between polls even at full FAR-mode speed. A reading that
// implies a bigger jump than this (e.g. a lone "0.00" glitch from I2C/EMI
// noise while the real heading was elsewhere) is almost certainly bad data,
// not real motion — reject it like a failed read rather than trusting it and
// spinning the boat an extra ~300deg chasing a phantom error.
#define COMPASS_MAX_PLAUSIBLE_JUMP_DEG 60.0f

// Reach verification: any "reached" event (FAR or NEAR) stops the motors
// immediately but doesn't finalize DONE until this coast-settle window
// passes and a fresh read confirms the heading actually held. Momentum from
// the last pulse/spin can still carry the hull a few degrees past target
// even in NEAR mode, which was showing up as small right-undershoot /
// left-overshoot on ordinary 30deg steps.
#define COMPASS_REACHED_VERIFY_MS  400UL

// Centre-heading correction applied before every CMD:CMOVE.
// The ESP reads the compass (median-of-3) when CMD:SETCENTRE arrives and
// stores it as centreHeadingDeg. Before a CMD:CMOVE move starts, if the
// current heading deviates by more than this from the stored centre, a
// doTurnCompass() correction runs first in the same non-blocking state
// machine — no extra TCP round-trips needed from the Jetson.
#define CENTRE_CORRECT_TOL_DEG        5.0f

// ── Stall detection/escalation (e.g. a floor bump stopping the hull) ───────
// FAR mode: expected minimum heading change between polls at full speed —
// if progress stays below this several polls running, ramp PWM.
#define FAR_STALL_PROGRESS_MIN_DEG    2.0f
#define FAR_STALL_STREAK_LIMIT        3
#define FAR_SPEED_STEP                15
#define FAR_SPEED_MAX                 255
// NEAR mode: once compassPulseDurMs is already maxed and still not making
// progress, ramp COMPASS_NEAR_SPEED instead of repeating useless pulses.
#define NEAR_STALL_AT_CAP_LIMIT       3
#define NEAR_SPEED_STEP               15
#define NEAR_SPEED_MAX                255
// Brief reverse pulse when fully stuck at max PWM in either mode — the one
// deliberate blocking exception in this otherwise non-blocking file, kept
// short and rare (only fires at max PWM with zero progress) so it shouldn't
// meaningfully disrupt CMD:STOP responsiveness.
#define ROCK_FREE_MS                  250UL

IST8310       ist8310;
bool          compassReady = false;
uint8_t       compassFailStreak            = 0;
uint16_t      compassConsecutiveTurnFails  = 0;

// ═══════════════════════════════════════════════════════════════════════════
//  WiFi / TCP
// ═══════════════════════════════════════════════════════════════════════════
const char* AP_SSID     = "WaterBot";
const char* AP_PASSWORD = "jetson";
const IPAddress AP_IP      (192, 168, 4, 1);
const IPAddress AP_GATEWAY (192, 168, 4, 1);
const IPAddress AP_SUBNET  (255, 255, 255, 0);
const uint16_t  TCP_PORT    = 4210;

WiFiServer tcpServer(TCP_PORT);
WiFiClient client;
String     cmdBuf = "";

// ═══════════════════════════════════════════════════════════════════════════
//  MOTION STATE TRACKING
// ═══════════════════════════════════════════════════════════════════════════
bool          isMoving      = false;
bool          isSettling    = false;
unsigned long motionEndTime = 0;
unsigned long settleEndTime = 0;

bool          isCompassTurning     = false;
float         compassTargetHeading = 0.0f;
unsigned long compassTurnTimeoutMs = 0;
unsigned long lastCompassPollMs    = 0;

// Last successfully-read compass heading, reported back to the Jetson
// alongside DONE (as "DONE HEADING=xx.x") so the Jetson can verify actual
// achieved headings (e.g. the 90 deg scan-dance edge check) instead of
// only seeing bare completion acks. NAN until a first good read happens.
float         lastCompassHeadingDeg = NAN;

// Direction chosen ONCE at the start of a turn, held fixed for its entire
// duration — never recomputed from live error. +1 = the direction that
// makes heading INCREASE (true clockwise), -1 = heading decreases.
int8_t        compassTurnDir       = 1;

bool          compassNearMode      = false;
bool          compassPulsing       = false;
bool          compassBraking       = false;
unsigned long compassPhaseEndMs    = 0;

// Adaptive NEAR-mode pulse duration: starts at COMPASS_PULSE_MS, grows (capped
// at COMPASS_PULSE_MS_MAX) whenever a pulse fails to make real progress
// against water drag, resets back to base once progress is seen again.
unsigned long compassPulseDurMs    = 0;
float         compassHeadingBeforePulse = 0.0f;

// Last ACCEPTED (plausible) heading during the current turn — used to reject
// glitch readings that imply an impossible jump. Reset to the start heading
// at the top of every doTurnCompass() call. NAN when no turn is in progress.
float         lastGoodTurnHeadingDeg = NAN;

// Oscillation detection for NEAR mode: if the pulse direction flips sign
// several cycles running, the hull is hunting back and forth past the
// target on momentum rather than just being slowed by drag.
int8_t        compassNearDirPrev        = 0;
uint8_t       compassOscillationStreak  = 0;

// ── FAR-mode stall detection (mirrors NEAR-mode's adaptive pulse approach) ──
// A bump can stall the hull mid-turn while the compass reading stays
// perfectly valid (it's a real heading, just not changing) — so this never
// trips the bad-read/glitch-rejection paths above. Only tracking actual
// progress between polls catches it.
float         far_headingAtLastPoll = NAN;   // heading at the previous FAR-mode poll
uint8_t       far_stallStreak       = 0;     // consecutive polls with no real progress
uint8_t       currentFarSpeed       = SPEED_TURN;  // adaptive — starts at SPEED_TURN, ramps on stall

// ── NEAR-mode stall escalation (past the existing pulse-duration cap) ──────
// compassPulseDurMs already grows on stalled progress, but caps at
// COMPASS_PULSE_MS_MAX — if a bump is bigger than that pulse can push
// through, it would otherwise just sit there pulsing uselessly forever.
uint8_t       currentNearSpeed      = COMPASS_NEAR_SPEED;  // adaptive — starts at base, ramps on stall
uint8_t       near_stallAtCapStreak = 0;     // consecutive maxed-out pulses with no progress

// Set anywhere a stall-escalation branch fires during this turn, reported
// back to the Jetson on DONE so it can distinguish a stall-recovered/failed
// turn from a clean one instead of both looking identical.
bool          turnHadStall = false;

// ── FAR-mode "reached" pending verification (see fix note at top of file) ──
// Set when a FAR-mode poll satisfies COMPASS_TURN_TOL_DEG directly, without
// the turn ever having slowed into NEAR mode. Motors are stopped immediately
// but the turn is not finalized until reachedVerifyEndMs passes and a fresh
// read confirms the heading actually held within tolerance.
bool          reachedPendingVerify  = false;
unsigned long reachedVerifyEndMs    = 0;

// ── Centre-heading correction (CMD:SETCENTRE / CMD:CMOVE) ───────────────────
// centreHeadingDeg: set by CMD:SETCENTRE (median-of-3, glitch-safe). NAN
// means no centre correction is configured — CMD:CMOVE falls back to a
// plain move with no heading check.
float         centreHeadingDeg     = NAN;
// CMD:CMOVE drives in CMOVE_SEGMENT_M chunks, checking heading against
// centreHeadingDeg between each and correcting if it drifted. This replaces
// the old one-shot pre-move check, since the actual drive is open-loop
// (I2C disabled during motor movement) and a motor imbalance can drift
// heading for the whole move otherwise.
#define CMOVE_SEGMENT_M      0.25f
bool          isSegmentedMove      = false;
float         segMoveRemainingM    = 0.0f;
float         segMoveTargetHeading = NAN;
// True while the correction CTURN between segments is in progress, so the
// settle-complete handler knows to recheck and drive the next segment
// instead of reporting DONE.
bool          segMoveCorrecting    = false;

// ═══════════════════════════════════════════════════════════════════════════
//  MOTOR PRIMITIVES
// ═══════════════════════════════════════════════════════════════════════════
void leftMotor(uint8_t speed, bool forward) {
    ledcWrite(PIN_ENA, speed);
    digitalWrite(PIN_IN1, forward ? HIGH : LOW);
    digitalWrite(PIN_IN2, forward ? LOW  : HIGH);
}

void rightMotor(uint8_t speed, bool forward) {
    ledcWrite(PIN_ENB, speed);
    digitalWrite(PIN_IN3, forward ? HIGH : LOW);
    digitalWrite(PIN_IN4, forward ? LOW  : HIGH);
}

void motorsStop() {
    ledcWrite(PIN_ENA, 0);
    ledcWrite(PIN_ENB, 0);
    digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
    digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

// Drives a compass turn in a FIXED direction, CORRECTED polarity:
// dir > 0 must make heading INCREASE (true clockwise) — confirmed backwards
// before this fix (commanding this branch was making heading DECREASE).
// dir < 0 makes heading decrease (counter-clockwise).
void driveCompassDir(int8_t dir, uint8_t speed) {
    if (dir > 0) {
        // CLOCKWISE (heading increases) — swapped from the previous mapping.
        leftMotor (speed, false);
        rightMotor(speed, true);
    } else {
        // COUNTER-CLOCKWISE (heading decreases) — swapped from the previous mapping.
        leftMotor (speed, true);
        rightMotor(speed, false);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  COMPASS HELPERS
// ═══════════════════════════════════════════════════════════════════════════

void recoverI2C() {
    static unsigned long lastRecoveryMs = 0;
    unsigned long now = millis();
    if (now - lastRecoveryMs < 2000UL) return;
    lastRecoveryMs = now;

    BLOGln("[I2C] Bus lockup or timeout detected! Running recovery...");

    Wire.end();
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delay(5);

    if (digitalRead(PIN_I2C_SDA) == LOW) {
        BLOGln("[I2C] SDA stuck LOW. Generating SCL clock pulses...");
        pinMode(PIN_I2C_SCL, OUTPUT);
        for (int i = 0; i < 9; i++) {
            digitalWrite(PIN_I2C_SCL, LOW);
            delayMicroseconds(5);
            digitalWrite(PIN_I2C_SCL, HIGH);
            delayMicroseconds(5);
        }
        pinMode(PIN_I2C_SDA, INPUT_PULLUP);
        pinMode(PIN_I2C_SCL, INPUT_PULLUP);
        delay(5);
        if (digitalRead(PIN_I2C_SDA) == HIGH) {
            BLOGln("[I2C] SDA released!");
        } else {
            BLOGln("[I2C] SDA still stuck LOW after toggling SCL!");
        }
    } else {
        BLOGln("[I2C] SDA is HIGH. Resetting I2C bus driver.");
    }

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);
#if defined(ARDUINO_ARCH_ESP32)
    Wire.setTimeOut(50);
#endif

    if (ist8310.setup(&Wire, &Serial)) {
        ist8310.set_flip_x_y(false);
        ist8310.set_declination_offset_radians(0.0);
        BLOGln("[Compass] IST8310 re-setup complete");
    } else {
        BLOGln("[Compass] IST8310 re-setup FAILED");
    }
}

// ── Motor noise I2C isolation ──────────────────────────────────────────────
bool i2cIsDisabled = false;

void disableI2C() {
    if (compassReady && !i2cIsDisabled) {
        Wire.end();
        pinMode(PIN_I2C_SDA, INPUT);
        pinMode(PIN_I2C_SCL, INPUT);
        i2cIsDisabled = true;
        BLOGln("[I2C] Disabled during motor movement to prevent EMI lockup");
    }
}

void enableI2C() {
    if (compassReady && i2cIsDisabled) {
        BLOGln("[I2C] Re-enabling bus after movement...");
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.setClock(100000);
#if defined(ARDUINO_ARCH_ESP32)
        Wire.setTimeOut(50);
#endif
        if (ist8310.setup(&Wire, &Serial)) {
            ist8310.set_flip_x_y(false);
            ist8310.set_declination_offset_radians(0.0);
            BLOGln("[I2C] Compass re-setup complete");
        } else {
            BLOGln("[I2C] Compass re-setup FAILED");
        }
        i2cIsDisabled = false;
    }
}

// Returns heading in [0, 360). Returns -1.0 on failure. Single-attempt,
// non-blocking — a missed DRDY window just skips this poll cycle.
float readHeadingDeg() {
    if (!compassReady || i2cIsDisabled) return -1.0f;
    if (!ist8310.update()) return -1.0f;

    float h = ist8310.get_heading_degrees();
    if (isnan(h) || isinf(h)) {
        BLOGln("[Compass] ERROR: Heading is NaN or Inf!");
        return -1.0f;
    }
    h = fmod(h, 360.0f);
    if (h < 0.0f) h += 360.0f;
    BLOGf("[Compass] heading=%.2f deg\r\n", h);
    return h;
}

float headingDiffDeg(float from, float to) {
    if (isnan(from) || isinf(from) || isnan(to) || isinf(to)) return 0.0f;
    float d = to - from;
    d = fmod(d, 360.0f);
    if (d > 180.0f)   d -= 360.0f;
    if (d <= -180.0f) d += 360.0f;
    return d;
}

// ═══════════════════════════════════════════════════════════════════════════
//  TCP HELPERS
// ═══════════════════════════════════════════════════════════════════════════
void sendLine(const char* msg) {
    if (client && client.connected()) {
        client.print(msg);
        client.print('\n');
        client.flush();
    }
    BLOGf("[->Jetson] %s\r\n", msg);
}

// ═══════════════════════════════════════════════════════════════════════════
//  NON-BLOCKING MOTION COMMANDS
// ═══════════════════════════════════════════════════════════════════════════
void cancelMotionSilent() {
    motorsStop();
    isMoving         = false;
    isSettling       = false;
    isCompassTurning = false;
    compassNearMode  = false;
    compassPulsing   = false;
    compassBraking   = false;
    reachedPendingVerify = false;
    isSegmentedMove  = false;
    segMoveCorrecting = false;
}

void doTurn(float degrees) {
    if (degrees == 0.0f) { sendLine("DONE"); return; }
    if (isMoving || isSettling || isCompassTurning) cancelMotionSilent();
    turnHadStall = false;   // this is a plain timed turn, not a compass turn

    unsigned long durMs = (unsigned long)((fabs(degrees) / TURN_DEG_PER_SEC) * 1000.0f);
    BLOGf("[TURN] %.1f deg  duration: %lu ms\r\n", degrees, durMs);

    disableI2C();

    if (degrees > 0) {
        leftMotor (SPEED_TURN, true);
        rightMotor(SPEED_TURN, false);
    } else {
        leftMotor (SPEED_TURN, false);
        rightMotor(SPEED_TURN, true);
    }

    motionEndTime = millis() + durMs;
    isMoving      = true;
}

// Compass-verified closed-loop turn. Direction is picked HERE, once, from
// the sign of deltaDeg — deltaDeg > 0 means "heading should increase" and
// is now correctly mapped via driveCompassDir() (see polarity fix note at
// top of file). Direction is never changed again for the rest of this turn.
// Reads the compass 3x (25ms apart) and returns the median — same
// noise-rejection approach doTurnCompass() uses for its start-heading read.
// Returns -1.0 if all 3 reads failed.
float readHeadingMedian(int* goodCountOut = nullptr) {
    float readings[3] = { -1.0f, -1.0f, -1.0f };
    int   goodCount    = 0;
    for (int i = 0; i < 3; i++) {
        delay(25);
        float r = readHeadingDeg();
        if (r >= 0.0f) { readings[goodCount++] = r; }
    }
    if (goodCountOut) *goodCountOut = goodCount;
    if (goodCount == 0) return -1.0f;
    float med = readings[0];
    if (goodCount >= 2) {
        for (int i = 0; i < goodCount - 1; i++)
            for (int j = i + 1; j < goodCount; j++)
                if (readings[j] < readings[i]) { float t = readings[i]; readings[i] = readings[j]; readings[j] = t; }
        med = readings[goodCount / 2];
    }
    return med;
}

void doTurnCompass(float deltaDeg) {
    if (deltaDeg == 0.0f) { sendLine("DONE"); return; }
    if (isMoving || isSettling || isCompassTurning) cancelMotionSilent();

    int   goodCount    = 0;
    float startHeading = readHeadingMedian(&goodCount);
    if (goodCount == 0) {
        BLOGln("[CTURN] All 3 start-heading reads failed — running I2C recovery "
               "then falling back to timed turn");
        recoverI2C();
        doTurn(deltaDeg);
        return;
    }

    compassTargetHeading = startHeading + deltaDeg;
    if (isnan(compassTargetHeading) || isinf(compassTargetHeading)) {
        BLOGln("[CTURN] ERROR: targetHeading is NaN or Inf! Using start heading.");
        compassTargetHeading = startHeading;
    }
    compassTargetHeading = fmod(compassTargetHeading, 360.0f);
    if (compassTargetHeading < 0.0f) compassTargetHeading += 360.0f;

    BLOGf("[CTURN] start=%.1f  delta=%.1f  target=%.1f  goodReads=%d\r\n",
          startHeading, deltaDeg, compassTargetHeading, goodCount);

    // Direction chosen ONCE here, correctly mapped, held fixed for the whole turn.
    compassTurnDir = (deltaDeg > 0) ? 1 : -1;
    currentFarSpeed = SPEED_TURN;   // reset adaptive FAR speed for this turn
    driveCompassDir(compassTurnDir, currentFarSpeed);

    isCompassTurning     = true;
    compassNearMode      = false;
    compassPulsing       = false;
    compassBraking       = false;
    reachedPendingVerify = false;
    compassTurnTimeoutMs = millis() + COMPASS_TURN_MAX_MS;
    lastCompassPollMs    = 0;
    compassFailStreak    = 0;
    lastGoodTurnHeadingDeg = startHeading;   // seed plausibility check for this turn
    far_headingAtLastPoll = NAN;
    far_stallStreak        = 0;
    currentNearSpeed       = COMPASS_NEAR_SPEED;
    near_stallAtCapStreak  = 0;
    turnHadStall            = false;
}

void doMove(float metres) {
    if (metres == 0.0f) { sendLine("DONE"); return; }
    if (isMoving || isSettling || isCompassTurning) cancelMotionSilent();
    turnHadStall = false;   // MOVE has no stall detection yet (open-loop) — separate fix needed

    bool forward = (metres > 0.0f);
    unsigned long durMs = (unsigned long)((fabs(metres) / MOVE_M_PER_SEC) * 1000.0f);
    BLOGf("[MOVE] %.2f m  %s  duration: %lu ms\r\n",
          fabs(metres), forward ? "forward" : "backward", durMs);

    disableI2C();

    leftMotor (SPEED_MOVE, forward);
    rightMotor(SPEED_MOVE, forward);

    motionEndTime = millis() + durMs;
    isMoving      = true;
}

// Segmented, compass-corrected move: drives the distance in CMOVE_SEGMENT_M
// chunks. Before each chunk, checks heading against centreHeadingDeg and
// runs a correction turn if it drifted beyond CENTRE_CORRECT_TOL_DEG. Falls
// back to a single plain doMove() if no centre is set or compass is down.
void driveNextMoveSegment() {
    float segDist = fabs(segMoveRemainingM) > CMOVE_SEGMENT_M
                        ? CMOVE_SEGMENT_M : fabs(segMoveRemainingM);
    if (segMoveRemainingM < 0.0f) segDist = -segDist;
    segMoveRemainingM -= segDist;
    doMove(segDist);
}

void startNextMoveSegment() {
    int   goodCount = 0;
    float cur       = readHeadingMedian(&goodCount);
    if (goodCount > 0) {
        float err = headingDiffDeg(cur, segMoveTargetHeading);
        if (fabs(err) > CENTRE_CORRECT_TOL_DEG) {
            BLOGf("[CMOVE] Mid-move drift %.1fdeg -- correcting\r\n", err);
            segMoveCorrecting = true;
            doTurnCompass(err);
            return;
        }
    } else {
        BLOGln("[CMOVE] Heading read failed -- continuing without correction");
    }
    segMoveCorrecting = false;
    driveNextMoveSegment();
}

void doCMove(float metres) {
    if (metres == 0.0f) { sendLine("DONE"); return; }
    if (isnan(centreHeadingDeg) || !compassReady) {
        doMove(metres);
        return;
    }
    segMoveTargetHeading = centreHeadingDeg;
    segMoveRemainingM    = metres;
    isSegmentedMove      = true;
    segMoveCorrecting    = false;
    BLOGf("[CMOVE] %.2fm in %.2fm segments, holding %.1fdeg\r\n",
          metres, CMOVE_SEGMENT_M, segMoveTargetHeading);
    startNextMoveSegment();
}

void doStop() {
    bool wasBusy = isMoving || isSettling || isCompassTurning || isSegmentedMove;
    motorsStop();
    isMoving         = false;
    isSettling       = false;
    isCompassTurning = false;
    compassNearMode  = false;
    compassPulsing   = false;
    compassBraking   = false;
    reachedPendingVerify = false;
    isSegmentedMove  = false;
    segMoveCorrecting = false;
    if (wasBusy) {
        enableI2C();
        sendLine("DONE");
        BLOGln("[STOP] Motion interrupted.");
    } else {
        BLOGln("[STOP] Already stopped.");
    }
}

// Passive heading query — does NOT move the motors. Replies "HEADING:123.4"
// on success, or "HEADING:ERR" if all 3 compass reads failed, so the Jetson
// can note the center heading before a scan-dance and verify the achieved
// heading after the 3x30deg steps (should be centre+-90).
void doGetHeading() {
    if (isMoving || isSettling || isCompassTurning) cancelMotionSilent();
    int   goodCount = 0;
    float h         = readHeadingMedian(&goodCount);
    if (goodCount == 0) {
        BLOGln("[GETHEADING] All 3 reads failed");
        sendLine("HEADING:ERR");
        return;
    }
    char buf[24];
    snprintf(buf, sizeof(buf), "HEADING:%.2f", h);
    BLOGf("[GETHEADING] %.2f deg\r\n", h);
    sendLine(buf);
}

// ═══════════════════════════════════════════════════════════════════════════
//  COMMAND PARSER
// ═══════════════════════════════════════════════════════════════════════════
void handleCommand(String line) {
    line.trim();
    if (line.length() == 0) return;
    BLOGf("[CMD] %s\r\n", line.c_str());

    if (line.startsWith("CMD:TURN ")) {
        float deg = line.substring(9).toFloat();
        doTurn(deg);
    } else if (line.startsWith("CMD:CTURN ")) {
        float deg = line.substring(10).toFloat();
        doTurnCompass(deg);
    } else if (line.startsWith("CMD:MOVE ")) {
        float dist = line.substring(9).toFloat();
        doMove(dist);
    } else if (line.startsWith("CMD:CMOVE ")) {
        float dist = line.substring(10).toFloat();
        doCMove(dist);
    } else if (line == "CMD:SETCENTRE") {
        // Read the compass now (median-of-3, glitch-safe) and store as the
        // reference heading for CMD:CMOVE corrections. Always re-reads rather
        // than trusting any Jetson-supplied value, so a glitch on that side
        // cannot corrupt the stored centre. Replies HEADING:xx.xx on success
        // or HEADING:ERR if all 3 reads failed.
        if (isMoving || isSettling || isCompassTurning) cancelMotionSilent();
        int   goodCount = 0;
        float h         = readHeadingMedian(&goodCount);
        if (goodCount == 0) {
            BLOGln("[SETCENTRE] All 3 reads failed — centre heading NOT updated");
            sendLine("HEADING:ERR");
        } else {
            centreHeadingDeg = h;
            char buf[32];
            snprintf(buf, sizeof(buf), "HEADING:%.2f", h);
            BLOGf("[SETCENTRE] Centre heading set to %.2fdeg\r\n", h);
            sendLine(buf);
        }
    } else if (line == "CMD:STOP") {
        doStop();
    } else if (line == "CMD:GETHEADING") {
        doGetHeading();
    } else {
        BLOGf("[WARN] Unknown: %s\r\n", line.c_str());
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    BT.begin("WaterBot-BT");
    delay(100);

    BLOGln("\n=== WaterBot ESP32 ===");

    pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);

    ledcAttach(PIN_ENA, LEDC_FREQ, LEDC_BITS);
    ledcAttach(PIN_ENB, LEDC_FREQ, LEDC_BITS);

    motorsStop();
    BLOGln("[Motor] L298N ready");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setClock(100000);
#if defined(ARDUINO_ARCH_ESP32)
    Wire.setTimeOut(50);
#endif

    compassReady = ist8310.setup(&Wire, &Serial);
    if (!compassReady) {
        BLOGln("[Compass] IST8310 setup FAILED — CMD:CTURN will fall back to timed turns");
    } else {
        ist8310.set_flip_x_y(false);
        ist8310.set_declination_offset_radians(0.0);

        bool verified = false;
        for (int i = 0; i < 5; i++) {
            delay(20);
            if (readHeadingDeg() >= 0.0f) { verified = true; break; }
        }
        if (!verified) {
            BLOGln("[Compass] IST8310 setup OK but no valid heading returned — "
                   "disabling, CMD:CTURN will use timed fallback (check wiring/address)");
            compassReady = false;
        } else {
            BLOGln("[Compass] IST8310 ready and verified");
        }
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
    if (strlen(AP_PASSWORD) > 0)
        WiFi.softAP(AP_SSID, AP_PASSWORD);
    else
        WiFi.softAP(AP_SSID);

    BLOGf("[WiFi] AP: %s  IP: %s\r\n",
          AP_SSID, WiFi.softAPIP().toString().c_str());

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    BLOGf("[TCP]  Listening on port %d\r\n", TCP_PORT);
    BLOGln("[BT]   WaterBot-BT ready — pair & connect with a BT Serial terminal");
}

// ═══════════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    // ── 1. Non-blocking motion-complete check (timed MOVE / TURN) ──────────
    if (isMoving && millis() >= motionEndTime) {
        motorsStop();
        isMoving   = false;
        isSettling = true;
        settleEndTime = millis() + STOP_SETTLE_MS;
        enableI2C();
    }

    // ── 1b. Non-blocking compass-turn-complete check (CTURN) ───────────────
    if (isCompassTurning) {
        unsigned long nowMs = millis();

        if (reachedPendingVerify) {
            // Motors are already stopped. Waiting out the extra coast-settle
            // window before trusting a "reached" event that arrived directly
            // from FAR mode (see fix note at top of file).
            if (nowMs >= reachedVerifyEndMs) {
                float h2 = readHeadingDeg();

                if (h2 >= 0.0f && !isnan(lastGoodTurnHeadingDeg)) {
                    float jump = fabs(headingDiffDeg(lastGoodTurnHeadingDeg, h2));
                    if (jump > COMPASS_MAX_PLAUSIBLE_JUMP_DEG) {
                        BLOGf("[CTURN] verify-coast implausible jump %.1fdeg "
                              "(last-good=%.1f new=%.1f) — rejecting, retrying\r\n",
                              jump, lastGoodTurnHeadingDeg, h2);
                        h2 = -1.0f;
                    }
                }

                if (h2 >= 0.0f) {
                    lastGoodTurnHeadingDeg = h2;
                    lastCompassHeadingDeg  = h2;
                    float err2 = headingDiffDeg(h2, compassTargetHeading);
                    BLOGf("[CTURN] verify-coast heading=%.1f target=%.1f err=%.1f\r\n",
                          h2, compassTargetHeading, err2);

                    if (fabs(err2) <= COMPASS_TURN_TOL_DEG) {
                        // Genuinely settled within tolerance — finalize.
                        reachedPendingVerify = false;
                        isCompassTurning     = false;
                        compassNearMode      = false;
                        compassPulsing       = false;
                        compassBraking       = false;
                        isSettling           = true;
                        settleEndTime        = nowMs + STOP_SETTLE_MS;
                        compassConsecutiveTurnFails = 0;
                        BLOGf("[CTURN] Verified reached heading=%.1f (target=%.1f)\r\n",
                              h2, compassTargetHeading);
                    } else {
                        // Coasted past tolerance — resume closed-loop approach
                        // from where it actually is now instead of reporting
                        // a false DONE.
                        BLOGf("[CTURN] FAR-mode reach was premature (coasted to "
                              "err=%.1f) — resuming approach\r\n", err2);
                        reachedPendingVerify = false;
                        if (fabs(err2) <= COMPASS_NEAR_THRESHOLD_DEG) {
                            compassNearMode           = true;
                            compassBraking            = true;
                            compassPhaseEndMs         = nowMs + COMPASS_BRAKE_MS;
                            compassPulseDurMs         = COMPASS_PULSE_MS;
                            compassHeadingBeforePulse = h2;
                            compassNearDirPrev        = 0;
                            compassOscillationStreak  = 0;
                        } else {
                            compassTurnDir = (err2 > 0.0f) ? 1 : -1;
                            driveCompassDir(compassTurnDir, currentFarSpeed);
                            far_headingAtLastPoll = h2;
                            far_stallStreak       = 0;
                            lastCompassPollMs     = nowMs;
                        }
                    }
                } else {
                    // Bad read during verify — give it a short additional
                    // window and try again rather than guessing blind.
                    reachedVerifyEndMs = nowMs + 100UL;
                }
            }
            // Skip the normal poll/finalize logic below entirely this pass.
        } else {

        if (compassNearMode && compassPulsing) {
            if (nowMs >= compassPhaseEndMs) {
                motorsStop();
                compassPulsing    = false;
                compassBraking    = true;
                compassPhaseEndMs = nowMs + COMPASS_BRAKE_MS;
            }
        } else if (compassNearMode && compassBraking) {
            if (nowMs >= compassPhaseEndMs) {
                compassBraking = false;
            }
        }

        bool readyToCheck = compassNearMode
                                ? (!compassPulsing && !compassBraking)
                                : (nowMs - lastCompassPollMs >= COMPASS_POLL_MIN_MS);

        if (readyToCheck) {
            lastCompassPollMs = nowMs;
            float h = readHeadingDeg();
            bool reached = false;
            bool failed  = false;

            // Reject a reading that implies an impossible jump since the last
            // ACCEPTED reading (e.g. a lone "0.00" glitch from I2C/EMI noise
            // while the boat was actually still near its previous heading).
            // Without this check one bad-but-in-range sample makes the
            // firmware believe it's ~100+ deg further from target than it
            // really is, and FAR mode will happily spin that whole extra
            // distance chasing a phantom error.
            if (h >= 0.0f && !isnan(lastGoodTurnHeadingDeg)) {
                float jump = fabs(headingDiffDeg(lastGoodTurnHeadingDeg, h));
                if (jump > COMPASS_MAX_PLAUSIBLE_JUMP_DEG) {
                    BLOGf("[CTURN] Implausible jump %.1fdeg (last-good=%.1f "
                          "new=%.1f) — rejecting as glitch, treating as bad read\r\n",
                          jump, lastGoodTurnHeadingDeg, h);
                    h = -1.0f;   // fall through to the failed-read branch below
                }
            }
            float err = 0.0f;

            if (h >= 0.0f) {
                compassFailStreak = 0;
                lastCompassHeadingDeg  = h;
                lastGoodTurnHeadingDeg = h;
                err = headingDiffDeg(h, compassTargetHeading);
                BLOGf("[CTURN] heading=%.1f  target=%.1f  err=%.1f  mode=%s\r\n",
                      h, compassTargetHeading, err, compassNearMode ? "NEAR" : "FAR");

                if (fabs(err) <= COMPASS_TURN_TOL_DEG) {
                    reached = true;
                } else if (!compassNearMode && fabs(err) <= COMPASS_NEAR_THRESHOLD_DEG) {
                    // Crossing into NEAR mode: stop completely, settle, THEN
                    // pulse. compassTurnDir is NEVER recomputed from err.
                    motorsStop();
                    compassNearMode           = true;
                    compassBraking            = true;
                    compassPhaseEndMs         = nowMs + COMPASS_BRAKE_MS;
                    compassPulseDurMs         = COMPASS_PULSE_MS;   // reset adaptive duration
                    compassHeadingBeforePulse = h;
                    compassNearDirPrev        = 0;
                    compassOscillationStreak  = 0;
                } else if (compassNearMode) {
                    int8_t nearDir = (err > 0.0f) ? 1 : -1;

                    // Recompute direction from the LIVE sign of err (not the
                    // stale compassTurnDir) — FAR mode's full-speed spin can
                    // carry the hull past the target on momentum before the
                    // stop takes effect, and once that happens the correct
                    // pulse direction is the OPPOSITE of the original turn
                    // direction. This is safe now that driveCompassDir()'s
                    // polarity itself is fixed; it was only unsafe under the
                    // old backwards-polarity bug.
                    bool dirFlipped = (compassNearDirPrev != 0 && nearDir != compassNearDirPrev);
                    compassOscillationStreak = dirFlipped ? (compassOscillationStreak + 1) : 0;

                    float progress = fabs(headingDiffDeg(compassHeadingBeforePulse, h));
                    if (compassOscillationStreak >= COMPASS_OSCILLATION_LIMIT) {
                        // Direction keeps flipping — the hull has enough
                        // momentum that pulses are overshooting the target
                        // BOTH ways (hunting), not just being soaked up by
                        // drag. Growing the pulse further would amplify the
                        // overshoot, not fix it — force a short, gentle tap
                        // instead so it can settle rather than hunt forever.
                        compassPulseDurMs = COMPASS_PULSE_MS_FINE;
                        BLOGf("[CTURN] NEAR oscillating (streak=%u) — forcing "
                              "fine pulse %lums\r\n",
                              (unsigned)compassOscillationStreak, compassPulseDurMs);
                    } else if (compassPulseDurMs > 0) {
                        // Check whether the LAST pulse actually made progress.
                        // If the hull barely moved (water drag/slosh soaking
                        // up the pulse), lengthen the next pulse (capped); if
                        // it made good progress, drop back to the base
                        // duration so we don't overshoot once drag improves.
                        if (progress < COMPASS_PULSE_PROGRESS_MIN) {
                            if (compassPulseDurMs < COMPASS_PULSE_MS_MAX) {
                                compassPulseDurMs = min(compassPulseDurMs + 100UL, COMPASS_PULSE_MS_MAX);
                                BLOGf("[CTURN] NEAR pulse stalled (progress=%.1f) — "
                                      "lengthening to %lums\r\n", progress, compassPulseDurMs);
                                near_stallAtCapStreak = 0;
                            } else {
                                // Already at max pulse duration and still not
                                // moving — the bump is too strong for
                                // COMPASS_NEAR_SPEED. Ramp PWM instead of
                                // repeating useless maxed-out pulses forever.
                                near_stallAtCapStreak++;
                                turnHadStall = true;
                                if (near_stallAtCapStreak >= NEAR_STALL_AT_CAP_LIMIT) {
                                    if (currentNearSpeed < NEAR_SPEED_MAX) {
                                        currentNearSpeed = min((int)currentNearSpeed + NEAR_SPEED_STEP, (int)NEAR_SPEED_MAX);
                                        BLOGf("[CTURN] NEAR stalled at max pulse duration — "
                                              "ramping speed to %u\r\n", (unsigned)currentNearSpeed);
                                    } else {
                                        BLOGln("[CTURN] NEAR stalled at max speed+duration — rocking free");
                                        driveCompassDir((int8_t)(-nearDir), currentNearSpeed);
                                        delay(ROCK_FREE_MS);
                                    }
                                    near_stallAtCapStreak = 0;
                                }
                            }
                        } else {
                            compassPulseDurMs     = COMPASS_PULSE_MS;
                            currentNearSpeed       = COMPASS_NEAR_SPEED;   // decay back to base once progress resumes
                            near_stallAtCapStreak  = 0;
                        }
                    } else {
                        compassPulseDurMs = COMPASS_PULSE_MS;
                    }

                    compassNearDirPrev        = nearDir;
                    compassHeadingBeforePulse = h;
                    driveCompassDir(nearDir, currentNearSpeed);
                    compassPulsing    = true;
                    compassPhaseEndMs = nowMs + compassPulseDurMs;
                } else {
                    // Still FAR mode — check whether the hull is actually
                    // making progress, not just spinning against a bump.
                    // A stall here is invisible to the glitch-rejection and
                    // bad-read paths above, since the compass reading stays
                    // perfectly valid (a real heading, just not changing).
                    if (!isnan(far_headingAtLastPoll)) {
                        float progress = fabs(headingDiffDeg(far_headingAtLastPoll, h));
                        if (progress < FAR_STALL_PROGRESS_MIN_DEG) {
                            far_stallStreak++;
                            BLOGf("[CTURN] FAR stall check: progress=%.1fdeg  streak=%u/%u\r\n",
                                  progress, (unsigned)far_stallStreak, (unsigned)FAR_STALL_STREAK_LIMIT);
                            if (far_stallStreak >= FAR_STALL_STREAK_LIMIT) {
                                turnHadStall = true;
                                if (currentFarSpeed < FAR_SPEED_MAX) {
                                    currentFarSpeed = min((int)currentFarSpeed + FAR_SPEED_STEP, (int)FAR_SPEED_MAX);
                                    BLOGf("[CTURN] FAR stalled — ramping speed to %u\r\n",
                                          (unsigned)currentFarSpeed);
                                    driveCompassDir(compassTurnDir, currentFarSpeed);
                                } else {
                                    BLOGln("[CTURN] FAR stalled at max PWM — rocking free");
                                    driveCompassDir((int8_t)(-compassTurnDir), currentFarSpeed);
                                    delay(ROCK_FREE_MS);
                                    driveCompassDir(compassTurnDir, currentFarSpeed);
                                }
                                far_stallStreak = 0;
                            }
                        } else {
                            far_stallStreak = 0;
                            // Progress resumed (bump cleared) — decay speed
                            // back toward the calibrated base rather than
                            // staying pinned at the ramped-up value forever.
                            if (currentFarSpeed > SPEED_TURN) {
                                currentFarSpeed = max((int)SPEED_TURN, (int)currentFarSpeed - FAR_SPEED_STEP);
                                driveCompassDir(compassTurnDir, currentFarSpeed);
                            }
                        }
                    }
                    far_headingAtLastPoll = h;
                }
            } else {
                compassFailStreak++;
                BLOGf("[CTURN] bad read streak=%u/%u\r\n",
                      (unsigned)compassFailStreak,
                      (unsigned)COMPASS_FAIL_ABORT_STREAK);
                failed = (compassFailStreak >= COMPASS_FAIL_ABORT_STREAK);
            }

            if (reached) {
                // Stop now, but don't finalize until a fresh read after an
                // extra coast-settle window confirms the heading actually
                // held (see fix note at top of file).
                motorsStop();
                reachedPendingVerify = true;
                reachedVerifyEndMs   = nowMs + COMPASS_REACHED_VERIFY_MS;
                BLOGf("[CTURN] Tolerance hit (err=%.1f mode=%s) — verifying "
                      "after %lums coast-settle before accepting\r\n",
                      err, compassNearMode ? "NEAR" : "FAR",
                      (unsigned long)COMPASS_REACHED_VERIFY_MS);
            } else if (nowMs >= compassTurnTimeoutMs || failed) {
                motorsStop();
                isCompassTurning = false;
                compassNearMode  = false;
                compassPulsing   = false;
                compassBraking   = false;
                isSettling       = true;
                settleEndTime    = nowMs + STOP_SETTLE_MS;

                if (failed) {
                    compassConsecutiveTurnFails++;
                    BLOGf("[CTURN] Compass read failures: %u/%u consecutive bad turns\r\n",
                          (unsigned)compassConsecutiveTurnFails,
                          (unsigned)COMPASS_DISABLE_AFTER_N_FAILS);
                    recoverI2C();
                    if (compassConsecutiveTurnFails >= COMPASS_DISABLE_AFTER_N_FAILS) {
                        compassReady = false;
                        BLOGln("[Compass] Disabling compass for remainder of session "
                               "- CMD:CTURN will use timed fallback. Check NEO3 wiring "
                               "(SDA/SCL/GND/3.3V) and I2C address.");
                    }
                } else {
                    BLOGf("[CTURN] Timeout (%.1fs) waiting for target=%.1f last=%.1f, stopping\r\n",
                          COMPASS_TURN_MAX_MS / 1000.0f, compassTargetHeading, h);
                }
            }
        }
        }
    }

    // ── 2. Non-blocking settle-complete check ──────────────────────────────
    if (isSettling && millis() >= settleEndTime) {
        isSettling = false;
        // Take one fresh read right now rather than trusting whatever
        // lastCompassHeadingDeg was at the moment "reached"/"timeout" fired —
        // the hull can keep coasting on momentum for a bit after the motors
        // stop (especially after a NEAR-mode timeout), so the true settled
        // heading can differ from what was true a settle-period ago.
        float freshH = readHeadingDeg();
        if (freshH >= 0.0f && !isnan(lastCompassHeadingDeg)) {
            // Same plausibility check used for in-turn polls — the first
            // read right after enableI2C()'s re-setup is frequently a
            // glitchy 0.00 (I2C not fully settled yet). Without this check
            // that glitch gets reported to the Jetson as a real heading.
            float jump = fabs(headingDiffDeg(lastCompassHeadingDeg, freshH));
            if (jump > COMPASS_MAX_PLAUSIBLE_JUMP_DEG) {
                BLOGf("[SETTLE] Implausible final read (last=%.1f new=%.1f "
                      "jump=%.1f) — keeping last known heading\r\n",
                      lastCompassHeadingDeg, freshH, jump);
                freshH = -1.0f;
            }
        }
        if (freshH >= 0.0f) lastCompassHeadingDeg = freshH;

        // If a segmented CMD:CMOVE is in progress, continue it instead of
        // sending DONE: either resume driving after a mid-move correction
        // turn just settled, or start the next segment (which rechecks
        // heading and corrects again if needed).
        if (isSegmentedMove) {
            if (segMoveCorrecting) {
                segMoveCorrecting = false;
                startNextMoveSegment();
                return;
            }
            if (fabs(segMoveRemainingM) >= 0.001f) {
                startNextMoveSegment();
                return;
            }
            isSegmentedMove = false;
            // Fall through — final segment done, report DONE below.
        }

        if (!isnan(lastCompassHeadingDeg)) {
            char doneBuf[56];
            snprintf(doneBuf, sizeof(doneBuf), "DONE HEADING=%.1f%s",
                     lastCompassHeadingDeg, turnHadStall ? " STALLED=1" : "");
            sendLine(doneBuf);
        } else {
            sendLine("DONE");
        }
    }

    // ── 3. Manage client connections ───────────────────────────────────────
    if (!client || !client.connected()) {
        WiFiClient incoming = tcpServer.available();
        if (incoming) {
            client = incoming;
            client.setNoDelay(true);
            cmdBuf = "";
            BLOGf("[TCP] Client: %s\r\n",
                  client.remoteIP().toString().c_str());
            sendLine("HELLO");
        }
    }

    // ── 4. Read all available bytes in one shot ────────────────────────────
    if (client && client.connected() && client.available()) {
        String incoming = "";
        while (client.available()) {
            char ch = (char)client.read();
            if (ch != '\r') incoming += ch;
        }
        if (incoming.length() + cmdBuf.length() > 512) {
            BLOGln("[WARN] RX overflow — clearing buffer");
            cmdBuf   = "";
            incoming = "";
        }

        int stopIdx = incoming.indexOf("CMD:STOP");
        if (stopIdx != -1) {
            doStop();
            int nlIdx = incoming.indexOf('\n', stopIdx);
            incoming  = (nlIdx != -1) ? incoming.substring(nlIdx + 1) : "";
            cmdBuf    = "";
        }

        cmdBuf += incoming;
        while (true) {
            int nlIdx = cmdBuf.indexOf('\n');
            if (nlIdx == -1) break;
            String line = cmdBuf.substring(0, nlIdx);
            cmdBuf      = cmdBuf.substring(nlIdx + 1);
            line.trim();
            if (line.length() > 0) handleCommand(line);
        }
    }

    // Yield to RTOS and prevent CPU core starvation/overheating under motor EMI.
    delay(15);
}