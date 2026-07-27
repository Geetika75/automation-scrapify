/*
 * esp32_waterbot.ino
 * Water-surface waste-collection bot — ESP32-WROOM-32 firmware
 *
 * Fix history:
 * - Polarity: CMD:CTURN deltaDeg > 0 now genuinely increases heading
 *   (driveCompassDir was mapped backwards).
 * - Turn direction is chosen once at the start of a compass turn and held
 *   for the FAR (full-speed) phase; NEAR-mode pulses recompute direction
 *   from the live error each cycle, since FAR-mode momentum can carry the
 *   hull past target before the stop takes effect.
 * - "Reached" is provisional: any reach event (FAR or NEAR) stops the
 *   motors immediately but isn't finalized until an extra coast-settle
 *   window (COMPASS_REACHED_VERIFY_MS) confirms the heading actually held.
 *   Momentum was carrying the hull a few to twenty-plus degrees past
 *   target before the old fixed 150ms settle read it.
 * - The one-shot compass read at settle-complete now goes through the same
 *   plausibility check as in-turn polls, since the first read right after
 *   enableI2C()'s re-setup is often a glitchy 0.00.
 * - CMD:CMOVE now steers continuously: I2C stays enabled through the whole
 *   move, and a proportional differential correction (CMOVE_STEER_KP,
 *   capped at CMOVE_STEER_MAX_CORR) keeps the boat on the held centre
 *   heading the entire time. Replaces an earlier stop-every-0.25m-and-turn
 *   approach that let drift build up between checkpoints and looked like
 *   the boat veering off course before each correction.
 *
 * CUAV NEO 3 (IST8310 compass) wiring: I2C only (SDA/SCL), GPS UART unused.
 *   NEO 3 SDA -> ESP32 GPIO21   NEO 3 SCL -> ESP32 GPIO22
 *   NEO 3 VCC -> 3.3V           NEO 3 GND -> GND
 *   Requires the IST8310 Arduino library (Wire-based).
 *
 * Protocol: CMD:TURN (timed, no compass), CMD:CTURN (compass-verified),
 * CMD:MOVE (timed), CMD:CMOVE (compass-steered), CMD:SETCENTRE,
 * CMD:GETHEADING, CMD:STOP. Replies: DONE, DONE HEADING=xx.x[ STALLED=1],
 * HEADING:xx.x, HELLO.
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
#define COMPASS_POLL_MIN_MS           80UL
#define COMPASS_FAIL_ABORT_STREAK     15
#define COMPASS_DISABLE_AFTER_N_FAILS 10

// NEAR-target approach: short stop-then-pulse cycles once within this
// threshold, replacing the earlier reactive direction-flip.
#define COMPASS_NEAR_THRESHOLD_DEG    20.0f
#define COMPASS_NEAR_SPEED            165
#define COMPASS_PULSE_MS              90UL
#define COMPASS_PULSE_MS_MAX          400UL  // cap so a stalled pulse can't run away
#define COMPASS_PULSE_PROGRESS_MIN    1.0f   // deg — below this, count as "stalled" vs drag
#define COMPASS_BRAKE_MS              200UL
// If NEAR-mode pulse direction flips sign this many cycles running, the
// hull is hunting past target both ways on momentum — force a short,
// gentle "fine tap" instead of growing the pulse further.
#define COMPASS_OSCILLATION_LIMIT     2
#define COMPASS_PULSE_MS_FINE         80UL
// A genuine reading can't move more than ~19-25deg between 250ms polls
// even at full FAR speed. A bigger implied jump (e.g. an I2C/EMI glitch)
// is almost certainly bad data, not real motion.
#define COMPASS_MAX_PLAUSIBLE_JUMP_DEG 60.0f

// Any "reached" event is provisional: stop immediately, then wait this
// long and recheck before finalizing, since momentum (FAR or NEAR mode)
// can still carry the hull a few degrees past target.
#define COMPASS_REACHED_VERIFY_MS  400UL

// Centre-heading correction used by CMD:CMOVE's continuous steering.
#define CENTRE_CORRECT_TOL_DEG        5.0f

// FAR mode: expected minimum heading change between polls at full speed —
// if progress stays below this several polls running, ramp PWM (handles a
// floor/obstruction bump stalling the hull while the compass stays valid).
#define FAR_STALL_PROGRESS_MIN_DEG    2.0f
#define FAR_STALL_STREAK_LIMIT        3
#define FAR_SPEED_STEP                15
#define FAR_SPEED_MAX                 255
// NEAR mode: once pulse duration is already maxed and still not making
// progress, ramp COMPASS_NEAR_SPEED instead of repeating useless pulses.
#define NEAR_STALL_AT_CAP_LIMIT       3
#define NEAR_SPEED_STEP               15
#define NEAR_SPEED_MAX                255
// Brief reverse pulse when fully stuck at max PWM in either mode — the one
// deliberate blocking exception here, kept short and rare.
#define ROCK_FREE_MS                  250UL

// CMD:CMOVE continuous steering.
#define CMOVE_STEER_POLL_MS   200UL   // heading recheck interval while driving
#define CMOVE_STEER_KP        3.0f    // deg error -> PWM differential
#define CMOVE_STEER_MAX_CORR  50.0f   // cap so one side never stalls or reverses

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
//  MOTION STATE
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
// alongside DONE (as "DONE HEADING=xx.x"). NAN until a first good read.
float         lastCompassHeadingDeg = NAN;

// Direction chosen once at the start of a turn, held fixed for the FAR
// phase. +1 = heading increases (clockwise), -1 = heading decreases.
int8_t        compassTurnDir       = 1;

bool          compassNearMode      = false;
bool          compassPulsing       = false;
bool          compassBraking       = false;
unsigned long compassPhaseEndMs    = 0;

// Adaptive NEAR-mode pulse duration: grows (capped) when a pulse makes no
// real progress against water drag, resets once progress resumes.
unsigned long compassPulseDurMs    = 0;
float         compassHeadingBeforePulse = 0.0f;

// Last ACCEPTED (plausible) heading during the current turn, used to
// reject glitch readings that imply an impossible jump.
float         lastGoodTurnHeadingDeg = NAN;

// NEAR-mode oscillation tracking (pulse direction flip-flopping on momentum).
int8_t        compassNearDirPrev        = 0;
uint8_t       compassOscillationStreak  = 0;

// FAR-mode stall detection.
float         far_headingAtLastPoll = NAN;
uint8_t       far_stallStreak       = 0;
uint8_t       currentFarSpeed       = SPEED_TURN;

// NEAR-mode stall escalation.
uint8_t       currentNearSpeed      = COMPASS_NEAR_SPEED;
uint8_t       near_stallAtCapStreak = 0;

// Set whenever a stall-escalation branch fires during the current turn,
// reported back to the Jetson on DONE.
bool          turnHadStall = false;

// Reach-verification: set when a poll satisfies compassCurrentTolDeg.
// Motors stop immediately; finalized only after reachedVerifyEndMs passes
// and a fresh read confirms the heading actually held. Skipped entirely
// for coarse turns (tolerance wider than COMPASS_TURN_TOL_DEG) — those
// don't need to be exact, so there's nothing to verify.
bool          reachedPendingVerify  = false;
unsigned long reachedVerifyEndMs    = 0;

// Accept tolerance for the current CTURN. Defaults to COMPASS_TURN_TOL_DEG;
// a caller can widen it (CMD:CTURN <deg> <tol>) for turns that don't need
// to land exactly on target, e.g. scan-dance intermediate splits.
float         compassCurrentTolDeg = COMPASS_TURN_TOL_DEG;

// Centre heading for CMD:CMOVE steering, set by CMD:SETCENTRE.
float         centreHeadingDeg = NAN;

// CMD:CMOVE continuous-steering state.
bool          isCMoveDriving     = false;
unsigned long cmoveEndTime       = 0;
unsigned long cmoveLastPollMs    = 0;
bool          cmoveForward       = true;

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

// dir > 0 makes heading increase (clockwise), dir < 0 decreases it.
void driveCompassDir(int8_t dir, uint8_t speed) {
    if (dir > 0) {
        leftMotor (speed, false);
        rightMotor(speed, true);
    } else {
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

// I2C is disabled during plain timed MOVE/TURN to avoid motor-EMI bus
// lockups. CMD:CMOVE's continuous steering needs live heading feedback,
// so it keeps I2C enabled the whole time instead.
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

// Returns heading in [0, 360), or -1.0 on failure. Single-attempt,
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

// Reads the compass 3x (25ms apart) and returns the median — rejects lone
// glitch samples the same way the in-turn plausibility check does.
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
    isCMoveDriving   = false;
}

void doTurn(float degrees) {
    if (degrees == 0.0f) { sendLine("DONE"); return; }
    if (isMoving || isSettling || isCompassTurning || isCMoveDriving) cancelMotionSilent();
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

void doTurnCompass(float deltaDeg, float tolDeg = COMPASS_TURN_TOL_DEG) {
    if (deltaDeg == 0.0f) { sendLine("DONE"); return; }
    if (isMoving || isSettling || isCompassTurning || isCMoveDriving) cancelMotionSilent();

    compassCurrentTolDeg = tolDeg;

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
        compassTargetHeading = startHeading;
    }
    compassTargetHeading = fmod(compassTargetHeading, 360.0f);
    if (compassTargetHeading < 0.0f) compassTargetHeading += 360.0f;

    BLOGf("[CTURN] start=%.1f  delta=%.1f  target=%.1f  goodReads=%d\r\n",
          startHeading, deltaDeg, compassTargetHeading, goodCount);

    compassTurnDir  = (deltaDeg > 0) ? 1 : -1;
    currentFarSpeed = SPEED_TURN;
    driveCompassDir(compassTurnDir, currentFarSpeed);

    isCompassTurning     = true;
    compassNearMode      = false;
    compassPulsing       = false;
    compassBraking       = false;
    reachedPendingVerify = false;
    compassTurnTimeoutMs = millis() + COMPASS_TURN_MAX_MS;
    lastCompassPollMs    = 0;
    compassFailStreak    = 0;
    lastGoodTurnHeadingDeg = startHeading;
    far_headingAtLastPoll = NAN;
    far_stallStreak        = 0;
    currentNearSpeed       = COMPASS_NEAR_SPEED;
    near_stallAtCapStreak  = 0;
    turnHadStall            = false;
}

void doMove(float metres) {
    if (metres == 0.0f) { sendLine("DONE"); return; }
    if (isMoving || isSettling || isCompassTurning || isCMoveDriving) cancelMotionSilent();
    turnHadStall = false;   // plain MOVE has no stall detection (open-loop)

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

// Continuous-steering move: drives the whole distance without stopping,
// keeping I2C enabled and correcting differential wheel speed toward
// centreHeadingDeg every CMOVE_STEER_POLL_MS. Falls back to a plain
// doMove() if no centre is set or the compass is unavailable.
void doCMove(float metres) {
    if (metres == 0.0f) { sendLine("DONE"); return; }
    if (isnan(centreHeadingDeg) || !compassReady) {
        doMove(metres);
        return;
    }
    if (isMoving || isSettling || isCompassTurning || isCMoveDriving) cancelMotionSilent();
    turnHadStall = false;

    cmoveForward = (metres > 0.0f);
    unsigned long durMs = (unsigned long)((fabs(metres) / MOVE_M_PER_SEC) * 1000.0f);
    BLOGf("[CMOVE] %.2f m  %s  holding %.1fdeg  duration: %lu ms\r\n",
          fabs(metres), cmoveForward ? "forward" : "backward", centreHeadingDeg, durMs);

    leftMotor (SPEED_MOVE, cmoveForward);
    rightMotor(SPEED_MOVE, cmoveForward);

    cmoveEndTime    = millis() + durMs;
    cmoveLastPollMs = 0;
    isCMoveDriving  = true;
}

void doStop() {
    bool wasBusy = isMoving || isSettling || isCompassTurning || isCMoveDriving;
    motorsStop();
    isMoving         = false;
    isSettling       = false;
    isCompassTurning = false;
    compassNearMode  = false;
    compassPulsing   = false;
    compassBraking   = false;
    reachedPendingVerify = false;
    isCMoveDriving   = false;
    if (wasBusy) {
        enableI2C();
        sendLine("DONE");
        BLOGln("[STOP] Motion interrupted.");
    } else {
        BLOGln("[STOP] Already stopped.");
    }
}

// Passive heading query — does NOT move the motors. Replies "HEADING:123.4"
// on success, or "HEADING:ERR" if all 3 compass reads failed.
void doGetHeading() {
    if (isMoving || isSettling || isCompassTurning || isCMoveDriving) cancelMotionSilent();
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
        String rest = line.substring(10);
        int sp = rest.indexOf(' ');
        if (sp == -1) {
            doTurnCompass(rest.toFloat());
        } else {
            float deg = rest.substring(0, sp).toFloat();
            float tol = rest.substring(sp + 1).toFloat();
            doTurnCompass(deg, tol);
        }
    } else if (line.startsWith("CMD:MOVE ")) {
        float dist = line.substring(9).toFloat();
        doMove(dist);
    } else if (line.startsWith("CMD:CMOVE ")) {
        float dist = line.substring(10).toFloat();
        doCMove(dist);
    } else if (line == "CMD:SETCENTRE") {
        // Median-of-3, glitch-safe. Always re-reads rather than trusting
        // any Jetson-supplied value.
        if (isMoving || isSettling || isCompassTurning || isCMoveDriving) cancelMotionSilent();
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

    // ── 1b. Non-blocking CMD:CMOVE continuous-steering check ────────────────
    if (isCMoveDriving) {
        unsigned long nowMs = millis();
        if (nowMs >= cmoveEndTime) {
            motorsStop();
            isCMoveDriving = false;
            isSettling     = true;
            settleEndTime  = nowMs + STOP_SETTLE_MS;
        } else if (nowMs - cmoveLastPollMs >= CMOVE_STEER_POLL_MS) {
            cmoveLastPollMs = nowMs;
            float h = readHeadingDeg();
            if (h >= 0.0f && !isnan(lastCompassHeadingDeg)) {
                float jump = fabs(headingDiffDeg(lastCompassHeadingDeg, h));
                if (jump > COMPASS_MAX_PLAUSIBLE_JUMP_DEG) h = -1.0f;
            }
            if (h >= 0.0f) {
                lastCompassHeadingDeg = h;
                float err  = headingDiffDeg(h, centreHeadingDeg);
                float corr = err * CMOVE_STEER_KP;
                if (corr >  CMOVE_STEER_MAX_CORR) corr =  CMOVE_STEER_MAX_CORR;
                if (corr < -CMOVE_STEER_MAX_CORR) corr = -CMOVE_STEER_MAX_CORR;
                // err > 0 -> heading needs to increase -> speed up left,
                // slow right (mirrors driveCompassDir's polarity). Flips
                // for reverse, since wheel torque direction is mirrored.
                float bias = cmoveForward ? corr : -corr;
                int leftSpeed  = (int)(SPEED_MOVE + bias);
                int rightSpeed = (int)(SPEED_MOVE - bias);
                leftSpeed  = constrain(leftSpeed, 60, 255);
                rightSpeed = constrain(rightSpeed, 60, 255);
                leftMotor (leftSpeed,  cmoveForward);
                rightMotor(rightSpeed, cmoveForward);
                if (fabs(err) > 1.0f) {
                    BLOGf("[CMOVE] heading=%.1f target=%.1f err=%.1f "
                          "L=%d R=%d\r\n", h, centreHeadingDeg, err,
                          leftSpeed, rightSpeed);
                }
            }
        }
    }

    // ── 1c. Non-blocking compass-turn-complete check (CTURN) ────────────────
    if (isCompassTurning) {
        unsigned long nowMs = millis();

        if (reachedPendingVerify) {
            // Motors are already stopped. Wait out the coast-settle window
            // before trusting a "reached" event, then recheck.
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
                        // Coasted past tolerance — resume closed-loop from
                        // where it actually is now.
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
                    reachedVerifyEndMs = nowMs + 100UL;
                }
            }
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
                bool  reached = false;
                bool  failed  = false;
                float err     = 0.0f;

                if (h >= 0.0f && !isnan(lastGoodTurnHeadingDeg)) {
                    float jump = fabs(headingDiffDeg(lastGoodTurnHeadingDeg, h));
                    if (jump > COMPASS_MAX_PLAUSIBLE_JUMP_DEG) {
                        BLOGf("[CTURN] Implausible jump %.1fdeg (last-good=%.1f "
                              "new=%.1f) — rejecting as glitch, treating as bad read\r\n",
                              jump, lastGoodTurnHeadingDeg, h);
                        h = -1.0f;
                    }
                }

                if (h >= 0.0f) {
                    compassFailStreak = 0;
                    lastCompassHeadingDeg  = h;
                    lastGoodTurnHeadingDeg = h;
                    err = headingDiffDeg(h, compassTargetHeading);
                    BLOGf("[CTURN] heading=%.1f  target=%.1f  err=%.1f  mode=%s\r\n",
                          h, compassTargetHeading, err, compassNearMode ? "NEAR" : "FAR");

                    if (fabs(err) <= compassCurrentTolDeg) {
                        reached = true;
                    } else if (!compassNearMode && fabs(err) <= COMPASS_NEAR_THRESHOLD_DEG) {
                        motorsStop();
                        compassNearMode           = true;
                        compassBraking            = true;
                        compassPhaseEndMs         = nowMs + COMPASS_BRAKE_MS;
                        compassPulseDurMs         = COMPASS_PULSE_MS;
                        compassHeadingBeforePulse = h;
                        compassNearDirPrev        = 0;
                        compassOscillationStreak  = 0;
                    } else if (compassNearMode) {
                        int8_t nearDir = (err > 0.0f) ? 1 : -1;

                        bool dirFlipped = (compassNearDirPrev != 0 && nearDir != compassNearDirPrev);
                        compassOscillationStreak = dirFlipped ? (compassOscillationStreak + 1) : 0;

                        float progress = fabs(headingDiffDeg(compassHeadingBeforePulse, h));
                        if (compassOscillationStreak >= COMPASS_OSCILLATION_LIMIT) {
                            compassPulseDurMs = COMPASS_PULSE_MS_FINE;
                            BLOGf("[CTURN] NEAR oscillating (streak=%u) — forcing "
                                  "fine pulse %lums\r\n",
                                  (unsigned)compassOscillationStreak, compassPulseDurMs);
                        } else if (compassPulseDurMs > 0) {
                            if (progress < COMPASS_PULSE_PROGRESS_MIN) {
                                if (compassPulseDurMs < COMPASS_PULSE_MS_MAX) {
                                    compassPulseDurMs = min(compassPulseDurMs + 100UL, COMPASS_PULSE_MS_MAX);
                                    BLOGf("[CTURN] NEAR pulse stalled (progress=%.1f) — "
                                          "lengthening to %lums\r\n", progress, compassPulseDurMs);
                                    near_stallAtCapStreak = 0;
                                } else {
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
                                currentNearSpeed       = COMPASS_NEAR_SPEED;
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
                    bool isCoarse = compassCurrentTolDeg > COMPASS_TURN_TOL_DEG + 0.01f;
                    if (isCoarse) {
                        motorsStop();
                        isCompassTurning = false;
                        compassNearMode  = false;
                        compassPulsing   = false;
                        compassBraking   = false;
                        isSettling       = true;
                        settleEndTime    = nowMs + STOP_SETTLE_MS;
                        compassConsecutiveTurnFails = 0;
                        BLOGf("[CTURN] Reached (coarse, tol=%.1f) heading=%.1f "
                              "(target=%.1f)\r\n", compassCurrentTolDeg, h, compassTargetHeading);
                    } else {
                        motorsStop();
                        reachedPendingVerify = true;
                        reachedVerifyEndMs   = nowMs + COMPASS_REACHED_VERIFY_MS;
                        BLOGf("[CTURN] Tolerance hit (err=%.1f mode=%s) — verifying "
                              "after %lums coast-settle before accepting\r\n",
                              err, compassNearMode ? "NEAR" : "FAR",
                              (unsigned long)COMPASS_REACHED_VERIFY_MS);
                    }
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
        float freshH = readHeadingDeg();
        if (freshH >= 0.0f && !isnan(lastCompassHeadingDeg)) {
            float jump = fabs(headingDiffDeg(lastCompassHeadingDeg, freshH));
            if (jump > COMPASS_MAX_PLAUSIBLE_JUMP_DEG) {
                BLOGf("[SETTLE] Implausible final read (last=%.1f new=%.1f "
                      "jump=%.1f) — keeping last known heading\r\n",
                      lastCompassHeadingDeg, freshH, jump);
                freshH = -1.0f;
            }
        }
        if (freshH >= 0.0f) lastCompassHeadingDeg = freshH;

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
