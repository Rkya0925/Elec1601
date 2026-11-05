#include <Servo.h>

// =====================================================================
//  EXTENSION PART 2 - find the LONG dead end
//
//  Built on our Extension Part 1 code (which passed): same pins, motor
//  calibration, corner approach -> turn -> slow exit, start alignment,
//  IR measurement and U-turn. Centring changes after lab testing (marked
//  in the code): sharper alignment, damping to stop ping-ponging, holding
//  distance from a single wall, and a pivot-away rescue near a wall.
//  Before a 90 degree turn, if the back would hit a wall, it reverses and
//  straightens up to the right spot first (turns only).
//  The robot reads all three sensors about every 50 ms while it drives
//  and steers to stay centred, so it never drives blind.
//
//  New for Part 2:
//    T-junction     (wall ahead, both sides open) -> turn to our wall side
//    Side branch    (front open, our side opens)  -> drive to its middle, turn
//    Dead end       (wall ahead, both sides wall) -> how long was the walled
//                   corridor before it?
//                      ~2 walls long -> LONG dead end: stop facing it
//                      anything else -> U-turn and keep exploring
//
//  Strategy: RIGHT-HAND RULE. Keep the right wall beside us and take every
//  right opening. In a maze without loops this walks past every dead end,
//  so it always reaches the long one - from any start, normal or mirrored.
// =====================================================================

// ==========================================
// PINS
// ==========================================

const int servoLeftPin = 13;
const int servoRightPin = 12;

const int irLedLeft = 10;
const int irReceiverLeft = 11;
const int redLedLeft = A2;

const int irLedMid = 6;
const int irReceiverMid = 7;
const int redLedMid = A1;

const int irLedRight = 2;
const int irReceiverRight = 3;
const int redLedRight = A0;

Servo servoLeft;
Servo servoRight;

// ==========================================
// MOTOR CALIBRATION (unchanged from Part 1)
// ==========================================

const int NEUTRAL = 1500;

const int LEFT_OFFSET = 0;
const int RIGHT_OFFSET = 2;

const int FORWARD_SPEED = 71;
const int TURN_SPEED = 71;

// ==========================================
// CENTRING (Part 2: sharper than Part 1)
// ==========================================

// Part 1 was 2.0 / 14. Sharper alignment along walls:
const float CENTER_GAIN = 3.5;          // more = straightens up faster
const float CENTER_DEADBAND = 0.25;
const int MAX_CENTER_ADJUST = 24;       // more = sharper correction allowed

// DAMPING (new): steers less when the robot is ALREADY moving back to
// the middle. This is what stops ping-ponging between the walls.
//   still ping-pongs -> increase (e.g. 6)
//   corrects too slowly / drifts -> decrease (e.g. 2)
const float CENTER_DAMP = 4.0;

// With only ONE wall visible, hold this distance from it (the side
// reading you get with the robot centred).
const float ONE_WALL_TARGET = 7.0;

// WALL RESCUE (new): if a side sensor reads this close (2.0 = about to
// touch), stop and pivot away on the spot instead of scraping along it.
const float RESCUE_DIST = 2.0;
const int RESCUE_PIVOT_MS = 60;         // ~9 degrees per pivot

// ==========================================
// CENTRING AFTER A TURN
// ==========================================

const int EXIT_SPEED = 45;
// Part 1 was 6.0. Lowered because at slow speed 6.0 overshoots the
// middle and ping-pongs; the damping above also applies here.
const float EXIT_CENTER_GAIN = 4.0;
const int EXIT_MAX_ADJUST = 22;

// ==========================================
// CORNER SETTINGS (unchanged from Part 1)
// ==========================================

const int BEFORE_TURN_MS = 200;
const int AFTER_TURN_MS = 600;

const int LEFT_TURN_MS = 595;
const int RIGHT_TURN_MS = 595;

const float FRONT_WALL = 10.0;
const float APPROACH_STOP = 6.5;

// ==========================================
// STARTUP ALIGNMENT (unchanged from Part 1)
// ==========================================

const int START_MAX_ATTEMPTS = 30;
const int START_ROTATION_MS = 40;
const int START_FORWARD_MS = 40;

// ==========================================
// PART 2 SETTINGS
// ==========================================

// true  = right-hand rule (take right openings, right at T-junctions)
// false = left-hand rule
const bool FOLLOW_RIGHT_WALL = true;

// getDistance() only returns <= 10.0 (wall) or 12.0 (open)
const float SIDE_OPEN = 12.0;

// A side only counts as OPEN (or as a WALL) after this many readings in a
// row. One reading takes ~50 ms (~0.6 cm). This stops one bad reading from
// turning us into a wall or restarting the long-dead-end measurement.
const int CONFIRM_READINGS = 3;

// Side branch: once the opening is confirmed, drive on this long so the
// wheels are level with the middle of the opening, then turn.
//   clips the NEAR pillar (turns too early) -> increase
//   clips the FAR pillar  (turns too late)  -> decrease
const int BRANCH_FORWARD_MS = 1000;

// Long dead end = the corridor with walls on BOTH sides before the dead
// end is about 2 walls long. Every dead end prints
//   "DEAD END #n: walled corridor = X cm"
// Calibrate: MIN halfway between the 1-wall and 2-wall values,
//            MAX halfway between the 2-wall and 3-wall values.
const float LONG_DEAD_END_MIN_CM = 36.0;
const float LONG_DEAD_END_MAX_CM = 64.0;

// totalTravel counts milliseconds at FORWARD_SPEED.
// driveForward5cm() = 440 ms = 5 cm  ->  88 ms per cm.
const float TRAVEL_MS_PER_CM = 88.0;

// At the long dead end: creep closer to the end wall (same approach as a
// corner: stops at APPROACH_STOP or after this time).
const int FINAL_APPROACH_MS = 400;

// TURN CLEARANCE (new, 90 degree turns ONLY): when the robot spins,
// its BACK swings out.
//   right turn -> back swings LEFT, left turn -> back swings RIGHT.
// Before a 90 degree turn, the wall on that side must read at least
// this far. If it's closer, the robot reverses and straightens up away
// from it, drives back up to the turning spot, then turns.
//   back still clips -> raise to 7.0
//   does it when it doesn't need to -> lower to 6.0
const float TURN_SIDE_CLEAR = 6.5;

// The reverse-and-straighten move: angle slightly, reverse, straighten.
// Each one moves the robot ~2 cm away from the wall (and ~4 cm back,
// which it drives forward again before turning).
const int SHUFFLE_PIVOT_MS = 200;       // ~30 degrees
const int SHUFFLE_BACK_MS = 400;        // ~4.5 cm
const int SHUFFLE_MAX = 3;              // max tries before turning anyway
const int REAPPROACH_MS = 400;          // drive back up to the turning spot
                                        // (same as SHUFFLE_BACK_MS, so it ends
                                        // where it started; stops early at a wall)

// Short dead end: reverse this long before the U-turn spin.
// 0 = off. Only use it if the front clips the end wall while spinning.
const int UTURN_BACKUP_MS = 0;

// ==========================================
// STATE
// ==========================================

bool finished = false;
bool reachedLongDeadEnd = false;

float dLeft = 12;
float dMid = 12;
float dRight = 12;

float totalTravel = 0;      // ms driven at FORWARD_SPEED since switch-on
float walledStart = 0;      // totalTravel when the walled corridor began
int deadEnds = 0;

// Side memory (updated every loop)
int leftOpenCount = 0;      // open readings in a row
int rightOpenCount = 0;
int leftWallCount = 0;      // wall readings in a row
int rightWallCount = 0;

// Has this side shown a wall since our last turn?
// Right after a turn, the corridor we came out of is still beside us and
// looks like an opening. An opening only counts as a BRANCH once that
// side has shown a wall - so we never turn back the way we came.
bool leftWallSeen = false;
bool rightWallSeen = false;

int currentLeftSpeed = 0;
int currentRightSpeed = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastPrint = 0;

// ==========================================
// TRAVEL TRACKING (unchanged from Part 1)
// ==========================================

void updateTravel() {
    unsigned long now = millis();
    unsigned long elapsed = now - lastMotorUpdate;

    float averageSpeed =
        (currentLeftSpeed + currentRightSpeed) / 2.0;

    if (averageSpeed > 0) {
        totalTravel +=
            elapsed * averageSpeed / FORWARD_SPEED;
    }

    lastMotorUpdate = now;
}

// ==========================================
// MOTOR FUNCTIONS (unchanged from Part 1)
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
    servoLeft.writeMicroseconds(leftSpeed);
    servoRight.writeMicroseconds(rightSpeed);
}

void setSpeed(int leftSpeed, int rightSpeed) {
    updateTravel();

    currentLeftSpeed = leftSpeed;
    currentRightSpeed = rightSpeed;

    int leftCorrection = 0;
    int rightCorrection = 0;

    if (leftSpeed > 0) {
        leftCorrection = LEFT_OFFSET;
    } else if (leftSpeed < 0) {
        leftCorrection = -LEFT_OFFSET;
    }

    if (rightSpeed > 0) {
        rightCorrection = RIGHT_OFFSET;
    } else if (rightSpeed < 0) {
        rightCorrection = -RIGHT_OFFSET;
    }

    setMotors(
        NEUTRAL + leftSpeed + leftCorrection,
        NEUTRAL - rightSpeed - rightCorrection
    );
}

void setSpeed(int speed) {
    setSpeed(speed, speed);
}

void halt() {
    setSpeed(0);
}

// ==========================================
// MOVEMENT FUNCTIONS (unchanged from Part 1)
// ==========================================

void driveForward5cm() {
    setSpeed(FORWARD_SPEED);
    delay(440);
    halt();
}

void turnLeft90() {
    setSpeed(-TURN_SPEED, TURN_SPEED);
    delay(LEFT_TURN_MS);
    halt();
}

void turnRight90() {
    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(RIGHT_TURN_MS);
    halt();
}

// Original U-turn.
void spin180() {
    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(1250);
    halt();
}

// Forward declaration (defined further down)
bool moveCenteredFor(int duration, float stopDistance, bool afterTurn);

void performUTurn() {
    spin180();

    // CHANGED for Part 2: the 5 cm after the spin used to be a blind
    // driveForward5cm(). Same 440 ms, but now it reads the sensors and
    // centres the whole way, and stops if a wall appears ahead.
    moveCenteredFor(440, FRONT_WALL, true);
}

// ==========================================
// LEDs (unchanged from Part 1)
// ==========================================

void showScenario(int scenario) {
    int pattern = scenario;

    if (scenario == 8) {
        pattern = 1;
    }

    digitalWrite(
        redLedLeft,
        (pattern & 4) ? HIGH : LOW
    );

    digitalWrite(
        redLedMid,
        (pattern & 2) ? HIGH : LOW
    );

    digitalWrite(
        redLedRight,
        (pattern & 1) ? HIGH : LOW
    );
}

// ==========================================
// IR MEASUREMENT (unchanged from Part 1)
// ==========================================

float getDistance(int ledPin, int rxPin) {
    int hits = 0;

    for (long f = 38000; f <= 45000; f += 1000) {
        tone(ledPin, f);
        delay(1);

        if (digitalRead(rxPin) == LOW) {
            hits++;
        }

        noTone(ledPin);
        delay(1);
    }

    if (hits >= 8) return 2.0;
    if (hits == 7) return 5.0;
    if (hits == 6) return 6.0;
    if (hits == 5) return 6.5;
    if (hits == 4) return 7.0;
    if (hits == 3) return 8.5;
    if (hits == 2) return 10.0;

    return 12.0;
}

void readSensorsWithLimit(float stopDistance) {
    dMid = getDistance(irLedMid, irReceiverMid);

    if (dMid <= stopDistance) {
        halt();
    }

    dLeft = getDistance(irLedLeft, irReceiverLeft);
    dRight = getDistance(irLedRight, irReceiverRight);

    updateTravel();
}

void readSensors() {
    readSensorsWithLimit(FRONT_WALL);
}

// NEW: when stopped at a wall, read every sensor 3 times and keep the
// middle value, so one bad reading can't change the decision.
float middleOf3(float a, float b, float c) {
    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

void readSensorsCarefully() {
    float m[3];
    float l[3];
    float r[3];

    for (int i = 0; i < 3; i++) {
        m[i] = getDistance(irLedMid, irReceiverMid);
        l[i] = getDistance(irLedLeft, irReceiverLeft);
        r[i] = getDistance(irLedRight, irReceiverRight);
    }

    dMid = middleOf3(m[0], m[1], m[2]);
    dLeft = middleOf3(l[0], l[1], l[2]);
    dRight = middleOf3(r[0], r[1], r[2]);
}

// ==========================================
// CENTRING CONTROL (Part 1, with Part 2 changes marked)
// ==========================================

// For damping: the error at the last reading, and which wall(s) it used.
// Errors from different walls aren't comparable, so damping only uses
// the change when the same wall(s) were visible both times.
float lastCenterError = 0;
int lastErrorType = 0;          // 0 none, 1 both walls, 2 left only, 3 right only
unsigned long lastErrorTime = 0;

void driveCenteredAt(
    int baseSpeed,
    float gain,
    int maxAdjustment
) {
    bool leftVisible = dLeft <= 10.0;
    bool rightVisible = dRight <= 10.0;

    // NEW: about to touch a side wall -> pivot away on the spot first.
    // Driving forward while angled into a wall just scrapes along it.
    if (dLeft <= RESCUE_DIST && dRight > RESCUE_DIST) {
        showScenario(7);
        setSpeed(TURN_SPEED, -TURN_SPEED);      // pivot right
        delay(RESCUE_PIVOT_MS);
        halt();
        lastErrorType = 0;
        return;
    }
    if (dRight <= RESCUE_DIST && dLeft > RESCUE_DIST) {
        showScenario(8);
        setSpeed(-TURN_SPEED, TURN_SPEED);      // pivot left
        delay(RESCUE_PIVOT_MS);
        halt();
        lastErrorType = 0;
        return;
    }

    float error = 0;
    int type = 0;

    if (leftVisible && rightVisible) {
        // Positive error means the left wall is closer.
        // Steer right, away from that wall.
        error = dRight - dLeft;

        if (dLeft < dRight) {
            error += 0.5;
        }
        type = 1;
    } else if (leftVisible) {
        // CHANGED for Part 2: only the left wall is visible. Part 1 only
        // reacted below 5, so the robot strayed. Hold the centred distance.
        error = ONE_WALL_TARGET - dLeft;
        type = 2;
    } else if (rightVisible) {
        // CHANGED for Part 2: same for the right wall.
        error = dRight - ONE_WALL_TARGET;
        type = 3;
    }

    // NEW: damping. If the error is shrinking (already heading back to
    // the middle) this steers less; if growing, it steers more.
    unsigned long now = millis();
    float change = 0;
    if (type != 0 && type == lastErrorType &&
        now - lastErrorTime < 300) {
        change = error - lastCenterError;
    }
    lastCenterError = error;
    lastErrorType = type;
    lastErrorTime = now;

    int adjustment = 0;

    if (error > CENTER_DEADBAND ||
        error < -CENTER_DEADBAND) {
        adjustment = (int)(error * gain + change * CENTER_DAMP);
    }

    if (adjustment > maxAdjustment) {
        adjustment = maxAdjustment;
    }

    if (adjustment < -maxAdjustment) {
        adjustment = -maxAdjustment;
    }

    if (adjustment > 0) {
        showScenario(5);
    } else if (adjustment < 0) {
        showScenario(6);
    } else {
        showScenario(1);
    }

    setSpeed(
        baseSpeed + adjustment,
        baseSpeed - adjustment
    );
}

void driveCentered() {
    driveCenteredAt(
        FORWARD_SPEED,
        CENTER_GAIN,
        MAX_CENTER_ADJUST
    );
}

void driveCenteredAfterTurn() {
    driveCenteredAt(
        EXIT_SPEED,
        EXIT_CENTER_GAIN,
        EXIT_MAX_ADJUST
    );
}

// ==========================================
// CHECKED APPROACH / CORNER EXIT (unchanged from Part 1)
// ==========================================

bool moveCenteredFor(
    int duration,
    float stopDistance,
    bool afterTurn
) {
    unsigned long started = millis();

    while (millis() - started < (unsigned long)duration) {
        readSensorsWithLimit(stopDistance);

        if (dMid <= stopDistance) {
            halt();
            return false;
        }

        if (dLeft <= 3.5 && dRight <= 3.5) {
            halt();
            return false;
        }

        if (millis() - started >= (unsigned long)duration) {
            break;
        }

        if (afterTurn) {
            driveCenteredAfterTurn();
        } else {
            driveCentered();
        }
    }

    halt();
    return true;
}

// ==========================================
// STOP
// ==========================================

void stopRobot(const char *message) {
    halt();
    finished = true;
    Serial.println(message);
}

// ==========================================
// STARTUP ALIGNMENT (Part 1, one change marked)
// ==========================================

void correctStartingPosition() {
    halt();

    int balancedReadings = 0;

    for (int attempt = 0;
         attempt < START_MAX_ATTEMPTS;
         attempt++) {

        readSensors();

        bool leftVisible = dLeft <= 10.0;
        bool rightVisible = dRight <= 10.0;
        bool frontClear = dMid > FRONT_WALL;

        Serial.print("START L:");
        Serial.print(dLeft);
        Serial.print(" F:");
        Serial.print(dMid);
        Serial.print(" R:");
        Serial.println(dRight);

        if (!leftVisible && !rightVisible) {
            halt();
            balancedReadings = 0;
            continue;
        }

        if (dLeft <= 3.5 && dRight <= 3.5) {
            stopRobot("START: too close on both sides.");
            return;
        }

        if (leftVisible && rightVisible) {
            if (!frontClear) {
                // CHANGED for Part 2: facing a dead end at the start is
                // allowed - navigation will U-turn out of it.
                Serial.println(
                    "START: wall ahead; navigation will handle it."
                );
                return;
            }

            float difference = dRight - dLeft;

            if (difference >= -CENTER_DEADBAND &&
                difference <= CENTER_DEADBAND) {

                halt();
                balancedReadings++;

                if (balancedReadings >= 2) {
                    Serial.println(
                        "START ALIGNED: continuing navigation."
                    );
                    return;
                }

                continue;
            }

            balancedReadings = 0;

            driveCentered();
            delay(START_FORWARD_MS);
            halt();
        } else {
            balancedReadings = 0;

            if (leftVisible) {
                showScenario(7);
                setSpeed(TURN_SPEED, -TURN_SPEED);
            } else {
                showScenario(8);
                setSpeed(-TURN_SPEED, TURN_SPEED);
            }

            delay(START_ROTATION_MS);
            halt();
        }
    }

    halt();

    Serial.println(
        "Startup correction finished; continuing navigation."
    );
}

// ==========================================
// NEW: SIDE MEMORY + WALLED-CORRIDOR LENGTH
// ==========================================

int countUp(int count) {
    return (count < 100) ? count + 1 : count;   // can't overflow
}

// Called once per loop, after reading the sensors.
void updateSideMemory() {
    if (dLeft >= SIDE_OPEN) {
        leftOpenCount = countUp(leftOpenCount);
        leftWallCount = 0;
    } else {
        leftWallCount = countUp(leftWallCount);
        leftOpenCount = 0;
    }

    if (dRight >= SIDE_OPEN) {
        rightOpenCount = countUp(rightOpenCount);
        rightWallCount = 0;
    } else {
        rightWallCount = countUp(rightWallCount);
        rightOpenCount = 0;
    }

    if (leftWallCount >= CONFIRM_READINGS) leftWallSeen = true;
    if (rightWallCount >= CONFIRM_READINGS) rightWallSeen = true;

    // A confirmed opening on either side means we are not in a walled
    // corridor: the walled corridor starts again from here.
    if (leftOpenCount >= CONFIRM_READINGS ||
        rightOpenCount >= CONFIRM_READINGS) {
        updateTravel();
        walledStart = totalTravel;
    }
}

// After every turn (90 or 180) we are in a new corridor.
void newCorridor() {
    updateTravel();
    walledStart = totalTravel;

    leftOpenCount = 0;
    rightOpenCount = 0;
    leftWallCount = 0;
    rightWallCount = 0;

    leftWallSeen = false;
    rightWallSeen = false;
}

float walledCorridorCm() {
    updateTravel();
    return (totalTravel - walledStart) / TRAVEL_MS_PER_CM;
}

// ==========================================
// TURNS (corner, T-junction, branch)
// ==========================================

// NEW: move the robot sideways a little, without changing its heading.
// Angle so the back points the way we want to go, reverse, straighten up.
void shuffleSideways(bool moveRight) {
    if (moveRight) {
        setSpeed(-TURN_SPEED, TURN_SPEED);      // angle left (back points right)
    } else {
        setSpeed(TURN_SPEED, -TURN_SPEED);      // angle right (back points left)
    }
    delay(SHUFFLE_PIVOT_MS);
    halt();

    setSpeed(-FORWARD_SPEED);                    // reverse
    delay(SHUFFLE_BACK_MS);
    halt();

    if (moveRight) {
        setSpeed(TURN_SPEED, -TURN_SPEED);      // straighten up
    } else {
        setSpeed(-TURN_SPEED, TURN_SPEED);
    }
    delay(SHUFFLE_PIVOT_MS);
    halt();
}

// NEW (90 degree turns only): if the back would hit the wall when we
// spin, reverse and straighten up away from it, then drive back up to
// the turning spot. Right turn -> the back swings LEFT, so check left.
void alignBeforeTurn(bool turnRight) {
    for (int i = 0; i < SHUFFLE_MAX; i++) {
        readSensorsCarefully();

        float swingSide = turnRight ? dLeft : dRight;

        if (swingSide >= TURN_SIDE_CLEAR) {
            return;     // enough room for the back (an opening reads 12)
        }

        Serial.println(turnRight
            ? "TURN: TOO CLOSE TO LEFT WALL - REVERSE AND STRAIGHTEN"
            : "TURN: TOO CLOSE TO RIGHT WALL - REVERSE AND STRAIGHTEN");

        shuffleSideways(turnRight);     // move away from that wall

        // Back up to the turning spot (stops at the front wall as usual)
        moveCenteredFor(REAPPROACH_MS, APPROACH_STOP, false);
        halt();
    }
}

// Rotate 90 degrees, then the Part 1 slow exit with stronger centring.
void rotateAndExit(bool turnRight) {
    alignBeforeTurn(turnRight);     // NEW: make room for the back first

    if (turnRight) {
        Serial.println("RIGHT ROTATION");
        showScenario(2);
        turnRight90();
    } else {
        Serial.println("LEFT ROTATION");
        showScenario(3);
        turnLeft90();
    }

    newCorridor();

    Serial.println("SLOW EXIT WITH STRONGER CENTRING");

    bool exitClear = moveCenteredFor(
        AFTER_TURN_MS,
        FRONT_WALL,
        true
    );

    if (exitClear) {
        Serial.println("TURN COMPLETE");
    } else {
        Serial.println("EXIT BLOCKED: RECHECKING");
    }
}

// Corner or T-junction: same approach as Part 1, then turn.
void takeCorner(bool turnRight) {
    halt();

    Serial.println("CORNER APPROACH");

    moveCenteredFor(
        BEFORE_TURN_MS,
        APPROACH_STOP,
        false
    );

    rotateAndExit(turnRight);
}

// Side branch (front still open): drive to the middle of the opening,
// then turn into it. If a wall appears ahead on the way, it is really a
// corner or T-junction - stop, and navigate() handles it next loop.
//
// The distance is measured with totalTravel (ms at FORWARD_SPEED), so a
// false front reading only pauses us: we then drive the REMAINING
// distance instead of starting the 1000 ms again (which turned us late).
void takeBranch(bool turnRight) {
    Serial.println(turnRight ? "BRANCH ON RIGHT" : "BRANCH ON LEFT");

    updateTravel();
    float target = totalTravel + BRANCH_FORWARD_MS;

    for (int attempt = 0; attempt < 3; attempt++) {
        updateTravel();
        if (totalTravel >= target) {
            break;
        }

        bool clear = moveCenteredFor(
            (int)(target - totalTravel),
            FRONT_WALL,
            false
        );

        updateTravel();
        if (clear) {
            break;
        }

        // Stopped: something ahead? Check carefully.
        readSensorsCarefully();

        if (dMid <= FRONT_WALL) {
            Serial.println("WALL AHEAD: CORNER / T - RECHECKING");
            return;
        }

        Serial.println("FALSE FRONT READING: CONTINUING TO BRANCH");
    }

    rotateAndExit(turnRight);
}

// ==========================================
// DEAD END: LONG -> FINISH, SHORT -> U-TURN
// ==========================================

void handleDeadEnd() {
    halt();
    showScenario(4);

    deadEnds++;
    float walled = walledCorridorCm();

    Serial.print("DEAD END #");
    Serial.print(deadEnds);
    Serial.print(": walled corridor = ");
    Serial.print(walled);
    Serial.println(" cm");

    if (walled >= LONG_DEAD_END_MIN_CM &&
        walled <= LONG_DEAD_END_MAX_CM) {

        Serial.println("LONG DEAD END: FINAL APPROACH");

        moveCenteredFor(
            FINAL_APPROACH_MS,
            APPROACH_STOP,
            false
        );

        reachedLongDeadEnd = true;
        stopRobot("LONG DEAD END REACHED - FINISHED");
        return;
    }

    Serial.println("SHORT DEAD END: U-TURN");

    if (UTURN_BACKUP_MS > 0) {
        setSpeed(-FORWARD_SPEED);
        delay(UTURN_BACKUP_MS);
        halt();
    }

    performUTurn();
    newCorridor();
}

// ==========================================
// NAVIGATION
// ==========================================

void navigate() {
    // ---------- Wall in front ----------
    if (dMid <= FRONT_WALL) {
        halt();

        // Stopped: take careful readings before deciding.
        readSensorsCarefully();

        if (dMid > FRONT_WALL) {
            return;     // false alarm - keep driving next loop
        }

        bool leftOpen = dLeft >= SIDE_OPEN;
        bool rightOpen = dRight >= SIDE_OPEN;

        if (!leftOpen && !rightOpen) {
            handleDeadEnd();
            return;
        }

        if (leftOpen && rightOpen) {
            Serial.println("T-JUNCTION");
            takeCorner(FOLLOW_RIGHT_WALL);
            return;
        }

        // Corner: only one way to go
        takeCorner(rightOpen);
        return;
    }

    // ---------- Front open ----------
    if (dLeft <= 3.5 && dRight <= 3.5) {
        halt();
        showScenario(0);
        return;
    }

    // Opening on the side of the wall we follow -> take it.
    // (xWallSeen = it is not the corridor we just came out of.)
    if (FOLLOW_RIGHT_WALL &&
        rightOpenCount >= CONFIRM_READINGS &&
        rightWallSeen) {
        takeBranch(true);
        return;
    }

    if (!FOLLOW_RIGHT_WALL &&
        leftOpenCount >= CONFIRM_READINGS &&
        leftWallSeen) {
        takeBranch(false);
        return;
    }

    // Openings on the other side are ignored: keep centring along.
    driveCentered();
}

// ==========================================
// SERIAL MONITOR
// ==========================================

void printStatus() {
    if (millis() - lastPrint < 500) {
        return;
    }

    lastPrint = millis();

    Serial.print("F:");
    Serial.print(dMid);
    Serial.print(" L:");
    Serial.print(dLeft);
    Serial.print(" R:");
    Serial.print(dRight);
    Serial.print(" Walled cm:");
    Serial.print(walledCorridorCm());
    Serial.print(" WallSeen L:");
    Serial.print(leftWallSeen);
    Serial.print(" R:");
    Serial.println(rightWallSeen);
}

// ==========================================
// SETUP
// ==========================================

void setup() {
    Serial.begin(9600);

    pinMode(irLedLeft, OUTPUT);
    pinMode(irReceiverLeft, INPUT);
    pinMode(redLedLeft, OUTPUT);

    pinMode(irLedMid, OUTPUT);
    pinMode(irReceiverMid, INPUT);
    pinMode(redLedMid, OUTPUT);

    pinMode(irLedRight, OUTPUT);
    pinMode(irReceiverRight, INPUT);
    pinMode(redLedRight, OUTPUT);

    servoLeft.writeMicroseconds(NEUTRAL);
    servoRight.writeMicroseconds(NEUTRAL);

    servoLeft.attach(servoLeftPin);
    servoRight.attach(servoRightPin);

    lastMotorUpdate = millis();

    halt();
    showScenario(0);

    Serial.println("Starting in 1 second.");
    delay(1000);

    lastMotorUpdate = millis();

    correctStartingPosition();

    newCorridor();
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
    if (finished) {
        halt();

        // Long dead end reached: flash all LEDs
        if (reachedLongDeadEnd) {
            int on = ((millis() / 300) % 2) ? HIGH : LOW;
            digitalWrite(redLedLeft, on);
            digitalWrite(redLedMid, on);
            digitalWrite(redLedRight, on);
        }
        return;
    }

    readSensors();
    updateSideMemory();
    navigate();
    printStatus();
}
