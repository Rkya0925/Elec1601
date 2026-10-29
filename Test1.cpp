#include <Servo.h>

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
// MOTOR CALIBRATION — ORIGINAL VALUES
// ==========================================

const int NEUTRAL = 1500;

const int LEFT_OFFSET = 0;
const int RIGHT_OFFSET = 4;

const int FORWARD_SPEED = 71;
const int REVERSE_SPEED = 71;
const int TURN_SPEED = 71;

const int ARC_FAST = 90;
const int ARC_SLOW = 40;
const int ARC_MEDIUM = 30;
const int ARC_SHARP = 97;

// ==========================================
// MOTOR FUNCTIONS — ORIGINAL MOVEMENTS
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
    servoLeft.writeMicroseconds(leftSpeed);
    servoRight.writeMicroseconds(rightSpeed);
}

// Positive = forward.
// Negative = backward.
// Zero = stop.
void setSpeed(int leftSpeed, int rightSpeed) {
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

    int leftMotor = NEUTRAL
                  + leftSpeed
                  + leftCorrection;

    int rightMotor = NEUTRAL
                   - rightSpeed
                   - rightCorrection;

    setMotors(leftMotor, rightMotor);
}

void setSpeed(int speed) {
    setSpeed(speed, speed);
}

void halt() {
    setSpeed(0);
}

void driveForward5cm() {
    setSpeed(FORWARD_SPEED);
    delay(440);
    halt();
}

void driveBackward5cm() {
    setSpeed(-REVERSE_SPEED);
    delay(220);
    halt();
}

void driveForward(int ms) {
    setSpeed(FORWARD_SPEED);
    delay(ms);
    halt();
}

void turnLeft90() {
    setSpeed(-TURN_SPEED, TURN_SPEED);
    delay(305);

    driveForward(100);

    setSpeed(-TURN_SPEED, TURN_SPEED);
    delay(290);

    halt();
}

void turnRight90() {
    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(305);

    driveForward(100);

    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(290);

    halt();
}

void spin180() {
    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(1250);
    halt();
}

void turnSuperSlightCW() {
    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(120);
    halt();
}

void turnSuperSlightCCW() {
    setSpeed(-TURN_SPEED, TURN_SPEED);
    delay(120);
    halt();
}

void turnSlightCW() {
    setSpeed(TURN_SPEED, -TURN_SPEED);
    delay(240);
    halt();
}

void turnSlightCCW() {
    setSpeed(-TURN_SPEED, TURN_SPEED);
    delay(240);
    halt();
}

// ==========================================
// LED FUNCTIONS
// ==========================================

void clearLeds() {
    digitalWrite(redLedLeft, LOW);
    digitalWrite(redLedMid, LOW);
    digitalWrite(redLedRight, LOW);
}

// ==========================================
// IR DISTANCE MEASUREMENT — ORIGINAL MAPPING
// ==========================================

float getDistance(int ledPin, int rxPin) {
    int hits = 0;

    for (long f = 38000; f <= 45000; f += 1000) {
        tone(ledPin, f);
        delay(1);

        if (digitalRead(rxPin) == 0) {
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
    if (hits == 1) return 12.0;

    return 12.0;
}

// ==========================================
// SCENARIO LED DISPLAY
// ==========================================

void displayScenario(int scenario) {
    if (scenario == 8) {
        digitalWrite(redLedLeft, LOW);
        digitalWrite(redLedMid, LOW);

        for (int i = 0; i < 5; i++) {
            digitalWrite(redLedRight, HIGH);
            delay(500);
            digitalWrite(redLedRight, LOW);
            delay(500);
        }
    }
    else if (scenario == 9) {
        digitalWrite(redLedLeft, LOW);
        digitalWrite(redLedRight, LOW);

        for (int i = 0; i < 5; i++) {
            digitalWrite(redLedMid, HIGH);
            delay(500);
            digitalWrite(redLedMid, LOW);
            delay(500);
        }
    }
    else {
        digitalWrite(
            redLedLeft,
            (scenario & 4) ? HIGH : LOW
        );

        digitalWrite(
            redLedMid,
            (scenario & 2) ? HIGH : LOW
        );

        digitalWrite(
            redLedRight,
            (scenario & 1) ? HIGH : LOW
        );

        delay(1000);
    }
}

// ==========================================
// TASK SELECTION
// ==========================================

// 1 = Blue: stop at first front wall.
// 2 = Purple: turn once, stop at next front wall.
// 3 = Yellow: follow corners, stop at dead end.
// 4 = Green: reach dead end, turn around, return.
// 5 = Extension: explore until calibrated two-wall dead end.
const int TASK = 1;

// NEW measurement settings.
//
// Serial Monitor at 9600 baud prints "Dead-end travel".
//
// Measure repeated known TWO-wall dead ends and enter a range
// containing those readings, but excluding shorter AND longer ends.
//
// Zero means uncalibrated: task 5 will not declare a finish.
const unsigned long LONG_MIN = 0;
const unsigned long LONG_MAX = 0;

/*
Movement functions and scenario movements retain your original
speeds and timings. Detection and task control have changed.

Travel is an approximate forward-equivalent motor time, NOT cm.
Stops and pure rotations do not add travel. Arc estimates use
mean wheel commands. Slip and motor nonlinearity cause error.

Calibrate LONG_MIN/MAX using this exact program. Test known
one-, two- and three-wall dead ends. If their ranges overlap,
this timing method cannot reliably distinguish exactly two walls.

Extension measurement starts after leaving a junction and
reacquiring both side walls. An initial partial corridor is
never accepted as the finish.

Right-hand priority is intended for the pictured tree maze.
It does not guarantee exploration of arbitrary cyclic mazes.

Green estimates the starting position from outward travel.
It has no physical marker to detect the exact starting position.

Original long movements remain open-loop. Physically verify
wall clearance, especially during the 180 turn in Part 2.
Also verify revised detection for angled starts and junctions.
*/

// ==========================================
// TASK STATE
// ==========================================

bool finished = false;
bool returning = false;

bool junctionActive = false;
bool lengthKnown = false;
bool measureNextCorridor = false;

unsigned int turnsOut = 0;
unsigned int turnsBack = 0;

unsigned long travel = 0;
unsigned long firstCorridor = 0;

float dMid = 12;
float dLeft = 12;
float dRight = 12;

// ==========================================
// SCENARIO DETECTION
// ==========================================

int detectScenario() {
    dMid = getDistance(irLedMid, irReceiverMid);
    dLeft = getDistance(irLedLeft, irReceiverLeft);
    dRight = getDistance(irLedRight, irReceiverRight);

    Serial.print("Front: ");
    Serial.print(dMid);

    Serial.print(" Left: ");
    Serial.print(dLeft);

    Serial.print(" Right: ");
    Serial.println(dRight);

    // getDistance() never returns above 12.
    // Treat 12 as an opening indication, not an exact distance.
    bool openLeft = dLeft >= 12.0;
    bool openRight = dRight >= 12.0;

    // Wall ahead.
    if (dMid <= 10.0) {
        // Very close to one side: original angled-start escape.
        if (dLeft <= 3.5 && openRight) {
            return 7;
        }

        if (dRight <= 3.5 && openLeft) {
            return 8;
        }

        // Dead end.
        if (dLeft <= 9.0 && dRight <= 9.0) {
            return 4;
        }

        // Right corner.
        if (dLeft <= 9.0 && openRight) {
            return 2;
        }

        // Left corner.
        if (dRight <= 9.0 && openLeft) {
            return 3;
        }

        // T-junction: front blocked, both sides open.
        if (openLeft && openRight) {
            return 10;
        }

        return 0;
    }

    // Corridor open ahead: close-wall corrections first.
    if (dLeft <= 3.5 && dRight >= 6.5) {
        return 5;
    }

    if (dRight <= 3.5 && dLeft >= 6.5) {
        return 6;
    }

    // Extension handles branches before reaching a front wall.
    if (TASK == 5 && !junctionActive) {
        if (openRight) {
            return 9;
        }

        if (openLeft) {
            return 11;
        }
    }

    // Continue through an opening already being handled.
    if (openLeft || openRight) {
        return 1;
    }

    if (dLeft <= 8.5 && dRight <= 8.5) {
        return 1;
    }

    float sumLeftRight = dLeft + dRight;

    if (dLeft <= 8.0 &&
        dRight > 9.0 &&
        sumLeftRight > 17) {
        return 7;
    }

    if (dRight <= 8.0 &&
        dLeft > 9.0 &&
        sumLeftRight > 17) {
        return 8;
    }

    return 0;
}

// ==========================================
// SCENARIO EXECUTION — ORIGINAL MOVEMENTS
// ==========================================

void executeScenarioEscape(int scenario) {
    switch (scenario) {
        case 0:
            halt();
            break;

        case 1:
            driveForward5cm();
            break;

        case 2:
            turnRight90();
            driveForward(600);
            break;

        case 3:
            turnLeft90();
            driveForward(600);
            break;

        case 4:
            spin180();
            driveForward5cm();
            break;

        case 5:
            // Hugging left wall: arc right, then straighten.
            setSpeed(ARC_FAST, ARC_SLOW);
            delay(780);

            setSpeed(ARC_MEDIUM, FORWARD_SPEED);
            delay(480);

            driveForward5cm();
            break;

        case 6:
            // Hugging right wall: arc left, then straighten.
            setSpeed(35, ARC_SHARP);
            delay(480);

            setSpeed(FORWARD_SPEED, ARC_SLOW);
            delay(880);

            driveForward5cm();
            break;

        case 7:
            turnSlightCW();
            driveForward5cm();
            break;

        case 8:
            turnSlightCCW();
            driveForward5cm();
            break;

        case 9:
            // Original alignment manoeuvre, then right turn.
            setSpeed(35, ARC_SHARP);
            delay(380);

            setSpeed(FORWARD_SPEED, ARC_SLOW);
            delay(680);

            turnRight90();
            driveForward(600);
            break;

        default:
            halt();
            break;
    }
}

// ==========================================
// TASK CONTROL
// ==========================================

void finishTask(const char *message) {
    halt();
    finished = true;
    Serial.println(message);
}

// Approximate forward-equivalent travel.
// These values track movement; they do not change motor timings.
unsigned long forwardTravel(int scenario) {
    switch (scenario) {
        case 1:
            return 440;

        case 5:
            // 780*(90+40)/142 + 480*(30+71)/142 + 440
            return 1495;

        case 6:
            // 480*(35+97)/142 + 880*(71+40)/142 + 440
            return 1574;

        case 7:
        case 8:
            return 440;

        default:
            return 0;
    }
}

void recordTurn() {
    if (!returning) {
        if (turnsOut == 0) {
            firstCorridor = travel;
        }

        turnsOut++;
    } else {
        turnsBack++;
    }

    // Original turn scenario already drove forward for 600 ms.
    travel = 600;

    measureNextCorridor = true;
    junctionActive = true;
    lengthKnown = false;
}

bool isLongDeadEnd() {
    return lengthKnown &&
           LONG_MIN > 0 &&
           LONG_MAX > LONG_MIN &&
           travel >= LONG_MIN &&
           travel <= LONG_MAX;
}

void runTask(int scenario) {
    if (scenario == 0) {
        halt();
        Serial.println("Unknown: stopped, checking again.");
        return;
    }

    bool frontEvent = scenario == 2 ||
                      scenario == 3 ||
                      scenario == 4 ||
                      scenario == 10;

    // Blue: stop at the first front wall.
    if (frontEvent && TASK == 1) {
        finishTask("BLUE complete: first front wall.");
        return;
    }

    // Purple: stop at the front wall after one turn.
    if (frontEvent && TASK == 2 && turnsOut >= 1) {
        finishTask("PURPLE complete: wall after one turn.");
        return;
    }

    // Dead end.
    if (scenario == 4) {
        Serial.print("Dead-end travel: ");
        Serial.println(travel);

        if (TASK == 3) {
            finishTask("YELLOW complete.");
            return;
        }

        if (TASK == 4) {
            if (returning) {
                finishTask(
                    "Return wall reached; verify start position."
                );
                return;
            }

            if (turnsOut == 0) {
                finishTask(
                    "No outward turns recorded: check detection."
                );
                return;
            }

            returning = true;
        }

        if (TASK == 5 && isLongDeadEnd()) {
            finishTask(
                "Two-wall finish candidate: stopped facing forward."
            );
            return;
        }

        if (TASK == 5 && LONG_MIN == 0) {
            Serial.println(
                "Length uncalibrated: cannot declare finish."
            );
        }

        // Original 180 turn followed by original forward step.
        executeScenarioEscape(4);

        travel = 440;
        measureNextCorridor = false;
        lengthKnown = false;
        junctionActive = true;
        return;
    }

    // Corners, right branches and T-junctions.
    if (scenario == 2 ||
        scenario == 3 ||
        scenario == 9 ||
        scenario == 10) {

        // At a T-junction, choose right.
        if (scenario == 10) {
            executeScenarioEscape(2);
        } else {
            executeScenarioEscape(scenario);
        }

        recordTurn();
        return;
    }

    // Left branch with the corridor still open ahead.
    if (scenario == 11) {
        // Right-hand priority: straight before left.
        // This still starts a new segment for length measurement.
        measureNextCorridor = true;
        junctionActive = true;
        lengthKnown = false;

        executeScenarioEscape(1);
        travel += 440;
        return;
    }

    // Corridor travel or original alignment corrections.
    executeScenarioEscape(scenario);
    travel += forwardTravel(scenario);
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

    halt();

    if (TASK < 1 || TASK > 5 ||
        ((LONG_MIN != 0 || LONG_MAX != 0) &&
         (LONG_MIN == 0 || LONG_MAX <= LONG_MIN))) {
        finishTask("Invalid TASK or length range.");
    }

    delay(5000);
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
    halt();

    if (finished) {
        return;
    }

    clearLeds();
    delay(500);

    int scenario = detectScenario();

    // Allow another junction decision after both side walls return.
    if (junctionActive && dLeft <= 9.0 && dRight <= 9.0) {
        junctionActive = false;

        // Begin extension length measurement only with clear space ahead.
        if (TASK == 5 &&
            measureNextCorridor &&
            dMid > 10.0) {
            travel = 0;
            lengthKnown = true;
        }
    }

    // Green: stop near the original starting position.
    if (TASK == 4 &&
        returning &&
        turnsBack >= turnsOut &&
        travel >= firstCorridor) {
        finishTask("GREEN: estimated start position reached.");
        return;
    }

    // Reuse existing LED patterns for added junction scenarios.
    if (scenario == 10) {
        displayScenario(2);
    } else if (scenario == 11) {
        displayScenario(1);
    } else {
        displayScenario(scenario);
    }

    Serial.print("Scenario: ");
    Serial.println(scenario);

    runTask(scenario);

    halt();
    delay(1000);
}
