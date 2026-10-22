
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
// MOTOR CALIBRATION
// ==========================================

const int NEUTRAL = 1500;

// Adjust these to calibrate each wheel.
// Positive offset = faster wheel magnitude.
const int LEFT_OFFSET = 0;
const int RIGHT_OFFSET = 4;

const int FORWARD_SPEED = 71;
const int REVERSE_SPEED = 71;
const int TURN_SPEED = 71;

// Arc speeds
const int ARC_FAST = 90;
const int ARC_SLOW = 40;
const int ARC_MEDIUM = 30;
const int ARC_SHARP = 97;

// ==========================================
// MOTOR FUNCTIONS
// ==========================================

void setMotors(int leftSpeed, int rightSpeed) {
    servoLeft.writeMicroseconds(leftSpeed);
    servoRight.writeMicroseconds(rightSpeed);
}

// Positive speed = forward
// Negative speed = backward
// Zero = stop
//
// Both wheels can have independent speeds.
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

// Both wheels at the same logical speed
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
    // 180 degree dead-end turn
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
// IR DISTANCE MEASUREMENT
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
// SCENARIO DETECTION
// ==========================================

int detectScenario() {
    float dMid = getDistance(irLedMid, irReceiverMid);
    float dLeft = getDistance(irLedLeft, irReceiverLeft);
    float dRight = getDistance(irLedRight, irReceiverRight);

    float sumLeftRight = dLeft + dRight;

    Serial.println("Front dist: " + String(dMid));
    
    Serial.println("Right dist: " + String(dRight));
    
    Serial.println("Left dist: " + String(dLeft));

    Serial.println("Sum left right: " + String(sumLeftRight));

    // Group 1: Wall in front
    if (dMid <= 10.0) {

        // Scenario 4: Dead end
        if (dLeft <= 9.0 && dRight <= 9.0) {
            return 4;
        }

        // Scenario 2: Right turn
        if (dLeft <= 9.0 && dRight > 12.0 && sumLeftRight <= 17) {
            return 2;
        }

        // Scenario 3: Left turn
        if (dRight <= 9.0 && dLeft > 12.0) {
            return 3;
        }

        // Scenario 7: Angled towards left wall
        if (dLeft <= 8.0 && dRight > 9.0 && sumLeftRight > 17) {
            return 7;
        }

        // Scenario 8: Angled towards right wall
        if (dRight <= 8.0 && dLeft > 9.0 && sumLeftRight > 17) {
            return 8;
        }
    }

    // Group 2: Corridor open ahead
    if (dMid > 10.0) {

        if (dLeft <= 8.0 && dRight > 10.0 && sumLeftRight > 17) {
            return 7;
        }

        if (dRight <= 8.0 && dLeft > 9.0 && sumLeftRight > 17) {
            return 8;
        }

        if (dLeft <= 10 && dRight >= 12) {
            return 9;
        }

        // Scenario 5: Hugging left wall
        if (dLeft <= 3.5 && dRight >= 6.5) {
            return 5;
        }

        // Scenario 6: Hugging right wall
        if (dRight <= 3.5 && dLeft >= 6.5 && sumLeftRight <= 17) {
            return 6;
        }

        // Scenario 1: Centered corridor
        if (dLeft <= 8.5 && dRight <= 8.5) {
            return 1;
        }
    }

    return 0;
}

// ==========================================
// SCENARIO EXECUTION
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
            // Hugging left wall
            // Arc right, then straighten

            setSpeed(ARC_FAST, ARC_SLOW);
            delay(780);

            setSpeed(ARC_MEDIUM, FORWARD_SPEED);
            delay(480);

            driveForward5cm();
            break;

        case 6:
            // Hugging right wall
            // Arc left, then straighten

            setSpeed(35, ARC_SHARP);
            delay(480);

            setSpeed(FORWARD_SPEED, ARC_SLOW);
            delay(880);

            driveForward5cm();
            break;

        case 7:
            // Angled towards left wall
            turnSlightCW();
            driveForward5cm();
            break;

        case 8:
            // Angled towards right wall
            turnSlightCCW();
            driveForward5cm();
            break;

        case 9:
            // Adjust alignment then turn right

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
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop() {
    halt();
    clearLeds();

    delay(500);

    int scenario = detectScenario();

    displayScenario(scenario);

    Serial.println(scenario);

    executeScenarioEscape(scenario);

    Serial.println("Next loop ========");

    halt();
    delay(1000);
}
