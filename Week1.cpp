
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

void setMotors(int leftSpeed, int rightSpeed) {
    servoLeft.writeMicroseconds(leftSpeed);
    servoRight.writeMicroseconds(rightSpeed);
}

void halt() {
    setMotors(1500, 1500);
}

void driveForward5cm() {
    setMotors(1571, 1407);
    delay(440);
    halt();
}

void driveBackward5cm() {
    setMotors(1407, 1571);
    delay(220);
    halt();
}

void driveForward(int ms) {
    setMotors(1571, 1407);
    delay(ms);
    halt();
}

void turnLeft90() {
    setMotors(1418, 1409);
    delay(305);

    driveForward(100);

    setMotors(1418, 1409);
    delay(290);

    halt();
}

void turnRight90() {
    setMotors(1571, 1592);
    delay(305);

    driveForward(100);

    setMotors(1571, 1592);
    delay(270);

    halt();
}

void spin180() {
    // 180 degree dead-end turn
    setMotors(1571, 1592);
    delay(1150);
    halt();
}

void turnSuperSlightCW() {
    // ~25 degree clockwise square-up
    setMotors(1571, 1592);
    delay(120);
    halt();
}

void turnSuperSlightCCW() {
    // ~25 degree counter-clockwise square-up
    setMotors(1418, 1407);
    delay(120);
    halt();
}

void turnSlightCW() {
    // ~25 degree clockwise square-up
    setMotors(1571, 1592);
    delay(240);
    halt();
}

void turnSlightCCW() {
    // ~25 degree counter-clockwise square-up
    setMotors(1418, 1407);
    delay(240);
    halt();
}

void clearLeds() {
    digitalWrite(redLedLeft, LOW);
    digitalWrite(redLedMid, LOW);
    digitalWrite(redLedRight, LOW);
}

// Multi-frequency distance estimator using calibration table
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

    if (hits >= 8) return 2.0;   // Very close
    if (hits == 7) return 5.0;    // Close
    if (hits == 6) return 6.0;
    if (hits == 5) return 6.5;
    if (hits == 4) return 7.0;
    if (hits == 3) return 8.5;
    if (hits == 2) return 10.0;
    if (hits == 1) return 12.0;

    return 25.0; // Open
}

// Display scenario in binary on LEDs for 5 seconds
void displayScenario(int scenario) {
    if (scenario == 8) {
        // Scenario 8: Flash A0 once per second for 5 seconds
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
        // Scenario 8: Flash A0 once per second for 5 seconds
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
        // Binary display
        digitalWrite(redLedLeft, (scenario & 4) ? HIGH : LOW);
        digitalWrite(redLedMid, (scenario & 2) ? HIGH : LOW);
        digitalWrite(redLedRight, (scenario & 1) ? HIGH : LOW);

        delay(1000);
    }
}

int detectScenario() {
    float dMid = getDistance(irLedMid, irReceiverMid);
    float dLeft = getDistance(irLedLeft, irReceiverLeft);
    float dRight = getDistance(irLedRight, irReceiverRight);

    // Group 1: Wall detected immediately in front (dMid <= 10cm)
    if (dMid <= 10.0) {

        // Scenario 4: Dead End (All 3 sides enclosed)
        if (dLeft <= 9.0 && dRight <= 9.0 && dMid >= 5) {
            return 4;
        }

        // Scenario 2: Ideal Right Turn (Front blocked, Left wall, Right open)
        if (dLeft <= 9.0 && dRight > 12.0) {
            return 2;
        }

        // Scenario 3: Ideal Left Turn (Front blocked, Right wall, Left open)
        if (dRight <= 9.0 && dLeft > 12.0) {
            return 3;
        }

        // Scenario 7: Angled ~30 deg towards left wall
        if (dLeft <= 5.0 && dRight > 9.0 && dMid < 5) {
            return 7;
        }

        // Scenario 8: Angled ~30 deg towards right wall
        if (dRight <= 5.0 && dLeft > 9.0) {
            return 8;
        }
    }

    // Group 2: Corridor is open ahead (dMid > 10cm)
    if (dMid > 10.0) {

        if (dLeft <= 5.0 && dRight > 10.0) {
            return 7;
        }

        if (dRight <= 5.0 && dLeft > 9.0) {
            return 8;
        }

        if (dLeft <= 10 && dRight > 12) {
            return 9;
        }

        // Scenario 5: Bad Starting - Hugging Left Wall
        if (dLeft <= 3.5 && dRight >= 6.5) {
            return 5;
        }

        // Scenario 6: Bad Starting - Hugging Right Wall
        if (dRight <= 3.5 && dLeft >= 6.5) {
            return 6;
        }

        // Scenario 1: Centered in Long Corridor
        if (dLeft <= 8.5 && dRight <= 8.5) {
            return 1;
        }
    }

    return 0; // Unknown
}

void executeScenarioEscape(int scenario) {
    switch (scenario) {

        case 0:
            // Scenario 0: Must not move at all
            halt();
            break;

        case 1:
            // Scenario 1: Move 5cm forward and stop
            driveForward5cm();
            break;

        case 2:
            // Scenario 2: Rotate 90° right, advance into opening, stop
            turnRight90();
            driveForward(600);
            break;

        case 3:
            // Scenario 3: Rotate 90° left, advance into opening, stop
            turnLeft90();
            driveForward(600);
            break;

        case 4:
            // Scenario 4: Rotate 180°, drive forward at least 5cm, stop
            spin180();
            driveForward5cm();
            break;

        case 5:
            // Scenario 5: Hugging left wall
            // Arc steer away forward (no tail swing)
            setMotors(1590, 1445);
            delay(780);

            setMotors(1530, 1407);
            delay(480);

            driveForward5cm();
            break;

        case 6:
            // Scenario 6: Hugging right wall
            // Arc steer away forward (no tail swing)
            setMotors(1535, 1375);
            delay(480);

            setMotors(1571, 1445);
            delay(880);

            driveForward5cm();
            break;

        case 7:
            // Scenario 7: Angled into left wall
            // Clockwise square up + advance
            turnSlightCW();
            driveForward5cm();
            
            break;

        case 8:
            // Scenario 8: Angled into right wall
            // Anti-clockwise square up + advance
            turnSlightCCW();
            driveForward5cm();
            break;

        case 9:
            // Scenario 2: Rotate 90° right, advance into opening, stop
            setMotors(1535, 1375);
            delay(380);

            setMotors(1571, 1445);
            delay(680);

            turnRight90();
            driveForward(600);
            break;

        default:
            halt();
            break;
    }
}

void setup() {
    pinMode(irLedLeft, OUTPUT);
    pinMode(irReceiverLeft, INPUT);
    pinMode(redLedLeft, OUTPUT);

    pinMode(irLedMid, OUTPUT);
    pinMode(irReceiverMid, INPUT);
    pinMode(redLedMid, OUTPUT);

    pinMode(irLedRight, OUTPUT);
    pinMode(irReceiverRight, INPUT);
    pinMode(redLedRight, OUTPUT);

    servoLeft.attach(servoLeftPin);
    servoRight.attach(servoRightPin);

    halt();
}

void loop() {
    halt();
    clearLeds();

    delay(500);

    // Detect the scenario
    int scenario = detectScenario();

    // Display scenario on LEDs for 5 seconds
    // (mandatory hold for assessment)
    displayScenario(scenario);

    Serial.println(scenario);

    // Execute recovery escape maneuver
    executeScenarioEscape(scenario);

    // Brief pause before cycling back
    halt();
    delay(1000);
}
