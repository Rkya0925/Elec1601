#include <Servo.h>

Servo wheelLeft, wheelRight;

int cw = 1500;
int ccw = 1500;
int offset = 5;
int scenario = 0;

const int ledRight = A0;
const int ledMid = A1;
const int ledLeft = A2;

// These two distances still need physical calibration.
const int forward5cmTime = 700;
const int reverseLevelTime = 1000;

// --------------------------------------------------
// MOVEMENT — YOUR EXISTING CALIBRATIONS
// --------------------------------------------------

void setSpeed(int speed) {
  cw = 1500 - speed;
  ccw = 1500 + speed;
}

void stop() {
  wheelLeft.write(1500);
  wheelRight.write(1500);
}

void forward() {
  wheelLeft.write(ccw - offset);
  wheelRight.write(cw);
}

void reverse() {
  wheelLeft.write(cw + offset);
  wheelRight.write(ccw);
}

void left(bool slight) {
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);

  delay(slight ? 150 : 600);
  stop();
}

void right(bool slight) {
  wheelLeft.write(ccw);
  wheelRight.write(ccw);

  delay(slight ? 250 : 600);
  stop();
}

void turn180() {
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);

  delay(1200);
  stop();
}

// --------------------------------------------------
// LEDs — RIGHT, MIDDLE, LEFT
// --------------------------------------------------

void displayLEDs(int right, int mid, int left) {
  digitalWrite(ledRight, right);
  digitalWrite(ledMid, mid);
  digitalWrite(ledLeft, left);
}

// --------------------------------------------------
// YOUR IR SENSOR FUNCTIONS
// --------------------------------------------------

int irDetect(int led, int receiver, long frequency) {
  tone(led, frequency);
  delay(1);

  int reading = digitalRead(receiver);

  noTone(led);
  delay(1);

  return reading;
}

int irDistance(int led, int receiver) {
  int distance = 0;

  for (long f = 38000; f <= 42000; f += 1000) {
    distance += irDetect(led, receiver, f);
  }

  return distance;
}

// --------------------------------------------------
// SCENARIO DETECTION
// l = left, r = right, f = front
//
// Scores are NOT centimetres.
// 5 = no wall detected.
// 4 = provisional distant-wall reading.
// Lower scores are treated as closer.
// --------------------------------------------------

int detectScenario(int l, int r, int f) {
  if (f < 5) {
    // 7: close left/front, distant right wall.
    if (l <= 2 && r == 4 && r - l >= 3) {
      return 7;
    }

    // 8: close right/front, distant left wall.
    if (r <= 2 && l == 4 && l - r >= 3) {
      return 8;
    }

    // 2: right open, left and front walls.
    if (r == 5 && l < 5) {
      return 2;
    }

    // 3: left open, right and front walls.
    if (l == 5 && r < 5) {
      return 3;
    }

    // 4: all three walls detected.
    if (l < 5 && r < 5) {
      return 4;
    }
  } else if (l < 5 && r < 5) {
    // 5: closer to the left wall.
    if (r - l >= 3) {
      return 5;
    }

    // 6: closer to the right wall.
    if (l - r >= 3) {
      return 6;
    }

    // 1: similar side readings, front clear.
    if (abs(l - r) <= 1) {
      return 1;
    }
  }

  // Unknown position: do not move.
  return 0;
}

// --------------------------------------------------
// SCENARIO MOVEMENTS
// Every scenario finishes with stop().
// --------------------------------------------------

void runScenario(int number) {
  stop();

  switch (number) {
    case 0:
      displayLEDs(LOW, LOW, LOW);
      break;

    case 1:
      displayLEDs(HIGH, LOW, LOW);

      setSpeed(50);
      forward();
      delay(forward5cmTime);
      break;

    case 2:
      displayLEDs(LOW, HIGH, LOW);

      // Rotate right, then enter the corridor.
      setSpeed(30);
      right(false);

      forward();
      delay(700);
      break;

    case 3:
      displayLEDs(HIGH, HIGH, LOW);

      // Rotate left, then enter the corridor.
      setSpeed(30);
      left(false);

      forward();
      delay(700);
      break;

    case 4:
      displayLEDs(LOW, LOW, HIGH);

      setSpeed(30);
      turn180();

      setSpeed(50);
      forward();
      delay(forward5cmTime);
      break;

    case 5:
      displayLEDs(HIGH, LOW, HIGH);

      // Turn away from the left wall.
      setSpeed(30);
      right(true);

      setSpeed(50);
      forward();
      delay(1000);
      stop();

      // Your original return-turn speed.
      setSpeed(20);
      left(true);

      setSpeed(50);
      reverse();
      delay(reverseLevelTime);
      break;

    case 6:
      displayLEDs(LOW, HIGH, HIGH);

      // Turn away from the right wall.
      setSpeed(30);
      left(true);

      setSpeed(50);
      forward();
      delay(1000);
      stop();

      setSpeed(30);
      right(true);

      setSpeed(50);
      reverse();
      delay(reverseLevelTime);
      break;

    case 7:
      displayLEDs(HIGH, HIGH, HIGH);

      setSpeed(30);
      right(true);
      break;

    case 8:
      // Flash right LED while stopped.
      displayLEDs(HIGH, LOW, LOW);
      delay(1000);

      displayLEDs(LOW, LOW, LOW);
      delay(1000);

      setSpeed(30);
      left(true);
      break;

    default:
      displayLEDs(LOW, LOW, LOW);
      break;
  }

  stop();
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------

void setup() {
  pinMode(10, OUTPUT);
  pinMode(11, INPUT);

  pinMode(2, OUTPUT);
  pinMode(3, INPUT);

  pinMode(6, OUTPUT);
  pinMode(7, INPUT);

  pinMode(ledRight, OUTPUT);
  pinMode(ledMid, OUTPUT);
  pinMode(ledLeft, OUTPUT);

  wheelLeft.attach(13);
  wheelRight.attach(12);

  Serial.begin(9600);

  setSpeed(0);
  stop();
  displayLEDs(LOW, LOW, LOW);

  // Time to position the robot.
  delay(5000);
}

// --------------------------------------------------
// MAIN LOOP
// --------------------------------------------------

void loop() {
  int l = irDistance(10, 11);
  int r = irDistance(2, 3);
  int f = irDistance(6, 7);

  scenario = detectScenario(l, r, f);

  Serial.print("Left: ");
  Serial.println(l);

  Serial.print("Right: ");
  Serial.println(r);

  Serial.print("Front: ");
  Serial.println(f);

  Serial.print("Scenario: ");
  Serial.println(scenario);

  runScenario(scenario);

  // Robot stays stopped before checking again.
  delay(1000);
}
