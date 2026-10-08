#include <Servo.h>

Servo wheelLeft;
Servo wheelRight;

int cw = 1500;
int ccw = 1500;

int left_reading = 0;
int right_reading = 0;

int offset = 5;
bool turning = false;
int leftOrRight = 1;
int thresholdValue = 3;

int scenario = 0;

// 0 = Unknown scenario
// 1 = Middle of a long corridor
// 2 = Ideal position for right turn
// 3 = Ideal position for left turn
// 4 = Dead end
// 5 = Close to left wall, parallel
// 6 = Close to right wall, parallel
// 7 = Close to left wall, approximately 30 degrees
// 8 = Close to right wall, approximately 30 degrees

const int ledRight = A0;
const int ledMid = A1;
const int ledLeft = A2;

void setSpeed(int speed) {
  if (speed == 0) {
    cw = 1500;
    ccw = 1500;
  } else {
    cw = 1500 - speed;
    ccw = 1500 + speed;
  }
}

void stop() {
  wheelLeft.write(1500);
  wheelRight.write(1500);
}

void reverse() {
  wheelLeft.write(cw + offset);
  wheelRight.write(ccw);
}

void left(bool isSlight) {
  Serial.println("Moving left");
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);

  turning = true;

  if (isSlight) {
    delay(600);
  } else {
    delay(250);
  }

  stop();
  turning = false;
}

void right(bool isSlight) {
  Serial.println("Moving right");
  wheelLeft.write(ccw);
  wheelRight.write(ccw);

  turning = true;

  if (isSlight) {
    delay(600);
  } else {
    delay(250);
  }

  stop();
  turning = false;
}

void turn180() {
  Serial.println("Turning 180");
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);

  turning = true;

  delay(1200);

  stop();
  turning = false;
}

void forward() {
  Serial.println("Moving forward");
  wheelLeft.write(ccw - offset);
  wheelRight.write(cw);
}

void displayLEDs(int right, int mid, int left) {
  digitalWrite(ledRight, right);
  digitalWrite(ledMid, mid);
  digitalWrite(ledLeft, left);
}

// Stop and display the detected scenario before moving.
void showScenario(int detectedScenario) {
  stop();

  switch (detectedScenario) {
    case 0:
      displayLEDs(LOW, LOW, LOW);
      break;

    case 1:
      displayLEDs(HIGH, LOW, LOW);
      break;

    case 2:
      displayLEDs(LOW, HIGH, LOW);
      break;

    case 3:
      displayLEDs(HIGH, HIGH, LOW);
      break;

    case 4:
      displayLEDs(LOW, LOW, HIGH);
      break;

    case 5:
      displayLEDs(HIGH, LOW, HIGH);
      break;

    case 6:
      displayLEDs(LOW, HIGH, HIGH);
      break;

    case 7:
      displayLEDs(HIGH, HIGH, HIGH);
      break;

    case 8:
      // Flash the right LED for six seconds.
      // The robot remains stopped throughout.
      for (int i = 0; i < 3; i++) {
        displayLEDs(HIGH, LOW, LOW);
        delay(1000);
        displayLEDs(LOW, LOW, LOW);
        delay(1000);
      }
      return;

    default:
      displayLEDs(LOW, LOW, LOW);
      break;
  }

  delay(5000);
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A3, INPUT);

  pinMode(ledRight, OUTPUT);
  pinMode(ledMid, OUTPUT);
  pinMode(ledLeft, OUTPUT);

  wheelLeft.attach(13);
  wheelRight.attach(12);

  Serial.begin(9600);

  setSpeed(0);
  stop();

  delay(1000);
}

void loop() {
  float leftDist = 0;
  float rightDist = 0;
  float frontDist = 0;

  leftDist = irDistance(10, 11);
  rightDist = irDistance(2, 3);
  frontDist = irDistance(6, 7);

  Serial.println("Left dist:  " + String(leftDist));
  Serial.println("Front dist:  " + String(frontDist));
  Serial.println("Right dist:  " + String(rightDist));

  scenario = 0;

  if (turning == false) {
    if (leftDist >= 5 && rightDist >= 5 && frontDist >= 5) {
      scenario = 0;
    } else {
      if (leftDist >= 5 && rightDist < 5) {
        if (rightDist <= 2 && frontDist <= 4) {
          scenario = 8;
        } else {
          if (rightDist - leftDist <= -4) {
            scenario = 6;
          } else {
            scenario = 3;
          }
        }
      } else if (rightDist >= 5 && leftDist < 5) {
        if (leftDist <= 2 && frontDist <= 4) {
          scenario = 7;
        } else {
          if (leftDist - rightDist <= -4) {
            scenario = 5;
          } else {
            scenario = 2;
          }
        }
      } else if ((rightDist <= 5) && (leftDist <= 5)) {
        if (frontDist < 5) {
          scenario = 4;
        } else {
          scenario = 1;
        }
      }
    }

    // Outside the else: this now runs for scenario 0 too.
    showScenario(scenario);

    switch (scenario) {
      case 0:
        setSpeed(0);
        stop();
        break;

      case 1:
        setSpeed(50);
        forward();
        break;

      case 2:
        setSpeed(50);
        forward();
        delay(500);

        stop();

        setSpeed(30);
        right(false);
        forward();
        delay(300);
        break;

      case 3:
        setSpeed(50);
        forward();
        delay(500);

        stop();

        setSpeed(30);
        left(false);
        forward();
        delay(300);
        break;

      case 4:
        setSpeed(30);
        turn180();
        break;

      case 5:
        setSpeed(30);
        right(true);
        break;

      case 6:
        setSpeed(30);
        left(true);
        break;

      case 7:
        setSpeed(30);
        right(true);
        break;

      case 8:
        setSpeed(30);
        left(true);
        break;

      default:
        stop();
        displayLEDs(LOW, LOW, LOW);
        break;
    }
  }

  Serial.println(scenario);
  delay(1000);
}

// IR Object Detection Function
int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);

  int ir = digitalRead(irReceiverPin);

  noTone(irLedPin);
  delay(1);

  return ir;
}

// IR distance measurement function
int irDistance(int irLedPin, int irReceiverPin) {
  int distance = 0;

  for (long f = 38000; f <= 42000; f += 1000) {
    distance += irDetect(irLedPin, irReceiverPin, f);
  }

  return distance;
}
