// C++ code
//

#include <Servo.h>

Servo wheelLeft;
Servo wheelRight;

int cw = 1500;
int ccw = 1500;

int left_reading = 0;
int right_reading = 0;

int offset = 5;

bool adjustingStartingPosition = false;
bool turning = false;

int leftOrRight = 1;

int thresholdValue = 3;



  // Select the scenario here:
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
const int ledMid   = A1;
const int ledLeft  = A2;

void setSpeed(int speed){
  if (speed == 0){
    cw = 1500;
    ccw = 1500;
  } else {
    cw = 1500 - speed;
    ccw = 1500 + speed;
  }
}

void stop(){
  wheelLeft.write(1500);
  wheelRight.write(1500);
}

void reverse(){
  wheelLeft.write(cw + offset);
  wheelRight.write(ccw);
}

void left(bool isSlight){
  Serial.println("Moving left");
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);
 
  turning = true;
 
  if (isSlight){
    delay(250);
  } else {
    delay(600);
  }
 
  turning = false;

  stop();
}

void right(bool isSlight){
  Serial.println("Moving right");
  wheelLeft.write(ccw);
  wheelRight.write(ccw);
 
  turning = true;
 
  if (isSlight){
    delay(250);
  } else {
    delay(600);
  }
 
  turning = false;

  stop();
}

void turn180(){
  Serial.println("Turning 180");
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);
 
  turning = true;
 
  delay(1200);
 
  turning = false;

  stop();
}

void forward(){
  Serial.println("Moving forward");
 
  wheelLeft.write(ccw - offset);
  wheelRight.write(cw);
}

void displayLEDs(int left, int mid, int right) {
  digitalWrite(ledRight, right);
  digitalWrite(ledMid, mid);
  digitalWrite(ledLeft, left);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A3, INPUT);
  pinMode(A2, INPUT);
 
  wheelLeft.attach(13);
  wheelRight.attach(12);
 
  //wheelLeft.write(ccw);
  //wheelRight.write(ccw);
 
  Serial.begin(9600);

  setSpeed(0);
 
  //setSpeed(50);
  //forward();

  //

  float leftDist = 0;
  float rightDist = 0;
  float frontDist = 0;                               // 0.5 second delay - just long enough to see the LED blink

  leftDist = irDistance(10, 11);
  rightDist = irDistance(2, 3);
  frontDist = irDistance(6, 7);

  //

  if (rightDist <= 2){
    scenario = 6;

    adjustingStartingPosition = true;
  }
  if (leftDist <= 2){
    scenario = 5;

    adjustingStartingPosition = true;
  }

  //
  
  delay(1000);
}

void loop()
{
  float leftDist = 0;
  float rightDist = 0;
  float frontDist = 0;                               // 0.5 second delay - just long enough to see the LED blink

  leftDist = irDistance(10, 11);
  rightDist = irDistance(2, 3);
  frontDist = irDistance(6, 7);

  Serial.println("Left dist:  " + String(leftDist));
  Serial.println("Front dist:  " + String(frontDist));
  Serial.println("Right dist:  " + String(rightDist));

  pinMode(ledRight, OUTPUT);
  pinMode(ledMid, OUTPUT);
  pinMode(ledLeft, OUTPUT);

  if (adjustingStartingPosition == false){
    scenario = 0;
  }
 
  if (turning == false && adjustingStartingPosition == false){
    if (leftDist >= 5 && rightDist >= 5 && frontDist >= 5){
      scenario = 0;
    } else {
      if (leftDist >= 5 && rightDist < 5){ // nothing left
        
       // Serial.println("Distance left - right: " + String(leftDist - rightDist));

        if (rightDist <= 2 && frontDist <= 4){
          scenario = 8;
        } else {
          scenario = 3;

          //if (rightDist - leftDist <= -5){ // closer to left wall than right wall by 3 units
            //scenario = 6;
          //} else {
            //scenario = 3;
          //}
        }
      }
      else if (rightDist >= 5 && leftDist < 5){ // nothing right
        //Serial.println("Distance right - left: " + String(rightDist - leftDist));

        if (leftDist <= 2 && frontDist <= 4){
          scenario = 7;
        } else {
          scenario = 2;

          //if (leftDist - rightDist >= -5){ // closer to left  wall than right wall by 3 units
            //scenario = 5;
          //} else {
            //scenario = 2;
          //}
        }
        
        // right();
      } else if ((rightDist <= 5) && (leftDist <= 5)) {
        if (frontDist < 5){
          scenario = 4;
        } else { // not deadend
          scenario = 1;
        }
      }

      switch (scenario) {
        case 0:
          displayLEDs(LOW, LOW, LOW);

          delay(700);

          setSpeed(0);

          stop();

          break;

        case 1:
          displayLEDs(HIGH, LOW, LOW);

          setSpeed(50);

          forward();

          break;

        case 2:
          displayLEDs(LOW, HIGH, LOW);
          
          setSpeed(50);

          forward();

          delay(900);
          
          setSpeed(30);

          right(false);

          delay(700);

          forward();

          delay(700);

          break;

        case 3:
          displayLEDs(HIGH, HIGH, LOW);
          
          setSpeed(50);

          forward();

          delay(900);
          
          setSpeed(30);

          left(false);

          delay(700);

          forward();

          delay(700);

          break;

        case 4:
          displayLEDs(LOW, LOW, HIGH);
          
          setSpeed(30);

          turn180();

          break;

        case 5:
          displayLEDs(HIGH, LOW, HIGH);
          
          setSpeed(30);

          right(true);

          break;

        case 6:
          displayLEDs(LOW, HIGH, HIGH);
          
          setSpeed(30);

          left(true);

          break;

        case 7:
          displayLEDs(HIGH, HIGH, HIGH);
          
          setSpeed(30);

          right(true);
          break;

        case 8:
          setSpeed(30);

          left(true);

          displayLEDs(HIGH, LOW, LOW);
          delay(100);
          displayLEDs(LOW, LOW, LOW);
          delay(100);

          break;

        default:
          displayLEDs(LOW, LOW, LOW);
          break;
      }
    }
    
   
    //Serial.println("Finished");
  }
  Serial.println(scenario);
  delay(1000);
}
// IR Object Detection Function

int irDetect(int irLedPin, int irReceiverPin, long frequency)
{
  tone(irLedPin, frequency);                 // Turn on the IR LED square wave
  delay(1);                                  // Wait 1 ms
  int ir = digitalRead(irReceiverPin);       // IR receiver -> ir variable
  noTone(irLedPin);                          // Turn off the IR LED
  delay(1);                                  // Down time before recheck
  return ir;                                 // Return 0 detect, 1 no detect
}

// IR distance measurement function

int irDistance(int irLedPin, int irReceiverPin)
{
   int distance = 0;
   for(long f = 38000; f <= 42000; f += 1000)
   {
      distance += irDetect(irLedPin, irReceiverPin, f);
   }
   return distance;
}
