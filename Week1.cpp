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

bool turning = false;

int leftOrRight = 1;

void setSpeed(int speed){
  cw = 1500 - speed;
  ccw = 1500 + speed;
}

void reverse(){
  wheelLeft.write(cw + offset);
  wheelRight.write(ccw);
}

void left(){
  Serial.println("Moving left");
  wheelLeft.write(ccw);
  wheelRight.write(ccw);
 
  turning = true;
 
  delay(1000);
 
  turning = false;
}

void right(){
  Serial.println("Moving right");
  wheelLeft.write(cw - 40);
  wheelRight.write(cw);
 
  turning = true;
 
  delay(1000);
 
  turning = false;
}

void forward(){
  Serial.println("Moving forward");
 
  wheelLeft.write(ccw - offset);
  wheelRight.write(cw);
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
 
  setSpeed(50);
  forward();
  
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
  Serial.println("Right dist:  " + String(rightDist));
  Serial.println("Front dist:  " + String(frontDist));
 
  if (turning == false){
    if (frontDist < 5){
      reverse();
    } else {
      if (leftDist < 5){ // nothing left
        setSpeed(35);
      
        left();
      }
      else if (rightDist < 5){ // nothing right
        setSpeed(10);
      
        right();
      } else if (!(rightDist < 5) and !(leftDist < 5)) {
        forward();
      }
    }
   
    Serial.println("Finished");
    setSpeed(50);
  }
  delay(100);
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
