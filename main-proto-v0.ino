/*
 * Main code, rough prototype
 * Not tested, do not use!
 * Reads from two IR proximity sensors, moves three servos to move the craft
 * Steers away from walls
 * Team Nemo 9/2017
 */


#include <Servo.h>
#include <SharpIR.h>

static int wingPeriod = 2000; //Period of wing movement, in milliseconds
static int wingCenter = 90; //Center of the servos' motion
static int wingRange = 90; //Range of motion, degrees, at full power
static int rudderCenter = 90; //Center of the servos' motion
static int rudderRange = 90; //Rudder range of motion, degrees
static int turnTime = 3000; //How long a turn takes, in milliseconds

static int sensorThreshhold = 750; //When to turn, in mm

static int leftSensorPin = A0, rightSensorPin = A1; //Analog pins to read sensors
static int leftWingPin = 9, rightWingPin = 10, rudderPin = 11; //PWM-capable pins for servos

SharpIR leftSensor(leftSensorPin, 20150); //Set up IR sensor objects (20150 is a magic number for this sensor model)
SharpIR rightSensor(leftSensorPin, 20150);

Servo leftWing, rightWing, rudder; //Servo objects for wings and rudder

int turnDir = 0; //0 for straight, -1 for left, 1 for right
int turnTimer = 0; //Milliseconds left in the turn
int phase; //Keeps track of the current position in the sine wave for the wings
long unsigned int timer = 0; //Current position in the wing-motion cycle
long unsigned int lastTime = 0; //Time the last iteration ended

void setup() {

  leftWing.attach(leftWingPin); //Attach servo objects to physical pins
  rightWing.attach(rightWingPin);
  rudder.attach(rudderPin);

}


void loop() {
  
  if (turnTimer) turnTimer--; //Eventually, finish the turn
  else turnDir = 0;

  if (leftSensor.distance() * 10 /*cm to mm*/ < sensorThreshhold) turnDir = 1; //Are we too close to a wall?
  if (rightSensor.distance() * 10 /*cm to mm*/ < sensorThreshhold) turnDir = -1; //If both triggered, turn left

  phase = sin(timer / wingPeriod * 2*PI) / 2 * wingRange; //Current wing position, in degrees, assuming full power

  switch (turnDir) {
    case 0:
      leftWing.write(wingCenter + phase);
      rightWing.write(wingCenter + phase);
      rudder.write(rudderCenter);
      break;
    case -1:
      leftWing.write(wingCenter + phase);
      rightWing.write(wingCenter);
      rudder.write(rudderCenter - rudderRange);
      break;
    case 1:
      leftWing.write(wingCenter);
      rightWing.write(wingCenter + phase);
      rudder.write(rudderCenter + rudderRange);
      break;


  timer += millis() - lastTime;
  timer = timer % wingPeriod; //Prevent overflows
  lastTime = millis(); //So we'll know how long the next loop takes

}
