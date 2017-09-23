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
const float pi = 3.14 //declaring pi

static int sensorThreshhold = 75; //When to turn, in mm

static int leftSensorPin = A0, rightSensorPin = A1; //Analog pins to read sensors
static int leftWingPin = 9, rightWingPin = 10, rudderPin = 11; //PWM-capable pins for servos

SharpIR leftSensor(leftSensorPin, 20150); //Set up IR sensor objects (20150 is a magic number for this sensor model)
SharpIR rightSensor(leftSensorPin, 20150);

Servo leftWing, rightWing, rudder; //Servo objects for wings and rudder

int turnDir = 0; //0 for straight, -1 for left, 1 for right, 2 for backwards
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
  /*________sense________*/
  // If turn from last loop isn't finish, this will ensure that it finishes.
  if (turnTimer){
    turnTimer -= lastTime; 
  }
  else {
    turnDir = 0;
    //do we need to specify how long this goes for???
  }

  // Getting the distances from walls from both sensors (converts from mm to cm)
  left_dist = leftSensor.distance() * 10;
  right_dist = rightSensor.distance()* 10;


  /*________think________*/

  // This determines our boat's direction by assigning it a switch case based on sensor output
  if (left_dist < sensorThreshhold){
    turnDir = 1;  //Are we too close to a wall?
    turnTimer = 500;  //Sets us to have a 5 sec turn
  }
  if (right_dist < sensorThreshhold){
    turnDir = -1; //If both triggered, turn left
    turnTimer = 500;
  }
  if ((right_dist < sensorThreshold) && (left_dist < sensorThreshold)){
    turnDir = 2
    turnDir = 500;  // Is this needed?   
  }
  
   //IS THIS LEGIT???
  // phase = sin(timer / wingPeriod * 2*PI) / 2 * wingRange; //Current wing position, in degrees, assuming full power
   //phase = 8*(pi^2)*(sin(x)−(1/9*sin(3*x))+(1/25*sin(5*x))−(1/49*sin(7*x)));
   rudder_phase = rudderRange*sin(wingPeriod);  //Does this make sense?


  /*_________act_________*/
    switch (turnDir) {
    //In case 0 the boat will go straight because the sensor did not sense a wall within range.
    case 0:
      leftWing.write(wingCenter + triangleWaveFunction(timer));
      rightWing.write(wingCenter + triangleWaveFunction(timer));
      rudder.write(rudderCenter);
      break;
    
    //In case -1 the boat will turn to the because the left IR sensed a wall within range. 
    case -1:
      leftWing.write(wingCenter + triangleWaveFunction(timer));
      rightWing.write(wingCenter);
      rudder.write(rudderCenter - rudderRange);
      break;
      
    //In case 1 the boat will turn to the because the right IR sensed a wall within range.
    case 1:
      leftWing.write(wingCenter);
      rightWing.write(wingCenter + triangleWaveFunction(timer));
      rudder.write(rudderCenter + rudderRange);
      break;
    
    //In case 2 the boat has sensed both walls as being dangerously close and is now using the rudder to reverse.
    case 2:
      leftWing.write(wingCenter);
      rightWing.write(wingCenter);
      rudder.write(rudder_phase);
      break;
    }
      
  //
  timer += millis() - lastTime;
  timer = timer % wingPeriod; //Prevent overflows
  lastTime = millis(); //So we'll know how long the next loop takes

}

// This is a function to be used for the wing servos. It is a trunkated triangle wave function. 
// This will make the servos move at a constant rate and uses a fourier transform to imitate a trunkated triangle wave.
int triangleWaveFunction(int x){
  return 8*(pi^2)*(sin(x)−(1/9*sin(3*x))+(1/25*sin(5*x))−(1/49*sin(7*x)))
}

