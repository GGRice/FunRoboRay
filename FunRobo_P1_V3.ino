/*
 * Main code, rough prototype
 * Not tested, do not use!
 * Reads from two IR proximity sensors, moves three servos to move the craft
 * Steers away from walls
 * Team Nemo 9/2017
 */
 
#include <Servo.h>
#include <SharpIR.h>

const int wingPeriodStraight = 8000; //Period of wing straight movement, in milliseconds
const int wingPeriodTurn = 3000; //Period of wing turning movement, in milliseconds
const int wingCenter = 90; //Center of the servos' motion
const int wingRange = 75; //Range of motion, degrees, at full power

const int rudderCenter = 80; //Center of the servos' motion
const int rudderRange = 65; //Rudder range of motion, degrees
/*const int rudderSpeed = 6; //Degrees per step, maximum rotation speed
int rudderPos = rudderCenter; //Current position
int rudderGoal = rudderCenter; //Where it wants to be*/

const int turnTime = 3000; //How long a turn takes, in milliseconds
const int skullTime = 6000; //Same but for skulling backwards

const int sensorAvgWindow = 10;
int leftReadings[sensorAvgWindow], rightReadings[sensorAvgWindow];
int leftAvg = 0, rightAvg = 0;

const int turnThreshhold = 225; //Turning begins when voltage is greater than this
const int reverseThreshhold = 300; //Skull backwards when voltage > this


const int leftSensorPin = A4, rightSensorPin = A5; //Analog pins to read sensors
const int leftWingPin = 9, rightWingPin = 10, rudderPin = 11; //PWM-capable pins for servos
const int led = A3; //Output pin to blinking LED

//SharpIR leftSensor(leftSensorPin,20150); //Set up IR sensor objects (20150 is a magic number for this sensor model)
//SharpIR rightSensor(leftSensorPin,20150);

int left_volt, right_volt; //Analog signals coming from the IR sensors

Servo leftWing, rightWing, rudder; //Servo objects for wings and rudder

int turnDir = 0; //0 for straight, -1 for left, 1 for right, 2 for backwards
int turnTimer = 0; //Milliseconds left in the turn
int phase; //Keeps track of the current position in the sine wave for the wings
long unsigned int timer = 0; //Current position in the wing-motion cycle
long unsigned int lastTime = 0; //Time the last iteration ended
long int loopTime = 0; //How long the last loop took -- signed to prevent underflow

void setup() {
  
  leftWing.attach(leftWingPin); //Attach servo objects to physical pins
  rightWing.attach(rightWingPin);
  rudder.attach(rudderPin);  
  pinMode(led, OUTPUT); //Set up LED pin
  Serial.begin(9600);

  for (int i = 0; i < sensorAvgWindow; i++) { //Initialize the averages to max so it doesn't start by turning
    leftReadings[i] = 0;
    rightReadings[i] = 0;
  }

}

void loop() {
  /*________sense________*/
  // If turn from last loop isn't finish, this will ensure that it finishes.
  if (turnTimer>0){
    turnTimer -= loopTime; //Decrement the turn timer, in milliseconds
    if (turnTimer < 0) turnTimer = 0;
  }
  else {
    turnDir = 0;
  }

  // Getting the distances from walls from both sensors (converts from mm to cm)
  //left_dist = leftSensor.getDistance()*10;
  //right_dist = rightSensor.getDistance()*10;

  left_volt = analogRead(leftSensorPin);
  right_volt = analogRead(rightSensorPin);

  Serial.print("Left reading: " + String(left_volt) +"  Right reading: " + String(right_volt));
  
  /*________think________*/

  for (int i = 0; i < sensorAvgWindow - 1; i++) {
    leftReadings[i] = leftReadings[i+1]; //Shift all readings back one space
    rightReadings[i] = rightReadings[i+1];
  }

  leftReadings[sensorAvgWindow-1] = left_volt;
  rightReadings[sensorAvgWindow-1] = right_volt;

  leftAvg = 0; //Reset the averages, so we can re-sum them
  rightAvg = 0;

  for (int i = 0; i < sensorAvgWindow; i++) { //Sum to average
    leftAvg += leftReadings[i];
    rightAvg += rightReadings[i];
  }

  leftAvg = leftAvg / sensorAvgWindow; //Divide after summing to preserve precision
  rightAvg = rightAvg / sensorAvgWindow;  

  Serial.print("Left avg: " + String(leftAvg) +"  Right avg: " + String(rightAvg));


  // This determines our boat's direction by assigning it a switch case based on sensor output
  if (leftAvg > turnThreshhold){
    turnDir = 1;  //Are we too close to a wall?
    turnTimer = turnTime;  //Sets us to have a 5 sec turn
  }
  if (rightAvg > turnThreshhold){
    turnDir = -1; //If both triggered (but not close enough to skull), turn left
    turnTimer = turnTime;
  }
  if ((rightAvg > reverseThreshhold) && (leftAvg > reverseThreshhold)){
    turnDir = 2;
    turnTimer = skullTime;
  }
 
  int rudder_phase = rudderCenter + triangleWaveFunction(timer*2, false) * rudderRange / wingRange;

  /*_________act_________*/
  switch (turnDir) {
    //In case 0 the boat will go straight because the sensor did not sense a wall within range.
    case 0:
      Serial.println("case 0");
      leftWing.write(wingCenter - triangleWaveFunction(timer, true));
      rightWing.write(wingCenter + triangleWaveFunction(timer, true));
      rudder.write(rudderCenter);
      Serial.println(rudder.read());
      break;
    
    //In case -1 the boat will turn to the because the left IR sensed a wall within range. 
    case -1:
      Serial.println("case -1");
      leftWing.write(wingCenter - triangleWaveFunction(timer, false));
      //rightWing.write(wingCenter);
      rudder.write(rudderCenter + rudderRange/2);
      Serial.println(rudder.read());
      Serial.println(rudderCenter - rudderRange);
      break;
      
    //In case 1 the boat will turn to the because the right IR sensed a wall within range.
    case 1:
      Serial.println("case 1");
      //leftWing.write(wingCenter);
      rightWing.write(wingCenter + triangleWaveFunction(timer, false));
      rudder.write(rudderCenter - rudderRange/2);
      //rudderGoal = rudderCenter - rudderRange/2;
      Serial.println(rudder.read());
      break;
    
    //In case 2 the boat has sensed both walls as being dangerously close and is now using the rudder to reverse.
    case 2:
      Serial.println("case 2");
      //leftWing.write(wingCenter);
      //rightWing.write(wingCenter);
      rudder.write(rudder_phase);
      //rudderGoal = rudder_phase;
      //rudderPos = rudderGoal; //No soft move for skulling!
      Serial.println(rudder.read());
      break;
  }

  //if (rudderPos > rudderGoal) rudderPos -= rudderSpeed; //Adjust the rudder's position
  //else if (rudderPos < rudderGoal) rudderGoal += rudderSpeed;
  //rudder.write(rudderPos); //Actually move the servo
 
  timer += millis() - lastTime;
  timer = timer % wingPeriodStraight; //Prevent overflows
  loopTime = millis() - lastTime;
  lastTime = millis(); //So we'll know how long the next loop takes
 
 //Blink LED while main loop is running
  if (timer < wingPeriodStraight/2){
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  }
  else {
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  }
}

// This is a function to be used for the wing servos. It is a trunkated triangle wave function. 
// This will make the servos move at a constant rate and uses a fourier transform to imitate a trunkated triangle wave.
int triangleWaveFunction(int phase, bool isStraight){
  float x = 0;
  if (isStraight)
    x = phase * 2*PI / wingPeriodStraight;
  else
    x = phase * 2*PI / wingPeriodTurn;
    
  return wingRange/2 * 8*(PI*PI)*(sin(x)-(1/9*sin(3*x))+(1/25*sin(5*x))-(1/49*sin(7*x))) / 100;
}
