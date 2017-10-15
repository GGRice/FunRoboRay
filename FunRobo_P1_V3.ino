/*
 * Main code, final version
 * Reads from two IR proximity sensors, moves three servos to move the craft
 * Steers away from walls, skulls backwards if necessary
 * Team Nemo 9/2017
 */

 //Setup: Include necessary libraries, set parameters, initialize variables
#include <Servo.h>
#include <SharpIR.h>

const int wingPeriodStraight = 8000; //Wings' period during straight motion, in milliseconds
const int wingPeriodTurn = 2500; ///Wings' period during turn, in milliseconds
const int wingCenter = 90; //Center of the wing servos' motion
const int wingRange = 75; //Range of servos' motion, in degrees

const int rudderCenter = 87; //Center of the rudder servo's motion
const int rudderRange = 100; //Rudder range of motion, in degrees

const int turnTime = 3000; //How long a turn takes, in milliseconds
const int skullTime = 6000; //Same but for skulling backwards -- longer because slower

const int sensorAvgWindow = 10; //number of sensor readings to average over to determine obstacle distance
int leftReadings[sensorAvgWindow], rightReadings[sensorAvgWindow]; //The past few readings
int leftAvg = 0, rightAvg = 0; //Used to average the readings

const int turnThreshhold = 225; //Begin turn when voltage > this
const int reverseThreshhold = 300; //Skull backwards when voltage > this

const int leftSensorPin = A4, rightSensorPin = A5; //Analog pins to read sensors
const int leftWingPin = 9, rightWingPin = 10, rudderPin = 11; //PWM-capable pins for servos
const int led = A3; //Output pin to blinking LED -- analog pin because of convenient wiring

int leftVolt, rightVolt; //Analog signals coming from the IR sensors

Servo leftWing, rightWing, rudder; //Servo objects for wings and rudder

int turnDir = 0; //0 for straight, -1 for left, 1 for right, 2 for backwards
int turnTimer = 0; //Milliseconds left in the turn
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

  // Keep track of how long we've been turning, stop if necessary
  if (turnTimer>0) {
    turnTimer -= loopTime; //Decrement the turn timer, in milliseconds
    if (turnTimer < 0) turnTimer = 0; //Because it might jump 0
  }
  else turnDir = 0; //when turn is finished, direction is set to forward

  leftVolt = analogRead(leftSensorPin); //Read sensor voltages to detect proximity
  rightVolt = analogRead(rightSensorPin); //This turns out to be more reliable than a library

  Serial.print("Left reading: " + String(leftVolt) +"  Right reading: " + String(rightVolt));
  

  /*________think________*/

  for (int i = 0; i < sensorAvgWindow - 1; i++) {
    leftReadings[i] = leftReadings[i+1]; //Shift all readings back one space, drop the oldest reading
    rightReadings[i] = rightReadings[i+1]; //A circular buffer might be faster, but would be more complex
  }

  leftReadings[sensorAvgWindow-1] = leftVolt; //Add the newest readings
  rightReadings[sensorAvgWindow-1] = rightVolt;

  leftAvg = 0; //Reset the averages, so we can re-sum them
  rightAvg = 0;

  for (int i = 0; i < sensorAvgWindow; i++) { //Sum window
    leftAvg += leftReadings[i];
    rightAvg += rightReadings[i];
  }

  leftAvg = leftAvg / sensorAvgWindow; //Divide after summing to preserve precision
  rightAvg = rightAvg / sensorAvgWindow;  

  Serial.print("Left avg: " + String(leftAvg) +"  Right avg: " + String(rightAvg));

  if (leftAvg > turnThreshhold) { //Are we too close to a wall on the left side?
    turnDir = 1;  //if so, turn to the right
    turnTimer = turnTime;  //Sets us to have a turn of duration determined by turnTime
  }
  if (rightAvg > turnThreshhold) { //Checks for wall on right side
    turnDir = -1; //if wall is too close, turn left (if both sensors detect a wall, the boat will turn left)
    turnTimer = turnTime;
  }
  if ((rightAvg > reverseThreshhold) && (leftAvg > reverseThreshhold)) { //If both sensors are really close, skull backwards
    turnDir = 2; 
    turnTimer = skullTime; //Skulls for preset time
  }
 
  int rudderPhase = rudderCenter + triangleWaveFunction(timer) * rudderRange / wingRange; //This only applies while skulling


  /*_________act_________*/

  switch (turnDir) {

    case 0: //Neither sensor triggered
      Serial.println("Going straight");
      leftWing.write(wingCenter - triangleWaveFunction(timer));
      rightWing.write(wingCenter + triangleWaveFunction(timer));
      rudder.write(rudderCenter);
      Serial.println(rudder.read());
      break;
    
    case -1: //Right sensor triggered, but not left
      Serial.println("Turning left");
      leftWing.write(wingCenter - triangleWaveFunction(timer));
      rudder.write(rudderCenter + rudderRange/2);
      Serial.println(rudder.read());
      Serial.println(rudderCenter - rudderRange);
      break;
      
    case 1: //Left (and maybe right) triggered
      Serial.println("Turning right");
      rightWing.write(wingCenter + triangleWaveFunction(timer));
      rudder.write(rudderCenter - rudderRange/2);
      Serial.println(rudder.read());
      break;
    
    case 2: //Both *really* triggered
      Serial.println("Skulling");
      rudder.write(rudder_phase);
      Serial.println(rudder.read());
      break;
  }
 
  timer += (millis() - lastTime) * (turnDir ? (wingPeriodStraight / wingPeriodTurn) : 1); //Count faster if we're turning
  timer = timer % wingPeriodStraight; //Prevent overflows
  loopTime = millis() - lastTime; //For decrementing turn timer
  lastTime = millis(); //So we'll know how long the next loop takes
 
  //Blink activity LED by turning it on iff we're in the first half of the period
  digitalWrite(led, (timer < wingPeriodStraight/2) ? HIGH : LOW);
}

//This creates a rounded triangle-wave function with period `wingPeriodStraight` and range `wingRange`
//The rounding is achieved via Fourier transform (done outside this program)
int triangleWaveFunction(int phase) {
  float x = phase * 2*PI / wingPeriodStraight;
  return wingRange/2 * 8*(PI*PI)*(sin(x)-(1/9*sin(3*x))+(1/25*sin(5*x))-(1/49*sin(7*x))) / 100;
}
