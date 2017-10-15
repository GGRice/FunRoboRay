/*
 * Main code, rough prototype
 * Not tested, do not use!
 * Reads from two IR proximity sensors, moves three servos to move the craft
 * Steers away from walls
 * Team Nemo 9/2017
 */

 //Setup: Include necessary libraries, set parameters, initialize variables
#include <Servo.h>
#include <SharpIR.h>

const int wingPeriodStraight = 8000; //Period of wing straight movement, in milliseconds
const int wingPeriodTurn = 2500; //Period of wing turning movement, in milliseconds
const int wingCenter = 90; //Center of the servos' motion
const int wingRange = 75; //Range of motion, degrees, at full power

const int rudderCenter = 87; //Center of the servos' motion
const int rudderRange = 100; //Rudder range of motion, degrees

const int turnTime = 3000; //How long a turn takes, in milliseconds
const int skullTime = 6000; //Same but for skulling backwards

const int sensorAvgWindow = 10; //number of sensor readings to average over to determine obstacle distance
int leftReadings[sensorAvgWindow], rightReadings[sensorAvgWindow]; //prep to take number of readings for each sensor determined by sensorAvgWindow
int leftAvg = 0, rightAvg = 0; //set averaged sensor readings for each sensor

const int turnThreshhold = 225; //Turning begins when voltage is greater than this
const int reverseThreshhold = 300; //Skull backwards when voltage > this


const int leftSensorPin = A4, rightSensorPin = A5; //Analog pins to read sensors
const int leftWingPin = 9, rightWingPin = 10, rudderPin = 11; //PWM-capable pins for servos
const int led = A3; //Output pin to blinking LED

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
    turnDir = 0; //when turn is finished, direction is set to forward
  }

//directly read sensor voltages
  left_volt = analogRead(leftSensorPin);
  right_volt = analogRead(rightSensorPin);

  Serial.print("Left reading: " + String(left_volt) +"  Right reading: " + String(right_volt));
  
  /*________think________*/

  for (int i = 0; i < sensorAvgWindow - 1; i++) {
    leftReadings[i] = leftReadings[i+1]; //Shift all readings back one space
    rightReadings[i] = rightReadings[i+1];
  }

  leftReadings[sensorAvgWindow-1] = left_volt; //store sensor readings
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
  if (leftAvg > turnThreshhold){ //Are we too close to a wall on the left side?
    turnDir = 1;  //if so, turn to the right
    turnTimer = turnTime;  //Sets us to have a turn of duration determined by turnTime
  }
  if (rightAvg > turnThreshhold){ //Checks for wall on right side
    turnDir = -1; //if wall is too close, turn left (if both sensors detect a wall, the boat will turn left)
    turnTimer = turnTime;
  }
  if ((rightAvg > reverseThreshhold) && (leftAvg > reverseThreshhold)){ //If both sensors are too close to a wall, skull backwards
    turnDir = 2; 
    turnTimer = skullTime; //skulls for preset time
  }
 
  int rudder_phase = rudderCenter + triangleWaveFunction(timer) * rudderRange / wingRange;

  /*_________act_________*/
  switch (turnDir) {
    //In case 0 the boat will go straight because the sensor did not sense a wall within range.
    case 0:
      Serial.println("case 0");
      leftWing.write(wingCenter - triangleWaveFunction(timer));
      rightWing.write(wingCenter + triangleWaveFunction(timer));
      rudder.write(rudderCenter);
      Serial.println(rudder.read());
      break;
    
    //In case -1 the boat will turn to the because the left IR sensed a wall within range. 
    case -1:
      Serial.println("case -1");
      leftWing.write(wingCenter - triangleWaveFunction(timer));
      rudder.write(rudderCenter + rudderRange/2);
      Serial.println(rudder.read());
      Serial.println(rudderCenter - rudderRange);
      break;
      
    //In case 1 the boat will turn to the because the right IR sensed a wall within range.
    case 1:
      Serial.println("case 1");
      rightWing.write(wingCenter + triangleWaveFunction(timer));
      rudder.write(rudderCenter - rudderRange/2);
      Serial.println(rudder.read());
      break;
    
    //In case 2 the boat has sensed both walls as being dangerously close and is now using the rudder to reverse.
    case 2:
      Serial.println("case 2");
      rudder.write(rudder_phase);
      Serial.println(rudder.read());
      break;
  }
 
  timer += (millis() - lastTime) * (turnDir ? (wingPeriodStraight / wingPeriodTurn) : 1); //Count faster if we're turning
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
int triangleWaveFunction(int phase) {
  float x = phase * 2*PI / wingPeriodStraight;
  return wingRange/2 * 8*(PI*PI)*(sin(x)-(1/9*sin(3*x))+(1/25*sin(5*x))-(1/49*sin(7*x))) / 100;
}
