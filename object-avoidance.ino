#include <string.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);


int sideLeftArr[5];
int midLeftArr[5];
int sideRightArr[5];
int midRightArr[5];

int sensorIndex = 0;

int sideRightAvg = 0;
int midRightAvg = 0;
int midLeftAvg = 0;
int sideLeftAvg = 0;

const int baseSpeed = 50;

double angle[4] = {0.7854, 0.2618, -0.2618, -0.7854};

void setup() {
  Serial.begin(9600);
  AFMS.begin();

  setSpeedAllMotors(0.2*baseSpeed);
  setMotorsForward();
  activateMotors();
}

void loop() {
  int sideRightVal = exp(8.5841-log(analogRead(A0)));
  int midRightVal = exp(8.5841-log(analogRead(A3)));
  int midLeftVal = exp(8.5841-log(analogRead(A1)));
  int sideLeftVal = exp(8.5841-log(analogRead(A2)));

  sideRightArr[sensorIndex] = sideRightVal;
  midRightArr[sensorIndex] = midRightVal;
  midLeftArr[sensorIndex] = midLeftVal;
  sideLeftArr[sensorIndex] = sideLeftVal;
  sensorIndex = (sensorIndex+1)%5;
  
  //right side sensor sliding window array
  sideRightAvg = slidingWindow(sideRightArr, midRightAvg, midRightVal);

  //middle right sensor sliding window array
  midRightAvg = slidingWindow(midRightArr, midRightAvg, midRightVal);

  //middle left sensor sliding window array
  midLeftAvg = slidingWindow(midLeftArr, midLeftAvg, midLeftVal);

  //left side sensor sliding window array
  sideLeftAvg = slidingWindow(sideLeftArr, sideLeftAvg, sideLeftVal);


  float distArray[] = {sideRightAvg, midRightAvg, midLeftAvg, sideLeftAvg};


  setMotorsForward();
  

  //Serial.println(distArray[0]);
  //Serial.println(distArray[1]);
  //Serial.println(distArray[2]);
  //Serial.println(distArray[3]);

  // START AP LITE

  float v, vx, vy, r, F, kMid, kSide, Fx, Fy, delta_vx, delta_vy,
    theta, thetaNew, delta, w, FR, STEP, mass, alpha;
  int power_left, power_right, RANGE, MAX_SPEED;
  RANGE = 60;          // Reactive distance to an obstacle
  MAX_SPEED = 70;      // Maximum power supply to motors
  STEP = 1;            // Step size of the robot
  mass = 1;            // Mass of the robot
  kMid = 7;            // Hooke's law constant for inner sensors
  kSide = 5;           // Hooke's law constant for outer sensors
  FR = 0.5;            // Friction
  alpha = 1;           // Decides the amount to turn
  Fx = 100;            // Attractive goal force in x direction
  Fy = 0;              // Robot does not move in y direction
  v = 0.7 * MAX_SPEED; // Initial value is 70%
  vx = FR * v;         // Velocity drops with friction
  vy = 0;               // Robot does not move in y direction




  for (int i = 0; i < 4; i++) {
    if (distArray[i] > 5) { // Filter out lower readings
      r = RANGE - distArray[i];
      if (r < 0) {
        r = 0;
      } // end if
      theta = angle[i];

      if (i == 1 || i == 2) {
        F = kMid * r; // Compute force from the middle sensors
      } else {
        F = kSide * r; // Compute force from the outer sensor
      }
      Fx = Fx - (F * cos(theta)); // Repulsive x component
      //Serial.println(Fx);
      Fy = Fy - (F * sin(theta)); // Repulsive y component
      //Serial.println(Fy);
    } // end if
  } // end for

  delta_vx = STEP * Fx / mass; // Change in velocity in x direction
  delta_vy = STEP * Fy / mass; // Change in velocity in y direction
  vx += delta_vx; // Velocity in x direction
  vy += delta_vy; // Velocity in y direction
  v = sqrt(vx*vx + vy*vy); // Current velocity
  delta = atan2(delta_vy, delta_vx); // Direction of change in velocity
  setSpeedAllMotors(v); // Reset current velocity
  //Serial.println(v);

  w = turnFunction(delta); // Angle of the robot turn
  thetaNew = atan2(vy, vx); // Robot moves in first or second quadrant
  Serial.println(thetaNew);
  if ((-PI/2.0 <= thetaNew) && (thetaNew <= PI/2.0)) {
    power_right = (int) (v + v * alpha * w); // Power to right Motor
    power_left = (int) (v - v * alpha * w); // Power to left Motor

    // Proportionally cap the motor power
    if (power_right > MAX_SPEED || power_left > MAX_SPEED) {
      if (power_right >= power_left) {
        power_left = MAX_SPEED * (power_left / power_right);
        power_right = MAX_SPEED;
      } else {
        power_right = MAX_SPEED * (power_right / power_left);
        power_left = MAX_SPEED;
      } // end if
    } // end if
  } // end if


    if (power_right >= 0) {
      setSpeedRightMotors(power_right);
    } // end if
    if (power_left >= 0) {
      setSpeedLeftMotors(power_left);
    }  // end if
    
  // END AP LITE
}


float turnFunction(float angle) {
  float angle_radians = angle;
  angle = angle_radians;

  if ((PI/2.0 < angle) && (angle <= PI)) {
    angle_radians = PI - angle;
  } else if ((-PI < angle) && (angle < -PI/2.0)) {
    angle_radians = -PI - angle;
  } // end if

  return angle_radians;
}



void setMotorsForward() {
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  myMotor3->run(BACKWARD);
  myMotor4->run(FORWARD);
}

void setMotorsBackward() {
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  myMotor3->run(FORWARD);
  myMotor4->run(BACKWARD);
}

void activateMotors() {
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  myMotor3->run(RELEASE);
  myMotor4->run(RELEASE);
}

void setSpeedAllMotors(float speed) {
  myMotor1->setSpeed(speed);
  myMotor2->setSpeed(speed);
  myMotor3->setSpeed(speed);
  myMotor4->setSpeed(speed);
}

// Right and left might be backwards
void setSpeedRightMotors(float speed) {
  myMotor2->setSpeed(speed);
  myMotor3->setSpeed(speed);
}

// Right and left might be backwards
void setSpeedLeftMotors(float speed) {
  myMotor1->setSpeed(speed);
  myMotor4->setSpeed(speed);
}

void getCurrentSpeed() {
  return 0;
}

double slidingWindow(int arr[], int avg, int val){
  avg = 0;
  for(int i = 0; i<5; i++){
    if(arr[i]>120){
      arr[i]=120;
    }
    avg += arr[i];
    if(arr[i]==0){
      avg += val;
    }
  }
  avg/=5;
  return avg;
}