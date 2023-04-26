#include <Braccio.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

int frequency = 0;

 
void setup() {
    Serial.begin(9600);
    Braccio.begin();
   
    //Braccio.ServoMovement(20, 90, 90, 90, 90, 90, 73); //sets the starting position of arm

    AFMS.begin();   // create with the default frequency 1.6KHz

    myMotor->setSpeed(0);
    myMotor2->setSpeed(0);
    myMotor3->setSpeed(0);
    myMotor4->setSpeed(0);
    myMotor->run(RELEASE);
    myMotor2->run(RELEASE);
    myMotor3->run(RELEASE);
    myMotor4->run(RELEASE);

    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(sensorOut, INPUT);
 
    // Setting frequency-scaling to 20%
    digitalWrite(S0,HIGH);
    digitalWrite(S1,LOW);

}
 
void loop() {
    Serial.print(isRed());
    if(isRed()){
      armPosition1();
      delay(1000);
      moveForward();
      delay(6000);
      armPosition2Release();
      delay(2000);
      moveBackward();
      delay(1000);
    }
    else{
      armPosition1();
      delay(1000);
      moveForward();
      delay(6000);
      armPosition1Release();
      delay(2000);
      moveBackward();
      delay(1000);
    }
    delay(10000);
}

void armPosition1() {
    //The braccio moves to the sponge. Only the M2 servo will moves
  Braccio.ServoMovement(20, 100,  133, 180, 150,  90,   10);

  delay(1000);

  Braccio.ServoMovement(20, 100,  133, 180, 155,  90,   73);

  delay(1000);

  Braccio.ServoMovement(20, 100,  133, 180, 150,  90,   73);

  delay(1000);

  Braccio.ServoMovement(20, 90,  45, 180, 180,  90,  73);
 
}
void armPosition1Release() {
    //The braccio moves to the sponge. Only the M2 servo will moves
  Braccio.ServoMovement(20,           100,  133, 180, 150,  90,   73);

  delay(5000);

  Braccio.ServoMovement(20,           100,  133, 180, 150,  90,   10);

  delay(1000);

  Braccio.ServoMovement(20,           90,  45, 180, 180,  90,  10);
 
}

void armPosition2() {
 
  Braccio.ServoMovement(20,           60,  133, 180, 150,  90,   10);

  delay(1000);

  Braccio.ServoMovement(20,           60,  133, 180, 150,  90,   73);

  delay(1000);

  Braccio.ServoMovement(20,           90,  45, 180, 180,  90,  73);

}

void armPosition2Release() {
 
  Braccio.ServoMovement(20,           60,  133, 180, 150,  90,   73);

  delay(5000);

  Braccio.ServoMovement(20,           60,  133, 180, 150,  90,   10);

  delay(1000);

  Braccio.ServoMovement(20,           90,  45, 180, 180,  90,  10);

}

void armPosition3() {
 
  //move the arm so that the block gets placed back onto the ground  
}

void moveForward(){

  myMotor->setSpeed(50);
   myMotor2->setSpeed(50);
   myMotor3->setSpeed(50);
   myMotor4->setSpeed(50);


for(int i = 0; i<6; i++)  {
  myMotor->run(FORWARD);
   myMotor2->run(BACKWARD);
   myMotor3->run(BACKWARD);
   myMotor4->run(FORWARD);
   delay (1000);
}
   
myMotor->setSpeed(0);
   myMotor2->setSpeed(0);
   myMotor3->setSpeed(0);
   myMotor4->setSpeed(0);

}

void moveBackward(){

myMotor->setSpeed(50);
   myMotor2->setSpeed(50);
   myMotor3->setSpeed(50);
   myMotor4->setSpeed(50);

for(int i = 0; i<6; i++)  {
  myMotor->run(BACKWARD);
   myMotor2->run(FORWARD);
   myMotor3->run(FORWARD);
   myMotor4->run(BACKWARD);
   delay (1000);
}

myMotor->setSpeed(0);
   myMotor2->setSpeed(0);
   myMotor3->setSpeed(0);
   myMotor4->setSpeed(0);

}
bool isRed(){
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  // Reading the output frequency
  frequency = pulseIn(sensorOut, LOW);
  frequency = map(frequency, 25,72,255,0);
  //Remaping the value of the frequency to the RGB Model of 0 to 255
  if(frequency>200){
    return true;
  }
  else{
    return false;
  }
}