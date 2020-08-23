//Servo & UltrasonicS Libraries
#include <HCSR04.h>
#include <Servo.h>

//Init Servo & UltrasonicS
#define TRIG_PIN 12
#define ECHO_PIN 6 
#define SERVO_PIN 3
UltraSonicDistanceSensor sensor(TRIG_PIN, ECHO_PIN);
Servo myServo;

//Servo & UltrasonicS Vars
long distance;
int bestPath;

//MOTOR SETUP
  //RIGHT MOTOR
  int R_SPEED = 11;
  int R_DIR_1 = 2;
  int R_DIR_2 = 4;

  //LEFT MOTOR
  int L_SPEED = 5;
  int L_DIR_1 = 8;
  int L_DIR_2 = 7;
  
//Functions

int findBestPath(UltraSonicDistanceSensor &sensor, Servo &servo){
  //Initializing
  long distanceCm = 0;
  int servoAngle = 0;
  long maxSum = 0;
  long tempSum = 0;
  int count = 0;
  int arr[200];
  int subArraySize = 20;
  int pos = 0;

  //Scanning and Setting TempSum
  servo.write(servoAngle);
  while(servoAngle <= 180){
    
    //Docs Recommend 60ms Between Measurements
    distanceCm = sensor.measureDistanceCm();
    delay(65);
    
    //Edge Case: Object Out Of Range
    if(distanceCm == -1){
      distanceCm = 420;
    }
    
    //Filling Temp Array
    if(servoAngle >= 0 && servoAngle <= subArraySize){
      tempSum += distanceCm;
    }

    //Storing and Advancing
    arr[count] = distanceCm;
    count++;
    servoAngle++;
    servo.write(servoAngle);
    
  }
  
  //Resetting Head Pos
  servo.write(90);

  //Finding Max SubArray
  for(int i = subArraySize; i < 180; i++){
    
    tempSum = tempSum - arr[i - subArraySize] + arr[i];
    
    if(tempSum >= maxSum){
      maxSum = tempSum;
      pos = i - 10;
    }
    
  }
  
  return pos;
  
}

double turnLeft(double angle, float curVoltagePercentage){
  
  //Calc Delay Num
    double percentageL = angle/180;
    double delayNumL = 500*percentageL;
      
  //Set Directions
    digitalWrite(R_DIR_1, LOW);
    digitalWrite(R_DIR_2, HIGH);
    digitalWrite(L_DIR_1, HIGH);
    digitalWrite(L_DIR_2, LOW);

  //Calc/Set Speed
    float speed = 170 / curVoltagePercentage;
    analogWrite(R_SPEED, speed);
    analogWrite(L_SPEED, speed);

  //Delay 
    //delay(30+265);
     unsigned long startUpCompensation = 30;
     delay(startUpCompensation + (unsigned long) delayNumL);
     return delayNumL;
 
}

double turnRight(double angle, float curVoltagePercentage){
  
  //Calc Delay Num
    double percentageR = angle/180;
    double delayNumR = 500*percentageR;

  //Set Directions
    digitalWrite(R_DIR_1, HIGH);
    digitalWrite(R_DIR_2, LOW);
    digitalWrite(L_DIR_1, LOW);
    digitalWrite(L_DIR_2, HIGH);
      
  //Calc/Set Speed
    float speed = 170 / curVoltagePercentage;
    analogWrite(R_SPEED, speed);
    analogWrite(L_SPEED, speed);

  //Delay
    //delay(30+265);
    unsigned long startUpCompensation = 30;
    delay(startUpCompensation + (unsigned long) delayNumR);
    return delayNumR;
     
}

void backUp(float curVoltagePercentage){
  
  //Setting Direction
  digitalWrite(R_DIR_1, HIGH);
  digitalWrite(R_DIR_2, LOW);
  digitalWrite(L_DIR_1, HIGH);
  digitalWrite(L_DIR_2, LOW);
  
  //Writing Speed
  float speed = 170 / curVoltagePercentage
  analogWrite(R_SPEED, (int) speed);
  analogWrite(L_SPEED, (int) speed);
  delay(200);
  
  //Reseting Speed and Direction
  analogWrite(R_SPEED, 0);
  analogWrite(L_SPEED, 0);
  digitalWrite(R_DIR_1, LOW);
  digitalWrite(R_DIR_2, HIGH);
  digitalWrite(L_DIR_1, LOW);
  digitalWrite(L_DIR_2, HIGH);
  
}

void setup() {
  //SETUP PIN_MODE
  pinMode(R_SPEED, OUTPUT);
  pinMode(L_SPEED, OUTPUT);

  //Motor Dir
  pinMode(R_DIR_1, OUTPUT);
  pinMode(L_DIR_1, OUTPUT);
  pinMode(R_DIR_2, OUTPUT);
  pinMode(L_DIR_2, OUTPUT);
  //Init Speed
  analogWrite(R_SPEED, 200);
  analogWrite(L_SPEED, 200);

  //SERVO SETUP
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  Serial.begin(9600);
}

void loop() {
  //UltrasonicS distance
  distance = sensor.measureDistanceCm();

  //AnalogRead for Voltage
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float curVoltagePercentage  = voltage / 5;
  
  //Set Direction Forward
  digitalWrite(R_DIR_1, LOW);
  digitalWrite(R_DIR_2, HIGH);
  digitalWrite(L_DIR_1, LOW);
  digitalWrite(L_DIR_2, HIGH);

  //Main Logic
  if(distance > 20 || distance == -1){
    
    int forwardSpeed = 200 / curVoltagePercentage;
    analogWrite(R_SPEED, 200);
    analogWrite(L_SPEED, 200);
    
  } else {
    
    analogWrite(R_SPEED, 0);
    analogWrite(L_SPEED, 0);
    delay(100);
    backUp(curVoltagePercentage);
    bestPath = findBestPath(sensor, myServo);
    
    if(bestPath >= 90 && bestPath <= 200){
      
      double leftAngle = 180 - (double) bestPath;
      leftAngle = 90 - leftAngle;
      turnLeft(leftAngle, curVoltagePercentage);  
      
    } else if(bestPath >= 0 && bestPath < 90){
      
      double rightAngle = 90 - (double) bestPath;
      turnRight(rightAngle, curVoltagePercentage);  
      
    }
  }
}
