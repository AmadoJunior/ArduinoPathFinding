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
int servoAngle = 0;
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
String findPath(UltraSonicDistanceSensor &sensor, Servo &servo){
  long distanceCm = 0;
  String direction = "";
  long firstHalfSum = 0;
  long secondHalfSum = 0;
  int servoAngle = 0;
  servo.write(servoAngle);
  while(servoAngle <= 90){
    distanceCm = sensor.measureDistanceCm();
    if(distanceCm <= 0 || distanceCm >= 380){
      distanceCm = 420;
    }
    firstHalfSum += distanceCm;
    servoAngle += 2;
    servo.write(servoAngle);
    delay(20);
  }
  while(servoAngle <= 180){
    distanceCm = sensor.measureDistanceCm();
    if(distanceCm <= 0 || distanceCm >= 380){
      distanceCm = 420;
    }
    secondHalfSum += distanceCm;
    servoAngle += 10;
    servo.write(servoAngle);
    delay(80);
  }
  servo.write(90);
  if(firstHalfSum > secondHalfSum){
    direction = "right";
  } else {
    direction = "left";
  }
  return direction;
}
int findBestPath(UltraSonicDistanceSensor &sensor, Servo &servo){
  long distanceCm = 0;
  int servoAngle = 0;
  servo.write(servoAngle);
  long maxSum = 0;
  long tempSum = 0;
  int count = 0;
  int arr[100];
  int subArraySize = 15;
  int pos = 0;
  while(servoAngle <= 180){
    distanceCm = sensor.measureDistanceCm();
    if(distanceCm == -1){
      distanceCm = 400;
    }
    arr[count] = distanceCm;
    count++;
    servoAngle += 2;
    servo.write(servoAngle);
    delay(5);
  }
  servo.write(90);
  for(int i = 0; i < subArraySize; i++){
    tempSum += arr[i];
  }
  for(int i = subArraySize; i < 90; i++){
    tempSum = tempSum - arr[i - subArraySize] + arr[i];
    if(tempSum > maxSum){
      maxSum = tempSum;
      pos = i;
    }
  }
  return pos*2;
}

int findBestPath2(UltraSonicDistanceSensor &sensor, Servo &servo){
  long distanceCm = 0;
  int servoAngle = 0;
  servo.write(servoAngle);
  long maxSum = 0;
  long tempSum = 0;
  int count = 0;
  int arr[200];
  int subArraySize = 15;
  int pos = 0;
  while(servoAngle <= 180){
    distanceCm = sensor.measureDistanceCm();
    if(distanceCm == -1){
      distanceCm = 400;
    }
    arr[count] = distanceCm;
    count++;
    servoAngle++;
    servo.write(servoAngle);
    delay(5);
  }
  servo.write(90);
  for(int i = 0; i < subArraySize; i++){
    tempSum += arr[i];
  }
  for(int i = subArraySize; i < 180; i++){
    tempSum = tempSum - arr[i - subArraySize] + arr[i];
    if(tempSum >= maxSum){
      maxSum = tempSum;
      pos = i - 7;
    }
  }
  return pos;
}

double turnLeft(double angle){
      double percentageL = angle/180;
      double delayNumL = 530*percentageL;
      digitalWrite(R_DIR_1, LOW);
      digitalWrite(R_DIR_2, HIGH);
      digitalWrite(L_DIR_1, HIGH);
      digitalWrite(L_DIR_2, LOW);
      analogWrite(R_SPEED, 130);
      analogWrite(L_SPEED, 130);
      //delay(30+265);
      delay(40 + (unsigned long) delayNumL);
      return delayNumL;
}
double turnRight(double angle){
      double percentageR = angle/180;
      double delayNumR = 530*percentageR;
      digitalWrite(R_DIR_1, HIGH);
      digitalWrite(R_DIR_2, LOW);
      digitalWrite(L_DIR_1, LOW);
      digitalWrite(L_DIR_2, HIGH);
      analogWrite(R_SPEED, 160);
      analogWrite(L_SPEED, 160);
      //delay(30+265);
      delay(40 + (unsigned long) delayNumR);
      return delayNumR;
}
void backUp(){
  digitalWrite(R_DIR_1, HIGH);
  digitalWrite(R_DIR_2, LOW);
  digitalWrite(L_DIR_1, HIGH);
  digitalWrite(L_DIR_2, LOW);
  analogWrite(R_SPEED, 160);
  analogWrite(L_SPEED, 160);
  delay(250);
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
  //Serial.println("Distance: " + distance);
  //Go forward
  digitalWrite(R_DIR_1, LOW);
  digitalWrite(R_DIR_2, HIGH);
  digitalWrite(L_DIR_1, LOW);
  digitalWrite(L_DIR_2, HIGH);

  if(distance > 10 || distance == -1){
    analogWrite(R_SPEED, 190);
    analogWrite(L_SPEED, 190);
  } else {
    analogWrite(R_SPEED, 0);
    analogWrite(L_SPEED, 0);
    delay(100);
    backUp();
    bestPath = findBestPath2(sensor, myServo);
    if(bestPath >= 90 && bestPath <= 200){
      double leftAngle = 180 - (double) bestPath;
      leftAngle = 90 - leftAngle;
      turnLeft(leftAngle+30);  
    } else if(bestPath >= 0 && bestPath < 90){
      double rightAngle = 90 - (double) bestPath;
      turnRight(rightAngle+30);  
    }
  }
}
