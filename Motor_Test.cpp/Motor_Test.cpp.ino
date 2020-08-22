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
String bestPath;

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
    servoAngle += 5;
    servo.write(servoAngle);
    delay(80);
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
  Serial.println("Distance: " + distance);
  //Go forward
  digitalWrite(R_DIR_1, LOW);
  digitalWrite(R_DIR_2, HIGH);
  digitalWrite(L_DIR_1, LOW);
  digitalWrite(L_DIR_2, HIGH);

  if(distance > 20){
    analogWrite(R_SPEED, 140);
    analogWrite(L_SPEED, 140);
  } else {
    analogWrite(R_SPEED, 0);
    analogWrite(L_SPEED, 0);
    delay(1000);
    bestPath = findPath(sensor, myServo);
    if(bestPath == "left"){
      digitalWrite(R_DIR_1, LOW);
      digitalWrite(R_DIR_2, HIGH);
      digitalWrite(L_DIR_1, HIGH);
      digitalWrite(L_DIR_2, LOW);
      analogWrite(R_SPEED, 120);
      analogWrite(L_SPEED, 120);
      delay(300);
    } else {
      digitalWrite(R_DIR_1, HIGH);
      digitalWrite(R_DIR_2, LOW);
      digitalWrite(L_DIR_1, LOW);
      digitalWrite(L_DIR_2, HIGH);
      analogWrite(R_SPEED, 120);
      analogWrite(L_SPEED, 120);
      delay(300);
    }
  }
}
