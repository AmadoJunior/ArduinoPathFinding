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

//MOTOR SETUP
  //RIGHT MOTOR
  int R_SPEED = 11;
  int R_DIR_1 = 2;
  int R_DIR_2 = 4;

  //LEFT MOTOR
  int L_SPEED = 10;
  int L_DIR_1 = 8;
  int L_DIR_2 = 7;
  

void setup() {
  //SETUP PIN_MODE
  pinMode(R_SPEED, OUTPUT);
  pinMode(L_SPEED, OUTPUT);

  pinMode(R_DIR_1, OUTPUT);
  pinMode(L_DIR_1, OUTPUT);
  pinMode(R_DIR_2, OUTPUT);
  pinMode(L_DIR_2, OUTPUT);

  //SERVO SETUP
  myServo.attach(SERVO_PIN);
  myServo.write(90);
  Serial.begin(9600);
}

void loop() {
  //UltrasonicS distance
  distance = sensor.measureDistanceCm();
  Serial.println(distance);

  //Servo test
  myServo.write(0);
  distance = sensor.measureDistanceCm();
  Serial.println(distance);
  delay(1000);
  myServo.write(180);
  distance = sensor.measureDistanceCm();
  Serial.println(distance);
  delay(1000);
  myServo.write(90);
  distance = sensor.measureDistanceCm();
  Serial.println(distance);
  delay(1000);
  
  //RIGHT MOTOR
  digitalWrite(R_DIR_1, LOW);
  digitalWrite(R_DIR_2, HIGH);
  //LEFT MOTOR
  digitalWrite(L_DIR_1, LOW);
  digitalWrite(L_DIR_2, HIGH);

  //SET SPEED
  analogWrite(R_SPEED, 255);
  analogWrite(L_SPEED, 255);
}
