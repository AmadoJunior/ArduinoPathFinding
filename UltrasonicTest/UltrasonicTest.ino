#include <HCSR04.h>
#include <Servo.h>

#define TRIG_PIN 12
#define ECHO_PIN 6 
#define SERVO_PIN 3

Servo myServo;
long distance;
int myArr[100];
UltraSonicDistanceSensor sensor(TRIG_PIN, ECHO_PIN);

int findBestPath(UltraSonicDistanceSensor &sensor, Servo &servo, int arr[]){
  long distanceCm = 0;
  int servoAngle = 0;
  servo.write(servoAngle);
  long maxSum = 0;
  long tempSum = 0;
  int count = 0;
  int subArraySize = 5;
  int pos = 0;
  while(servoAngle <= 180){
    distanceCm = sensor.measureDistanceCm();
    if(distanceCm == -1){
      distanceCm = 420;
    }
    arr[count] = distanceCm;
    count++;
    servoAngle+=2;
    servo.write(servoAngle);
    delay(10);
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

void setup() {
  //SERVO SETUP
  myServo.attach(SERVO_PIN);
  myServo.write(90);
    Serial.begin(9600);
}

void loop() {
  //distance = sensor.measureDistanceCm();
  //Serial.println(distance);
  Serial.println("Best path is at: ");
  Serial.println(findBestPath(sensor, myServo, myArr));
  Serial.println("Array of data is: ");
  for(int i = 0; i < 85; i++){
    Serial.print(myArr[i]);
    Serial.print(" ,");
  }
  delay(100000);
}
