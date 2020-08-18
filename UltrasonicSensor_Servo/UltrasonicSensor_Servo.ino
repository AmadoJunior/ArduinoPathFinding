#include <Servo.h>
#include <HCSR04.h>
#define TRIG_PIN 12
#define ECHO_PIN 11 

UltraSonicDistanceSensor sensor(TRIG_PIN, ECHO_PIN);
Servo myservo;

long a;
int angle = 0;
boolean sentinel = true;

void setup() {
  
  myservo.attach(9);//connect pin 9 with the control line(the middle line of Servo) 
  myservo.write(0);// move servos to center position -> 90°
  Serial.begin(9600);//Initialization of Serial Port
  delay(1000);
}

void loop() {
  a = sensor.measureDistanceCm();
  Serial.println(a);
  if(a <= 20 && sentinel){
    angle++;
    if(angle >= 170){
      sentinel = false;
    }
  } else if(a <= 15){
    angle--;
    if(angle <= 10){
      sentinel = true;
    }
  }
  myservo.write(angle);
}
