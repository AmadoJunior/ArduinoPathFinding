//MOTOR SETUP
  //RIGHT MOTOR
  int R_SPEED = 11;
  int R_DIR_1 = 2;
  int R_DIR_2 = 4;

  //LEFT MOTOR
  int L_SPEED = 5;
  int L_DIR_1 = 8;
  int L_DIR_2 = 7;
  
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


void setup() {
  //SETUP PIN_MODE
  pinMode(R_SPEED, OUTPUT);
  pinMode(L_SPEED, OUTPUT);
  Serial.begin(9600);
}

void loop() {

  //AnalogRead for Voltage
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0) * 2;
  float curVoltagePercentage  = voltage / 9;

  //Forward Only Test
//  digitalWrite(R_DIR_1, LOW);
//  digitalWrite(R_DIR_2, HIGH);
//  digitalWrite(L_DIR_1, LOW);
//  digitalWrite(L_DIR_2, HIGH);
//  analogWrite(R_SPEED, 200);
//  analogWrite(L_SPEED, 200);

  
  Serial.println(turnLeft(90, curVoltagePercentage));
  analogWrite(R_SPEED, 0);
  analogWrite(L_SPEED, 0);
  delay(2000);
  Serial.println(turnRight(90, curVoltagePercentage));
  analogWrite(R_SPEED, 0);
  analogWrite(L_SPEED, 0);
  delay(2000);
}
