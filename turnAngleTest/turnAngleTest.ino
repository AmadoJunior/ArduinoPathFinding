//MOTOR SETUP
  //RIGHT MOTOR
  int R_SPEED = 11;
  int R_DIR_1 = 2;
  int R_DIR_2 = 4;

  //LEFT MOTOR
  int L_SPEED = 5;
  int L_DIR_1 = 8;
  int L_DIR_2 = 7;

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
      delay(30 + (unsigned long) delayNumL);
      return delayNumL;
}
double turnRight(double angle){
      double percentageR = angle/180;
      double delayNumR = 530*percentageR;
      digitalWrite(R_DIR_1, HIGH);
      digitalWrite(R_DIR_2, LOW);
      digitalWrite(L_DIR_1, LOW);
      digitalWrite(L_DIR_2, HIGH);
      analogWrite(R_SPEED, 130);
      analogWrite(L_SPEED, 130);
      //delay(30+265);
      delay(30 + (unsigned long) delayNumR);
      return delayNumR;
}


void setup() {
  //SETUP PIN_MODE
  pinMode(R_SPEED, OUTPUT);
  pinMode(L_SPEED, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println(turnLeft(180));
  analogWrite(R_SPEED, 0);
  analogWrite(L_SPEED, 0);
  delay(2000);
  Serial.println(turnRight(180));
  analogWrite(R_SPEED, 0);
  analogWrite(L_SPEED, 0);
  delay(2000);
}
