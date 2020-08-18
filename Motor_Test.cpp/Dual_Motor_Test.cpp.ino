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

  Serial.begin(96000);
}

void loop() {
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
