const int stepMotorAD = 4;
const int stepMotorAP = 5;
const int stepMotorAInt = 2;
const int pd = 500;
volatile byte AH = LOW;
volatile byte AL = LOW; 
volatile bool dir = LOW;
void setup() {
  // put your setup code here, to run once:
  pinMode(stepMotorAD, OUTPUT);
  pinMode(stepMotorAP, OUTPUT);
  pinMode(stepMotorAInt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stepMotorAInt),up,RISING);
  attachInterrupt(digitalPinToInterrupt(stepMotorAInt),down,FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  digitalWrite(stepMotorAD, dir);
  digitalWrite(stepMotorAP, HIGH);
  delayMicroseconds(pd);
  digitalWrite(stepMotorAP,LOW);
  delayMicroseconds(pd);
}
void up(){
  dir = HIGH;
}
void down(){
  dir = LOW;
}
