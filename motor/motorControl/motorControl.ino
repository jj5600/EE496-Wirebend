//inputs for the motors, led, touchscreens etc
const int stepMotorAD[5] = {3,5,7,9,11};//direction input
const int stepMotorAP[5] = {4,6,8,10,12};//movement input(toggle to cause movement)
const int led[5] = {13,14,15,16,17};//led values
const int stepMotorStop = 2;//for emergency stop
//variables to keep track of steps and angle
int startingStep= 0;//starting angle of the system
int stepM[5] = {0,0,0,0,0};//recorded step for the stepper motor to keep track of where the stepper motor is at
double angle[5] = {0,0,0,0,0};//recorded angle
//a way to set up a delay between the stepper motors
//boolean values for starting, general movement and direction of the steppper motor
bool dir = LOW;//when this is low it goes clockwise? when this is high it should go counterclockwise? needs testing to make sure
bool start[5] ={true,true,false,false,false};//When a motor is supposed to go this should be true, starts off as false
bool select[5] = {false, false, false, false, false};
bool flash[5] = {false, false, false, false, false};
bool blinkL[5] = {false, false, false, false, false};
bool err[5] = {false, false, false, false, false};
//min max step motor values
int MAX = 50;//max steps in the positive direction, maximum possible should be 50?
int MIN = -50;//min steps in the negative direction, minimum possible should be -50?
//variables to keep track of cycles and when to stop
int cycleCount[5] = {0,0,0,0,0};//keeps track of cycle, cycles are checked at starting angle
int cycleMax = 10;//keeps track of max angle
int pd = 3500;


volatile int motor = 0;//determines what motor is changing mostly used to interact with the screen

void cycle(bool [], int, int, int);
void resetM(int);
void stopping();
void set(const int [], bool [], bool);
void cycleDir(bool [],int , bool );

void setup() {
  // put your setup code here, to run once:
  pinMode(stepMotorAD[0], OUTPUT);
  pinMode(stepMotorAP[0], OUTPUT);
  pinMode(stepMotorAD[1], OUTPUT);
  pinMode(stepMotorAP[1], OUTPUT);
  pinMode(stepMotorAD[2], OUTPUT);
  pinMode(stepMotorAP[2], OUTPUT);
  pinMode(stepMotorAD[3], OUTPUT);
  pinMode(stepMotorAP[3], OUTPUT);
  pinMode(stepMotorAD[4], OUTPUT);
  pinMode(stepMotorAP[4], OUTPUT);
  pinMode(stepMotorStop, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(stepMotorStop),stopping,RISING);//emergency stop interrupt
}

void loop() {
  for(int i = 0; i<cycleMax;i++)
  {
    for(int n =0; n<5; n++)
    {
      if(start[n])
      {
        cycleCount[n]++;
      }
    }
    cycle(start,MIN, MAX,startingStep);
  }
 
}
void set(const int val[],bool check[], bool valSet)
{
  for(int i=0; i<5;i++)
  {
    if(check[i]){
      digitalWrite(val[i],valSet);
    }
  }
}
void cycleDir(bool val[],int amount, bool dirB)
{
  set(stepMotorAD,val,dirB);
  for(int i=0;i<amount;i++)
  {
    set(stepMotorAP,val,HIGH);
    delayMicroseconds(pd);
    set(stepMotorAP,val,LOW);
    delayMicroseconds(pd);
  }
}
void cycle(bool val[], int minVal, int maxVal, int start){
  cycleDir(val, (maxVal-start), HIGH);
  cycleDir(val, (maxVal-minVal), LOW);
  cycleDir(val, (start-minVal), HIGH);
}
void resetM(int m){
  MAX=50;
  MIN=-50;
  cycleCount[m]=0;
  start[m] = false;
  flash[m] = false;
  blinkL[m] = false;
  digitalWrite(led[m], false);
}
void stopping()
{
  for(int i = 0; i < 5;i++)
  {
    if(start[i]==true)
    {
      start[i]==false;
      flash[i]==true;
    }
  }
}
void error(int m)
{
  
  //this function should be when a cable breaks in one of the stations
}
