//inputs for the motors, led, touchscreens etc
const int stepMotorAD[5] = {3,5,7,9,11};//direction input
const int stepMotorAP[5] = {4,6,8,10,12};//movement input(toggle to cause movement)
const int led[5] = {13,14,15,16,17};//led values
const int stepMotorStop = 2;//for emergency stop
//variables to keep track of steps and angles
volatile int startingStep[5] = {0,0,0,0,0};//starting angle of the system
volatile int stepM[5] = {0,0,0,0,0};//recorded step for the stepper motor to keep track of where the stepper motor is at
volatile double angle[5] = {0,0,0,0,0};//recorded angle
//a way to set up a delay between the stepper motors
volatile int count[5] = {0,0,0,0,0};
volatile int countM[5] = {3500,3500,3500,3500,3500};//how long it takes for a step to happen, higher it is the longer the delay, if this value decreasses it lowers the delay
//boolean values for starting, general movement and direction of the steppper motor
volatile bool dir[5] = {LOW,LOW,LOW,LOW,LOW};//when this is low it goes clockwise? when this is high it should go counterclockwise? needs testing to make sure
volatile bool movem[5] = {LOW,LOW,LOW,LOW,LOW};
volatile bool start[5] ={false,false,false,false,false};//When a motor is supposed to go this should be true, starts off as false
//min max step motor values
volatile int MAX[5] = {50,50,50,50,50};//max steps in the positive direction, maximum possible should be 50?
volatile int MIN[5] = {-50,-50,-50,-50,-50};//min steps in the negative direction, minimum possible should be -50?
//variables to keep track of cycles and when to stop
volatile int cycleCount[5] = {0,0,0,0,0};//keeps track of cycle, cycles are checked at starting angle
volatile int cycleMax[5] = {1000,1000,1000,1000,1000};//keeps track of max angle

volatile int motor = 0;//determines what motor is changing mostly used to interact with the screen

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
  // all that is missing is led implementation I will work on that shortly, error implementation, adding touchscreen + controls and figuring out how to implement resistors 
  for(i=0;i<5;i++)
  {
    if(start[i])
    {
      if(count[i]==countM[i])
      {
        digitalWrite(stepMotorAP[i], movem[i]);
        movem[i]=~movem[i];
        count[i]=0;
        if(dir[i])
        {
          stepM[i]++;
          if(stepM[i]==MAX[i])
          {
            change(m);
          }
        }
        else
        {
          stepM[i]--;
          if(stepM[i]==MIN[i])
          {
            change(m);
          }
        }
        if(stepM[i]==startingStep[i])
        {
          cycleCount[i]++;
          if(cycleCount[i]==cycleMax[i])
          {
            resetM(i);
          }
        }
      }
    else
    {
      count[i]++;
    }
  }
   
}

}
void change(int m){
  dir[m] = ~dir[m];
  digitalWrite(stepMotorAD[i],dir[i]);
}
void reset(int m){
  count[m] = 0;
  MAX[m]=50;
  MIN[m]=-50;
  cycleCount[m]=0;
  start[m] =false;
}
void stopping()
{
  start[0] = false;
  start[1] = false;
  start[2] = false;
  start[3] = false;
  start[4] = false;
  //set all the led's to warning flashing????
}
void error(int m)
{
  //this function should be when a cable breaks in one of the stations
}

