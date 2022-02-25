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
volatile bool select[5] = {false, false, false, false, false};
volatile bool flash[5] = {false, false, false, false, false};
volatile bool blinkL[5] = {false, false, false, false, false};
volatile bool err[5] = {false, false, false, false, false};
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
  for(int i=0;i<5;i++)
  {
    if(count[i]==countM[i])
    {
      if(start[i])
      {
        digitalWrite(stepMotorAP[i], movem[i]);
        movem[i]=~movem[i];
        count[i]=0;
        if(dir[i])
        {
          stepM[i]++;
          if(stepM[i]==MAX[i])
          {
            change(i);
          }
        }
        else
        {
          stepM[i]--;
          if(stepM[i]==MIN[i])
          {
            change(i);
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
        else if(stepM[i] == 0&&err[i])
        {
          start[i] =false;
          flash[i] =true;
        }
      }
      else if(flash[i])
      {
        blinkL[i] = ~blinkL[i];
        digitalWrite(led[i],blinkL[i]);
      }
  }
   else
    {
      count[i]++;
    }
}

}
void change(int m){
  dir[m] = ~dir[m];
  digitalWrite(stepMotorAD[m],dir[m]);
}
void resetM(int m){
  count[m] = 0;
  MAX[m]=50;
  MIN[m]=-50;
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
