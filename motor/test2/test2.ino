const int stepMotorAD[5] = {3,5,7,9,11};//direction input
const int stepMotorAP[5] = {4,6,8,10,12};//movement input(toggle to cause movement)
volatile int startingStep[5] = {0,0,0,0,0};//starting angle of the system
int stepM[5] = {0,0,0,0,0};//recorded step for the stepper motor to keep track of where the stepper motor is at
int count[5] = {0,0,0,0,0};
int countM[5] = {200,200,200,200,200};//how long it takes for a step to happen, higher it is the longer the delay, if this value decreasses it lowers the delay
volatile bool dir[5] = {LOW,LOW,LOW,LOW,LOW};//when this is low it goes clockwise? when this is high it should go counterclockwise? needs testing to make sure
volatile bool movem[5] = {LOW,LOW,LOW,LOW,LOW};
volatile bool start[5] ={true,false,false,false,false};//When a motor is supposed to go this should be true, starts off as false
volatile int MAX[5] = {50,50,50,50,50};//max steps in the positive direction, maximum possible should be 50?
volatile int MIN[5] = {-50,-50,-50,-50,-50};//min steps in the negative direction, minimum possible should be -50?
//variables to keep track of cycles and when to stop
volatile int cycleCount[5] = {0,0,0,0,0};//keeps track of cycle, cycles are checked at starting angle
volatile int cycleMax[5] = {10,10,1000,1000,1000};//keeps track of max cycles

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
  digitalWrite(stepMotorAD[0],LOW);
  digitalWrite(stepMotorAD[1],LOW);
  digitalWrite(stepMotorAD[2],LOW);
  digitalWrite(stepMotorAD[3],LOW);
  digitalWrite(stepMotorAD[4],LOW);
}

void loop() {
 for(int i=0;i<5;i++)
  {
    
    if(count[i]==countM[i])
    {
      count[i]=0;
      if(start[i])
      {
        movem[i] = ~movem[i];
        if(movem[i])
        {
          digitalWrite(stepMotorAP[i], HIGH);
        }
        else
        {
          digitalWrite(stepMotorAP[i], LOW);
        }
        
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

      }
  }
   else
    {
      count[i] = count[i]+1;
    }
    delay(1);
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
}
