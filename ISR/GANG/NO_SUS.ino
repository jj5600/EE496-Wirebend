// For RAMPS 1.4
#define X_DIR_PIN          2
#define X_STEP_PIN         3
#define X_ENABLE_PIN       38

#define Y_DIR_PIN          4
#define Y_STEP_PIN         5
#define Y_ENABLE_PIN       56
/*
#define Z_DIR_PIN          48
#define Z_STEP_PIN         46
#define Z_ENABLE_PIN       62

#define A_DIR_PIN          28
#define A_STEP_PIN         26
#define A_ENABLE_PIN       24

#define B_DIR_PIN          34
#define B_STEP_PIN         36
#define B_ENABLE_PIN       30

#define C_DIR_PIN          32
#define C_STEP_PIN         47
#define C_ENABLE_PIN       45


#define X_STEP_HIGH             PORTF |=  0b00000001;
#define X_STEP_LOW              PORTF &= ~0b00000001;

#define Y_STEP_HIGH             PORTF |=  0b01000000;
#define Y_STEP_LOW              PORTF &= ~0b01000000;

#define Z_STEP_HIGH             PORTL |=  0b00001000;
#define Z_STEP_LOW              PORTL &= ~0b00001000;

#define A_STEP_HIGH             PORTA |=  0b00010000;
#define A_STEP_LOW              PORTA &= ~0b00010000;

#define B_STEP_HIGH             PORTC |=  0b00000010;
#define B_STEP_LOW              PORTC &= ~0b00000010;

#define C_STEP_HIGH             PORTL |=  0b00000100;
#define C_STEP_LOW              PORTL &= ~0b00000100;
*/
#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);
//ohmmeter begin 
bool temp11 = true;
const int NUM=2; 
int sensorPin;
typedef struct
{
  float Vin;
  float Vout;
  float Rref;
  float R;
  int sensorValue;
  bool flag; // false means all is well true means wire is broken.
  int z;
  
}Station;
Station s[NUM];
//ohmmeter end

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned int minStepInterval;   // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  volatile unsigned int stepCount;         // number of steps completed in current movement
};

void xStep() {
  //X_STEP_HIGH
  digitalWrite(X_STEP_PIN,HIGH);
 // X_STEP_LOW
  digitalWrite(X_STEP_PIN,LOW);
}
void xDir(int dir) {
  digitalWrite(X_DIR_PIN, dir);
}

void yStep() {
  //Y_STEP_HIGH
  //Y_STEP_LOW
  digitalWrite(Y_STEP_PIN,HIGH);
  digitalWrite(Y_STEP_PIN,LOW);
}
void yDir(int dir) {
  digitalWrite(Y_DIR_PIN, dir);
}

bool ohmmeter ()
{
  //sensorValue0 = analogRead(sensorPin0);
   for(int i=0; i<NUM; i++)
   {
      switch(i){
        case 0: 
          sensorPin=A0;
          break;
        case 1:
          sensorPin=A1;
          break;
        case 2:
          sensorPin=A2;
          break;
        case 3:
          sensorPin=A3;
          break;
        case 4:
          sensorPin=A4;
          break;
                }
      if(s[i].flag==false)
      {
      float temp;
      float temp2;
      temp=s[i].R;
      temp2=s[i].R;
      float p21= temp * 1.20;
      float m21= temp * .80;
      float p22= temp2 * 1.20;
      float m22= temp2 * .80;
      s[i].sensorValue=analogRead(sensorPin);
      s[i].Vout=(s[i].Vin * s[i].sensorValue)/1023;
      s[i].R = s[i].Rref * (1/((s[i].Vin / s[i].Vout)-1));
      if(s[i].R>p21 || s[i].R<m21 )
        { //Serial.print("WOW1");
          if(s[i].z==0)
          { //Serial.print("WOW2");
            s[i].flag=true;
          }

        }
       if(s[i].R>p22 || s[i].R<m22 )
        { //Serial.print("WOW1");
          if(s[i].z==0)
          { //Serial.print("WOW2");
            s[i].flag=true;
          }

        }
      if(s[i].z==1)
        {
          s[i].z=0;
          //Serial.print("DONE");
        
        }
        /*
      Serial.print("station __");
      Serial.print(i);
      Serial.print("value:");
      Serial.println(s[i].R);
     
      delay(1000);
      */
      Serial.print("SUS");
      return true;
      }
      
      else 
      {
      /*
       Serial.print("station __");
      Serial.print(i);
      Serial.print("CABLE BROKEN");
      Serial.println();
      delay(1000);
      */
      Serial.print("CABLE BROKEN");
      return false;
      }
   }
     
}
/*
void zStep() {
  Z_STEP_HIGH
  Z_STEP_LOW
}
void zDir(int dir) {
  digitalWrite(Z_DIR_PIN, dir);
}

void aStep() {
  A_STEP_HIGH
  A_STEP_LOW
}
void aDir(int dir) {
  digitalWrite(A_DIR_PIN, dir);
}

void bStep() {
  B_STEP_HIGH
  B_STEP_LOW
}
void bDir(int dir) {
  digitalWrite(B_DIR_PIN, dir);
}

void cStep() {
  C_STEP_HIGH
  C_STEP_LOW
}
void cDir(int dir) {
  digitalWrite(C_DIR_PIN, dir);
}
*/
void resetStepperInfo( stepperInfo& si ) {
  si.n = 0;
  si.d = 0;
  si.di = 0;
  si.stepCount = 0;
  si.rampUpStepCount = 0;
  si.totalSteps = 0;
  si.stepPosition = 0;
  si.movementDone = false;
}

#define NUM_STEPPERS 2

volatile stepperInfo steppers[NUM_STEPPERS];

void setup() {
  Serial.begin(9600);
  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
/*
  pinMode(Z_STEP_PIN,   OUTPUT);
  pinMode(Z_DIR_PIN,    OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  pinMode(A_STEP_PIN,   OUTPUT);
  pinMode(A_DIR_PIN,    OUTPUT);
  pinMode(A_ENABLE_PIN, OUTPUT);

  pinMode(B_STEP_PIN,   OUTPUT);
  pinMode(B_DIR_PIN,    OUTPUT);
  pinMode(B_ENABLE_PIN, OUTPUT);

  pinMode(C_STEP_PIN,   OUTPUT);
  pinMode(C_DIR_PIN,    OUTPUT);
  pinMode(C_ENABLE_PIN, OUTPUT);
*/
  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);
  /*
  digitalWrite(Z_ENABLE_PIN, LOW);
  digitalWrite(A_ENABLE_PIN, LOW);
  digitalWrite(B_ENABLE_PIN, LOW);
  digitalWrite(C_ENABLE_PIN, LOW);
  */
   for(int i=0; i<NUM; i++)
  {
    s[i].Vin=5;
    s[i].Vout=0;
    s[i].Rref=100;
    s[i].R=0;
    s[i].sensorValue=0;
    s[i].flag=false;
    s[i].z=1;
    Serial.print(i);
    
  }

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();

  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = 5000;
  steppers[0].minStepInterval = 80;

  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = 5000;
  steppers[1].minStepInterval = 80;
/*
  steppers[2].dirFunc = cDir;
  steppers[2].stepFunc = cStep;
  steppers[2].acceleration = 1000;
  steppers[2].minStepInterval = 50;

  steppers[3].dirFunc = xDir;
  steppers[3].stepFunc = xStep;
  steppers[3].acceleration = 1000;
  steppers[3].minStepInterval = 250;

  steppers[4].dirFunc = yDir;
  steppers[4].stepFunc = yStep;
  steppers[4].acceleration = 1000;
  steppers[4].minStepInterval = 50;

  steppers[5].dirFunc = zDir;
  steppers[5].stepFunc = zStep;
  steppers[5].acceleration = 1000;
  steppers[5].minStepInterval = 450;
  */
}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  si.stepCount = 0;
  si.n = 0;
  si.rampUpStepCount = 0;
  si.movementDone = false;
}

volatile byte remainingSteppersFlag = 0;

void prepareMovement(int whichMotor, int steps) {
  volatile stepperInfo& si = steppers[whichMotor];
  si.dirFunc( steps < 0 ? HIGH : LOW );
  si.dir = steps > 0 ? 1 : -1;
  si.totalSteps = abs(steps);
  resetStepper(si);
  remainingSteppersFlag |= (1 << whichMotor);
}

volatile byte nextStepperFlag = 0;

volatile int ind = 0;
volatile unsigned int intervals[100];

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned int mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;

    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

    volatile stepperInfo& s = steppers[i];

    if ( s.stepCount < s.totalSteps ) {
      s.stepFunc();
      s.stepCount++;
      s.stepPosition += s.dir;
      if ( s.stepCount >= s.totalSteps ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    if ( s.rampUpStepCount == 0 ) {
      s.n++;
      s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      if ( s.d <= s.minStepInterval ) {
        s.d = s.minStepInterval;
        s.rampUpStepCount = s.stepCount;
      }
      if ( s.stepCount >= s.totalSteps / 2 ) {
        s.rampUpStepCount = s.stepCount;
      }
    }
    else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }

    s.di = s.d; // integer
  }

  setNextInterruptInterval();

  TCNT1  = 0;
}

void runAndWait() {
  setNextInterruptInterval();
  ohmmeter();
  while ( remainingSteppersFlag );
}

void loop() {

  TIMER1_INTERRUPTS_ON
/*
  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < NUM_STEPPERS; k++) {
      prepareMovement( k,  200 );
     runAndWait();
    }
  }
  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < NUM_STEPPERS; k++) {
      prepareMovement( k,  200 );
    }
    runAndWait();
  }
  
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, 400 );
  runAndWait();
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, -400 );
  runAndWait();
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, 200 );
  runAndWait();
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, -200 );
  runAndWait();
  */
  for(int i=0; i<20; i++)
  {
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, 200 );
  runAndWait();
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, -200 );

  runAndWait();

  }
  while (true);

}
