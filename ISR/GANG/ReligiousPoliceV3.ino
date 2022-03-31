
#define dirPin1 2     //stepper1 direction_pin
#define stepPin1 3    //stepper1 step_pin
#define dirPin2 4   //stepper2 direction_pin
#define stepPin2 5   //stepper2 step_pin
#define dirPin3 6    //stepper3 direction_pin
#define stepPin3 7   //stepper3 step_pin
#define dirPin4 8    //stepper4 direction_pin
#define stepPin4 9   //stepper4 step_pin
#define dirPin5 10   //stepper5 direction_pin
#define stepPin5 11  //stepper5 step_pin

#define HALL_SENSOR 12 //hall sensor pin 

//#define stepsPerRevolution 10000 //steps per revolution for each motor

#define X_DIR_PIN          2
#define X_STEP_PIN         3
#define X_ENABLE_PIN       38

#define Y_DIR_PIN          4
#define Y_STEP_PIN         5
#define Y_ENABLE_PIN       56

//WE THE FUCKING BEST
String data;   // string variable to store incoming serial data
bool start_flag=false; // bool variable to store the start and pause status
bool start_calibrate=false;

// OHMMETER START
bool temp11 = true;
const int NUM=1; 
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


// OHMMETER END


float motor_speed=0;
float motor1_speed=0;
float motor2_speed=0;
float motor3_speed=0;
float motor4_speed=0;
float motor5_speed=0;

unsigned long myTime;

//float baseMotorSpeed1 = 87;
float stepperMotorSpeed1=0;
float stepperMotorSpeed2=0;
float stepperMotorSpeed3=0;
float stepperMotorSpeed4=0;
float stepperMotorSpeed5=0;

float angle_diff1;
float speed_diff1;
float speed_fast1;

int hallstate =0;

float start_angle=0;
float start_angle1=0;
float start_angle2=0; 
float start_angle3=0; 
float start_angle4=0; 
float start_angle5=0; 
 
float stop_angle=0;
float stop_angle1=0;
float stop_angle2=0;
float stop_angle3=0;
float stop_angle4=0;
float stop_angle5=0;

float temp = angle_diff1;
int val = (int) temp;

int cycle_count=0;
int cycle_count1=0;
int cycle_count2=0;
int cycle_count3=0;
int cycle_count4=0;
int cycle_count5=0;

int counter=0;
int counter1=0;
int counter2=0;
int counter3=0;
int counter4=0;
int counter5=0;

String station1_status;
String station2_status;
String station3_status;
String station4_status;
String station5_status;

bool s1=false;
bool s2=false;
bool s3=false;
bool s4=false;
bool s5=false;

#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

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
void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
  // step and pins 1-5 are from the old code
  /*
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
  pinMode(stepPin5, OUTPUT);
  pinMode(dirPin5, OUTPUT);
*/
  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);

  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);

  digitalWrite(X_ENABLE_PIN, LOW);
  digitalWrite(Y_ENABLE_PIN, LOW);

  
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
  //ohmmeter();
  Serial.print("HERE");
  while ( remainingSteppersFlag ); /// THIS may be the issue
  Serial.println("WAKEYWAKEY");
}
void loop() 
{ 
serial_data();
if((start_flag==true)&&(station1_status=="ON"))  //if start button is pressed and station1 button is pressed
 {
  //stepper_motor1(); // stepper1 motor function call
  Serial.print("test1");
  if(counter1>=cycle_count1||temp11==false)  // if counter1 is greater than entered count cycle then reset it.
  {
    motor1_speed=0;
    start_angle1=0;
    stop_angle1=0;
    cycle_count1=0;
    counter1=0;
    s[0].Vin=5;
    s[0].Vout=0;
    s[0].Rref=100;
    s[0].R=0;
    s[0].sensorValue=0;
    s[0].flag=false;
    s[0].z=1;
    start_flag=false;
    s[0].flag=false;
    temp11= true;
  }

   TIMER1_INTERRUPTS_ON
  Serial.println("WTF");

  
  for(int i=0; i<cycle_count; i++)
  {
  //ohmmeter();
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, 400 );
  runAndWait();
  for (int i = 0; i < NUM_STEPPERS; i++)
    prepareMovement( i, -400 );
    counter1=counter1+1;
  Serial.print(counter1);
  runAndWait();
  
  }
  Serial.print("FUCK");
  //goto label;
  //TIMER1_INTERRUPTS_OFF
  //while (true);
  Serial.print("FUCK YOU");
   start_flag=false;
 // TIMER1_INTERRUPTS_OFF
 }


}

//////////////////////////////////////////////
//Calibration function
void calibrate(){
  pinMode (HALL_SENSOR, INPUT);
  //Serial.print("HELLO MOFO");
    digitalWrite(dirPin1, HIGH);
    digitalWrite(dirPin2, HIGH);
    
        for (int i = 0; i < 50; i++)     // These four lines result in 1 step:
        {
          digitalWrite(stepPin1, HIGH);
          digitalWrite(stepPin2, HIGH);
          delayMicroseconds(1500);
          digitalWrite(stepPin1, LOW);
          digitalWrite(stepPin2, LOW);
          delayMicroseconds(1500);
          
       }
        hallstate = digitalRead(HALL_SENSOR);
        //Serial.println(hallstate);

        while(hallstate == 1){
          hallstate=digitalRead(HALL_SENSOR);
          //Serial.println(hallstate);
          digitalWrite(dirPin1, LOW);
          digitalWrite(dirPin2, LOW);
         
          for (int i =0; i<0.1; i++){
            digitalWrite(stepPin1, HIGH);
            delayMicroseconds(3000);
            digitalWrite(stepPin2, HIGH);
            delayMicroseconds(3000);
            digitalWrite(stepPin1, LOW);
            delayMicroseconds(3000);
            digitalWrite(stepPin2, LOW);
            delayMicroseconds(3000);
          }
          Serial.println(hallstate);
        }
        if (hallstate == 0){
          Serial.println(hallstate);
          }
        

}

//////////////////////////////////////////////
void serial_data()
{
 while(Serial1.available())  //wait until serial data is available
    {   
     if(Serial1.available() > 0) // if available data is greater than zero
       {
        data=Serial1.readString(); //Read serial data on Rx1,Tx1
        Serial.println(data);      // Print serial data
        if(data.startsWith("CALIBRATE"))
        {start_calibrate=true;
         calibrate();
        }

        else{ start_calibrate=false;}
        if(data.startsWith("START"))  
        {start_flag=true;}
        
       if(data.startsWith("PAUSE"))
        {start_flag=false;}

        if(data.startsWith("s1"))
        {
          s1=true;
          s2=false;
          s3=false;
          s4=false;
          s5=false;
        }

        if(data.startsWith("s2"))
        {
          s1=false;
          s2=true;
          s3=false;
          s4=false;
          s5=false;
        }

        if(data.startsWith("s3"))
        {
          s1=false;
          s2=false;
          s3=true;
          s4=false;
          s5=false;
        }

        if(data.startsWith("s4"))
        {
          s1=false;
          s2=false;
          s3=false;
          s4=true;
          s5=false;
        }

        if(data.startsWith("s5"))
        {
          s1=false;
          s2=false;
          s3=false;
          s4=false;
          s5=true;
        }
  
       if(data.startsWith("save"))
          {
        int start_angle_index = data.indexOf("start_angle");
        int stop_angle_index = data.indexOf("stop_angle");
        int cycle_count_index= data.indexOf("cycle_count");
        int slider_index= data.indexOf("slider");
        int station1_index=data.indexOf("station1=");
        int station2_index=data.indexOf("station2=");
        int station3_index=data.indexOf("station3=");
        int station4_index=data.indexOf("station4=");
        int station5_index=data.indexOf("station5=");
        
       String String_start_angle = data.substring(start_angle_index+11, stop_angle_index);
       start_angle=String_start_angle.toInt();
       Serial.print("start_angle=");
       Serial.println(start_angle);
       //////////////
       String String_stop_angle = data.substring(stop_angle_index+10,cycle_count_index);
       stop_angle=String_stop_angle.toInt();
       Serial.print("stop_angle=");
       Serial.println(stop_angle);
       //////////////
       String String_cycle_count=data.substring(cycle_count_index+11);
       cycle_count=String_cycle_count.toInt();
       Serial.print("cycle_count1=");
       Serial.println(cycle_count);
       /////////////
       String String_motor_speed=data.substring(slider_index+6);
       motor_speed=String_motor_speed.toInt();
       Serial.print("motor_speed=");
       Serial.println(motor_speed);
        /////////////
       String string_station1=data.substring(station1_index+9,station2_index);
       station1_status=string_station1;
       Serial.print("station1_status=");
       Serial.println(station1_status);
       //////////////
       String string_station2=data.substring(station2_index+9,station3_index);
       station2_status=string_station2;
       Serial.print("station2_status=");
       Serial.println(station2_status);
       //////////////
       String string_station3=data.substring(station3_index+9,station4_index);
       station3_status=string_station3;
       Serial.print("station3_status=");
       Serial.println(station3_status);
       /////////////
       String string_station4=data.substring(station4_index+9,station5_index);
       station4_status=string_station4;
       Serial.print("station4_status=");
       Serial.println(station4_status);
       /////////////
       String string_station5=data.substring(station5_index+9);
       station5_status=string_station5;
       Serial.print("station5_status=");
       Serial.println(station5_status);
       /////////////


 /////------------------------------------------------------------------------------------------ 
         if(station1_status=="ON")
          {
             angle_diff1 = (stop_angle-start_angle); // find angle diff
             start_angle1=(start_angle*55.6); // start and stop angle for FOR loop (out of 20000 steps)
             stop_angle1=(stop_angle*55.6);
             cycle_count1=cycle_count; // how many cycles to operate for.
             
             if(motor_speed==1) // if the user chooses SLOW aka "1"
              {
            
                   
                  if(angle_diff1>=0 && angle_diff1<=45) // if angle is between 0 - 45
                    {
                      motor1_speed=184;  // <==== REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>45 && angle_diff1<=90) // if angle is between 45-90
                    {
                       motor1_speed=84; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>90 && angle_diff1<=135) // if angle is between 90-135
                    {
                      motor1_speed=51; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>135 && angle_diff1<=180) //if angle is between 135 and 180
                    {

                      motor1_speed=35; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                
                
               } // end of 1
               
           if(motor_speed==2) // if the user chooses MED aka "2"
              {
            
                   
                  if(angle_diff1>=0 && angle_diff1<=45) // if angle is between 0 - 45
                    {
                      motor1_speed=134;  // <==== REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>45 && angle_diff1<=90) // if angle is between 45-90
                    {
                       motor1_speed=59; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>90 && angle_diff1<=135) // if angle is between 90-135
                    {
                      motor1_speed=35; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>135 && angle_diff1<=180) //if angle is between 135 and 180
                    {

                      motor1_speed=22; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                
                
               } // end of 2
               
             if(motor_speed==3) // if the user chooses FAST aka "3"
              {
            
                  //Serial.print("test"); 
                  if(angle_diff1>=0 && angle_diff1<=45) // if angle is between 0 - 45
                    {
                      motor1_speed=87;  // <==== REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>45 && angle_diff1<=90) // if angle is between 45-90
                    {
                       motor1_speed=35; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>90 && angle_diff1<=135) // if angle is between 90-135
                    {
                      motor1_speed=18; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff1>135 && angle_diff1<=180) //if angle is between 135 and 180
                    {

                        motor1_speed=10; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                
                
               } // end of 3
            
            
          }
          else
            {
            motor1_speed=0;
            start_angle1=0;
            stop_angle1=0;
            cycle_count1=0;
            }



 ////------------------------------------------------------------------------------------------------
           if(station2_status=="ON")
            {
            motor2_speed=motor_speed;
            start_angle2=start_angle;
            stop_angle2=stop_angle;
            cycle_count2=cycle_count;
            }
          else
            {
            motor2_speed=0;
            start_angle2=0;
            stop_angle2=0;
            cycle_count2=0;
            }
            
            if(station3_status=="ON")
            {
            motor3_speed=motor_speed;
            start_angle3=start_angle;
            stop_angle3=stop_angle;
            cycle_count3=cycle_count;
            }
          else
            {
            motor3_speed=0;
            start_angle3=0;
            stop_angle3=0;
            cycle_count3=0;
            }
            
            if(station4_status=="ON")
            {
            motor4_speed=motor_speed;
            start_angle4=start_angle;
            stop_angle4=stop_angle;
            cycle_count4=cycle_count;
            }
          else
            {
            motor4_speed=0;
            start_angle4=0;
            stop_angle4=0;
            cycle_count4=0;
            }
            
            if(station5_status=="ON")
            {
            motor5_speed=motor_speed;
            start_angle5=start_angle;
            stop_angle5=stop_angle;
            cycle_count5=cycle_count;
            }
          else
            {
            motor5_speed=0;
            start_angle5=0;
            stop_angle5=0;
            cycle_count5=0;
            }
         }
        //Serial.print("start_flag=");
        //Serial.print(start_flag);
      }
     }
   }

 
//////////////////////////////////////////////////////////////////
/*
void stepper_motor1()
{
  //while(counter1<=cycle_count1)
  //{

  //serial_data();
  //if(start_flag==false)
  //{break;}
  //if (motor1_speed < 100 ) 
   //{motor1_speed = 0;}
   //stepperMotorSpeed1 = map(motor1_speed , 0, 1023, 4000, 0); // map the values of motor speed  0 to 4000 and 1023 to 0
  if (temp11==true){
    
   for (int i=0 ; i<cycle_count1; i++)
    {
      
      serial_data();
    //myTime = millis();

    //Serial.println(myTime); // prints time since program started
      step1clockwise();
      //Serial.println("clockwise");
      serial_data();
      step1anticlockwise();
      step1clockwise();
      //Serial.println("Anticlockwise");
      counter1=counter1+1;
      temp11 = ohmmeter(); //calling ohmmeter
      
      if(temp11==false){
        start_flag == false;
        //break;
      }
      s[0].flag ==false;
      Serial.print("**");
      Serial.println(temp11);
      
    }
    
  progress();
  //counter1=counter1+1;
  Serial.print("counter1="); 
  Serial.println(counter1); 
 //}
}
}
//////////////////////////////////////////////////////////////////



*/
///////////////////////////////////////
void progress()
{
if(s1==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counter1); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

   else if(s2==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counter2); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

    else if(s3==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counter3); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

    else if(s4==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counter4); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

    else if(s5==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counter5); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

}
////////////////////////////////
/*
void step1clockwise()
{ digitalWrite(dirPin1, LOW);
  for (int i = 0; i <((stop_angle1-start_angle1)/2); i++)     // These four lines result in 1 step:
  {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(motor1_speed);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(motor1_speed);
  }
}
void step1anticlockwise()
{ digitalWrite(dirPin1, HIGH);
  for (int i = 0; i < (stop_angle1-start_angle1); i++)     // These four lines result in 1 step:
  {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(motor1_speed);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(motor1_speed);
  }
}
*/

//////////////////////////
//OHMMETER
//////////////////////////

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
      temp=s[i].R;
      float p20= temp * 1.20;
      float m20= temp * .80;
      s[i].sensorValue=analogRead(sensorPin);
      s[i].Vout=(s[i].Vin * s[i].sensorValue)/1023;
      s[i].R = s[i].Rref * (1/((s[i].Vin / s[i].Vout)-1));
      if(s[i].R>p20 || s[i].R<m20 )
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
      //Serial.print("station __");
      //Serial.print(i);
      //Serial.print("value:");
      //Serial.println(s[i].R);
     
      //delay(1000);
      Serial.println(s[i].R);
      return true;
      
      }
      
      else 
      {
      
      //Serial.print("station __");
      //Serial.print(i);
      //Serial.print("CABLE BROKEN");
      //Serial.println();
      //delay(1000);
      Serial.print("CABLE BROKEN");
      return false;
      }
   }
     
}
