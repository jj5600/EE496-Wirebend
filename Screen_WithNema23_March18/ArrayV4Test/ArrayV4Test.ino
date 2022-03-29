#include <AccelStepper.h>
#define dirPin1 3     //stepper1 direction_pin
#define stepPin1 4    //stepper1 step_pin
#define dirPin2 2   //stepper2 direction_pin
#define stepPin2 5   //stepper2 step_pin
#define dirPin3 6    //stepper3 direction_pin
#define stepPin3 7   //stepper3 step_pin
#define dirPin4 8    //stepper4 direction_pin
#define stepPin4 9   //stepper4 step_pin
#define dirPin5 10   //stepper5 direction_pin
#define stepPin5 11  //stepper5 step_pin
#define stepsPerRevolution 10000 //steps per revolution for each motor


// CANT STOP WONT STOP
AccelStepper motor1 = (1, stepPin1, dirPin1);
AccelStepper motor2 = (1, stepPin2, dirPin2);
AccelStepper motor3 = (1, stepPin3, dirPin3);
AccelStepper motor4 = (1, stepPin4, dirPin4);
AccelStepper motor5 = (1, stepPin5, dirPin5);
AccelStepper motors[] = {motor1,motor2,motor3,motor4,motor5};
String data;   // string variable to store incoming serial data
bool start_flag=false; // bool variable to store the start and pause status

float motor_speed=0;
float motor_speedA[] = {0,0,0,0,0};

unsigned long myTime;

//float baseMotorSpeed1 = 87;
float stepperMotorSpeed[]={0,0,0,0,0};

float angle_diff[]={0.0, 0.0, 0.0, 0.0, 0.0};
float speed_diff[]={0.0, 0.0, 0.0, 0.0, 0.0};
float speed_fast[]={0.0, 0.0, 0.0, 0.0, 0.0};

float start_angle=0;
float start_angleA[] = {0,0,0,0,0};
 
float stop_angle=0;
float stop_angleA[]={0,0,0,0,0};

float temp = angle_diff[0];
int val = (int) temp;

int cycle_count=0;
int cycle_countA[]={0,0,0,0,0};

int counter=0;
int counterA[]= {0,0,0,0,0};

String station_status[5] ={"Off","Off","Off","Off","Off"};
bool SA[] = {false,false,false,false,false};


void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);
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

}
void loop() 
{ 
serial_data();
if((start_flag==true)&&(station_status[0]=="ON"))  //if start button is pressed and station1 button is pressed
 {
  stepper_motor(); // stepper1 motor function call
  Serial.print("test1");
  if(counterA[0]>=cycle_countA[0])  // if counter1 is greater than entered count cycle then reset it.
  { 
    motor_speedA[0]=0;
    start_angleA[0]=0;
    stop_angleA[0]=0;
    cycle_countA[0]=0;
    counterA[0]=0;
    start_flag=false;
  }
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
        if(data.startsWith("START"))  
        {start_flag=true;}
        
       if(data.startsWith("PAUSE"))
        {start_flag=false;}

        if(data.startsWith("s1"))
        {
          SA[0]=true;
        }

        if(data.startsWith("s2"))
        {
          SA[1]=true;
        }

        if(data.startsWith("s3"))
        {
          SA[2]=true;
        }

        if(data.startsWith("s4"))
        {
          SA[3]=true;
        }

        if(data.startsWith("s5"))
        {
          SA[4]=true;
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
       Serial.print("cycle count 1=");
       Serial.println(cycle_count);
       /////////////
       String String_motor_speed=data.substring(slider_index+6);
       motor_speed=String_motor_speed.toInt();
       Serial.print("motor_speed=");
       Serial.println(motor_speed);
        /////////////
       String string_station1=data.substring(station1_index+9,station2_index);
       station_status[0]=string_station1;
       Serial.print("station 1 status =");
       Serial.println(station_status[0]);
       //////////////
       String string_station2=data.substring(station2_index+9,station3_index);
       station_status[1]=string_station2;
       Serial.print("station 2 status=");
       Serial.println(station_status[1]);
       //////////////
       String string_station3=data.substring(station3_index+9,station4_index);
       station_status[2]=string_station3;
       Serial.print("station 3 status=");
       Serial.println(station_status[2]);
       /////////////
       String string_station4=data.substring(station4_index+9,station5_index);
       station_status[3]=string_station4;
       Serial.print("station 4 status=");
       Serial.println(station_status[3]);
       /////////////
       String string_station5=data.substring(station5_index+9);
       station_status[4]=string_station5;
       Serial.print("station 5 status=");
       Serial.println(station_status[4]);
        ///////////
 /////------------------------------------------------------------------------------------------ 
         for(int i = 0; i<5;i++)
         {
          if(station_status[i]=="ON")
            {
             angle_diff[i] = (stop_angle-start_angle); // find angle diff
             start_angleA[i]=(start_angle*55.6); // start and stop angle for FOR loop (out of 20000 steps)
             stop_angleA[i]=(stop_angle*55.6);
             cycle_countA[i]=cycle_count; // how many cycles to operate for.
             
             if(motor_speed==1) // if the user chooses SLOW aka "1"
              {
            
                   
                  if(angle_diff[i]>=0 && angle_diff[i]<=45) // if angle is between 0 - 45
                    {
                      motor_speedA[i]=184;  // <==== REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>45 && angle_diff[i]<=90) // if angle is between 45-90
                    {
                       motor_speedA[i]=84; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>90 && angle_diff[i]<=135) // if angle is between 90-135
                    {
                      motor_speedA[i]=51; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>135 && angle_diff[i]<=180) //if angle is between 135 and 180
                    {

                      motor_speedA[i]=35; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                
                
               } // end of 1
               
           if(motor_speed==2) // if the user chooses MED aka "2"
              {
            
                   
                  if(angle_diff[i]>=0 && angle_diff[i]<=45) // if angle is between 0 - 45
                    {
                      motor_speedA[0]=134;  // <==== REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>45 && angle_diff[i]<=90) // if angle is between 45-90
                    {
                       motor_speedA[0]=59; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>90 && angle_diff[i]<=135) // if angle is between 90-135
                    {
                      motor_speedA[0]=35; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>135 && angle_diff[i]<=180) //if angle is between 135 and 180
                    {

                      motor_speedA[0]=22; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                
                
               } // end of 2
               
             if(motor_speed==3) // if the user chooses FAST aka "3"
              {
            
                  //Serial.print("test"); 
                  if(angle_diff[i]>=0 && angle_diff[i]<=45) // if angle is between 0 - 45
                    {
                      motor_speedA[0]=87;  // <==== REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>45 && angle_diff[i]<=90) // if angle is between 45-90
                    {
                       motor_speedA[0]=35; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>90 && angle_diff[i]<=135) // if angle is between 90-135
                    {
                      motor_speedA[0]=18; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                  if(angle_diff[i]>135 && angle_diff[i]<=180) //if angle is between 135 and 180
                    {

                        motor_speedA[0]=10; //<====  REPLACE motor_speed with HARD CODED VALUE
                    }
                
                
               } // end of 3
            
            
          }
          else
            {
            motor_speedA[i]=0;
            start_angleA[i]=0;
            stop_angleA[i]=0;
            cycle_countA[i]=0;
            }          
      }
     }
   }
   }
}

 
//////////////////////////////////////////////////////////////////
void stepper_motor()
{
  //while(counter1<=cycle_count1)
  //{

  //serial_data();
  //if(start_flag==false)
  //{break;}
  //if (motor1_speed < 100 ) 
   //{motor1_speed = 0;}
   //stepperMotorSpeed1 = map(motor1_speed , 0, 1023, 4000, 0); // map the values of motor speed  0 to 4000 and 1023 to 0

   for (int i=0 ; i<cycle_countA[0]; i++)
    {
      
      serial_data();
         myTime = millis();

   Serial.println(myTime); // prints time since program started
      stepclockwise();
      //Serial.println("clockwise");
      serial_data();
      stepanticlockwise();
      stepclockwise();
      //Serial.println("Anticlockwise");
      counterA[0]=counterA[0]+1;
    }
    
  progress();
  //counter1=counter1+1;
  Serial.print("counter for motor 1="); 
  Serial.println(counterA[0]); 
 //}
}

//////////////////////////////////////////////////////////////////




///////////////////////////////////////
void progress()
{
if(SA[0]==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counterA[0]); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

   else if(SA[1]==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counterA[1]); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

    else if(SA[2]==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counterA[2]); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

    else if(SA[3]==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counterA[3]); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

    else if(SA[4]==true)
    {
      Serial1.print("t3.txt=");
      Serial1.print("\""); 
      Serial1.print(counterA[4]); 
      Serial1.print("\"");      
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }

}
////////////////////////////////
void stepclockwise()
{ digitalWrite(dirPin1, LOW);
  for (int i = 0; i <((stop_angleA[0]-start_angleA[0])/2); i++)     // These four lines result in 1 step:
  {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(motor_speedA[0]);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(motor_speedA[0]);
  }
}
void stepanticlockwise()
{ digitalWrite(dirPin1, HIGH);
  for (int i = 0; i < (stop_angleA[0]-start_angleA[0]); i++)     // These four lines result in 1 step:
  {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(motor_speedA[0]);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(motor_speedA[0]);
  }
}
