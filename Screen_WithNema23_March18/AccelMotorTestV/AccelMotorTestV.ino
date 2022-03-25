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
String data;   // string variable to store incoming serial data
bool start_flag=false; // bool variable to store the start and pause status
AccelStepper motor1 = (1, stepPin1, dirPin1);
AccelStepper motor2 = (1, stepPin2, dirPin2);
AccelStepper motor3 = (1, stepPin3, dirPin3);
AccelStepper motor4 = (1, stepPin4, dirPin4);
AccelStepper motor5 = (1, stepPin5, dirPin5);
AccelStepper motors[] = {motor1,motor2,motor3,motor4,motor5}
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
if((start_flag==true)&&(station1_status=="ON"))  //if start button is pressed and station1 button is pressed
 {
  stepper_motor1(); // stepper1 motor function call
  Serial.print("test1");
  if(counter1>=cycle_count1)  // if counter1 is greater than entered count cycle then reset it.
  { 
    motor1_speed=0;
    start_angle1=0;
    stop_angle1=0;
    cycle_count1=0;
    counter1=0;
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
        ///////////
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
   motor1.setCurrentPosition();
   motor1.setMaxSpeed(motor1_speed);
   for (int i=0 ; i<cycle_count1; i++)
    {
      
      serial_data();
         myTime = millis();

   Serial.println(myTime); // prints time since program started
      motor1.runToNewPosition(stop_angle1);
      //Serial.println("clockwise");
      serial_data();
      motor1.runToNewPosition(start_angle1);
      motor1.runToNewPosition(0);
      //Serial.println("Anticlockwise");
      counter1=counter1+1;
    }
    
  progress();
  //counter1=counter1+1;
  Serial.print("counter1="); 
  Serial.println(counter1); 
 //}
}

//////////////////////////////////////////////////////////////////




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
//void step1clockwise()
//{ 
//  digitalWrite(dirPin1, LOW);
//  for (int i = 0; i <((stop_angle1-start_angle1)/2); i++)     // These four lines result in 1 step:
//  {
//    digitalWrite(stepPin1, HIGH);
//    delayMicroseconds(motor1_speed);
//    digitalWrite(stepPin1, LOW);
//    delayMicroseconds(motor1_speed);
//  }
//}
//void step1anticlockwise()
//{ digitalWrite(dirPin1, HIGH);
//  for (int i = 0; i < (stop_angle1-start_angle1); i++)     // These four lines result in 1 step:
//  {
//    digitalWrite(stepPin1, HIGH);
//    delayMicroseconds(motor1_speed);
//    digitalWrite(stepPin1, LOW);
//    delayMicroseconds(motor1_speed);
//  }
//}
