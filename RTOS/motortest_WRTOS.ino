#include <Arduino_FreeRTOS.h>
const int stepMotorAD = 2; //dir
const int stepMotorAP = 3; //step
const int stepMotorAS = 9; //dir
const int stepMotorAE = 10; //step
const int stepMotorAInt = 2;
const int pd = 50;
volatile byte AH = LOW;
volatile byte AL = LOW; 

void Motor1(void *pvParameters );
void Motor2(void *pvParameters );
void Taskprint( void *pvParameters );
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  xTaskCreate(
    Motor1
    ,  "task1"   
    ,  128  
    ,  NULL
    ,  1  
    ,  NULL );
  xTaskCreate(
    Motor2
    ,  "task2"
    ,  128  
    ,  NULL
    ,  1  
    ,  NULL );
    xTaskCreate(
    Taskprint
    ,  "task3"
    ,  128  
    ,  NULL
    ,  2  
    ,  NULL );
vTaskStartScheduler();
}
void loop()
{
}
void Motor1(void *pvParameters)  {
  pinMode(stepMotorAD, OUTPUT); // dir// can these be moved to setup?
  pinMode(stepMotorAP, OUTPUT);/// step

  while(1)
  {

    digitalWrite(stepMotorAD, HIGH);
    for (int i = 0; i<10000; i++)
    {
      digitalWrite(stepMotorAP,HIGH);
      delayMicroseconds(pd);
      digitalWrite(stepMotorAP,LOW);
      delayMicroseconds(pd);
    }
    digitalWrite(stepMotorAD,LOW);
    for (int i = 0; i<10000; i++)
    {
      digitalWrite(stepMotorAP, HIGH);
      delayMicroseconds(pd);
      digitalWrite(stepMotorAP,LOW);
      delayMicroseconds(pd);
    }
    /*
    Serial.println("Task1");
    digitalWrite(8, HIGH);   
    vTaskDelay( 200 / portTICK_PERIOD_MS ); 
    digitalWrite(8, LOW);    
    vTaskDelay( 200 / portTICK_PERIOD_MS ); 
    */
  }
}
void Motor2(void *pvParameters)  
{
 pinMode(stepMotorAS, OUTPUT);
 pinMode(stepMotorAE, OUTPUT);
  
  while(1)
  {
    for (int i = 0; i<10000; i++)
    {
      digitalWrite(stepMotorAE,HIGH);
      delayMicroseconds(pd);
      digitalWrite(stepMotorAE,LOW);
      delayMicroseconds(pd);
    }
      digitalWrite(stepMotorAS,LOW);  // put your main code here, to run repeatedly:

    for (int i = 0; i<10000; i++)
    {
      digitalWrite(stepMotorAE, HIGH);
      delayMicroseconds(pd);
      digitalWrite(stepMotorAE,LOW);
      delayMicroseconds(pd);
    }
    // Not sure if we need the vTaskDelay
    /*
    Serial.println("Task2");
    digitalWrite(7, HIGH);   
    vTaskDelay( 300 / portTICK_PERIOD_MS ); 
    digitalWrite(7, LOW);   
    vTaskDelay( 300 / portTICK_PERIOD_MS ); 
    */
  }
}
void Taskprint(void *pvParameters)  {
  int counter = 0;
  while(1)
  {
counter++;
  Serial.println(counter); 
  vTaskDelay(500 / portTICK_PERIOD_MS);    }
}
