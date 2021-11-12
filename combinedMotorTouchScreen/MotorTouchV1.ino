#include <Adafruit_TFTLCD.h> 

#include <Adafruit_GFX.h>    
#include <TouchScreen.h>

#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

#define TS_MINX 204
#define TS_MINY 195
#define TS_MAXX 948
#define TS_MAXY 910

#define YP A5
#define XM A6
#define YM 8
#define XP 9

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

const int stepMotorAD = 3;
const int stepMotorAP = 4;
const int stepMotorAInt = 2;
const int pd = 3500;
volatile byte AH = LOW;
volatile byte AL = LOW; 
//volatile bool dir = LOW;

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

boolean buttonEnabled=true;
boolean value=true;
void setup() {

    pinMode(stepMotorAD, OUTPUT);
    pinMode(stepMotorAP, OUTPUT);
  
  Serial.begin(9600);
  Serial.print("Starting...");
  
  tft.reset();
  tft.begin(0x8357);
  tft.setRotation(1);
  
  tft.fillScreen(BLACK);

//  //Draw white frame
//  tft.drawRect(0,0,319,240,WHITE);
//  
  //Print Text
  tft.setCursor(10,30);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Welcome to the Electrically Integrated");

  //Print text 
  tft.setCursor(10,60);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("Cable Bend Cycle Test System Team!");
  
  //Print text 
  tft.setCursor(40,120);
  tft.setTextColor(RED);
  tft.setTextSize(2);
  tft.print("Start/Stop Motor below");
  
  //Create Red Button
  tft.fillRect(60,180,100,40, GREEN);
  tft.drawRect(60,180,100,40,WHITE);
  tft.setCursor(80,190);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print("Start!");
  
  //Create Red Button
  tft.fillRect(200,180,100,40, RED);
  tft.drawRect(200,180,100,40,WHITE);
  tft.setCursor(220,190);
  tft.setTextColor(BLACK);
  tft.setTextSize(2);
  tft.print("Stop!");

}

  void loop() 
{
  static int val;

  TSPoint p = ts.getPoint();  //Get touch point
  
  if (p.z > ts.pressureThreshhold) {

   Serial.print("X = "); Serial.print(p.x);
   Serial.print("\tY = "); Serial.print(p.y);
   Serial.print("Here is the status of buttonEnabled: "); Serial.print(buttonEnabled);
   Serial.print("Here is the status of value: "); Serial.print(value);
   Serial.print("\n");
   
   p.x = map(p.x, TS_MAXX, TS_MINX, 0, 480);
   p.y = map(p.y, TS_MAXY, TS_MINY, 0, 320);
       
   if(p.x>300 && p.x<400 && (p.y>100 && p.y<200) && buttonEnabled) 
   // The user has pressed inside the red rectangle
   {
    val=0;
    
    buttonEnabled = true; //Disable button
    value=true;
    //This is important, because the libraries are sharing pins
   pinMode(XM, OUTPUT);
   pinMode(YP, OUTPUT);
    
    //Erase the screen
   
 
//    //Draw frame
//    tft.drawRect(0,0,480,320,WHITE);
  /*
      tft.setCursor(50,50);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("\tMotor\n\n  Stopping!");
    */
    //tft.fillScreen(RED);
   }
   //delay(10);  
  
  }

  if(p.x>=300 && p.x<=400 && (p.y>220 && p.y<350) && buttonEnabled) 
   // The user has pressed inside the red rectangle
   {
    buttonEnabled = true; //Disable button
    value=false;
     val=1;
    //This is important, because the libraries are sharing pins
   pinMode(XM, OUTPUT);
   pinMode(YP, OUTPUT);
    
    //Erase the screen
    //tft.fillScreen(GREEN);
    
//    //Draw frame
//    tft.drawRect(0,0,480,320,WHITE);
  /*
     tft.setCursor(50,50);
    tft.setTextColor(WHITE);
    tft.setTextSize(3);
    tft.print("\tMotor\n\Starting!");
*/
   }
   //delay(10);  

   //motor code below
   if (value == true){
    // put your main code here, to run repeatedly:
  digitalWrite(stepMotorAD, HIGH);
  for (int i = 0; i<100; i++)
  {
    //digitalWrite(stepMotorAD, dir);
    digitalWrite(stepMotorAP, HIGH);
    delayMicroseconds(pd);
    digitalWrite(stepMotorAP,LOW);
    delayMicroseconds(pd);
  }
  digitalWrite(stepMotorAD, LOW);  // put your main code here, to run repeatedly:
    for (int i = 0; i<100; i++)
  {
   // digitalWrite(stepMotorAD, dir);
    digitalWrite(stepMotorAP, HIGH);
    delayMicroseconds(pd);
    digitalWrite(stepMotorAP,LOW);
    delayMicroseconds(pd);
  }

   }

}
