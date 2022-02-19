
const int NUM=5; 
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
void setup ()
{ 
  Serial.begin(9600);
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
  //Serial.begin(9600);      // Initialize serial communications at 9600 bps
  
}

void loop ()
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
      Serial.print("station __");
      Serial.print(i);
      Serial.print("value:");
      Serial.println(s[i].R);
     
      delay(1000);
      }
      else 
      {
       Serial.print("station __");
      Serial.print(i);
      Serial.print("CABLE BROKEN");
      Serial.println();
      delay(1000);
        
      }
   }
     
}
