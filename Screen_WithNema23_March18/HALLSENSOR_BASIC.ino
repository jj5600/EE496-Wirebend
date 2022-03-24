const int hallPin = 2 ;     // initializing a pin for the sensor output

const int ledPin =  13 ; 


// initializing a pin for the led. Arduino has built in led attached to pin 13

// variables will change :

int hallState = 0 ;          // initializing a variable for storing the status of the hall sensor.

void setup ( ) {

  pinMode ( ledPin , INPUT ) ;      // This will initialize the LED pin as an output pin :

  pinMode ( hallPin , OUTPUT ) ;    
  
  
  // This will initialize the hall effect sensor pin as an input pin to the Arduino :

 Serial.begin( 9600 ) ;



}

void loop ( ) {

  hallState = digitalRead ( ledPin ) ;     
  Serial.println(hallState);
  // reading from the sensor and storing the state of the hall effect sensor :

  if ( hallState == HIGH ) {                                                // Checking whether the state of the module is high or low

    //Serial.println ( “ The state of the analog hall module is high ” ) ;
      Serial.println("BOI");                               // turn on the LED if he state of the module is high

  } 

  if(hallState == LOW)
  {

      Serial.println("GANG SHIT");                              // otherwise turn off the LED 
  }

}
