/*
*  This code is in the public domain.
*  (Do whatever you want with it.)
*/

//lower limit is 764

// Need the Servo library
#include <Servo.h>

// This is our motor.
Servo one,two,three,four;

// This is the final output
// written to the motor.
String incomingString;
int last = -1;

// Set everything up
void setup()
{
  // Put the motor to Arduino pin #9
  one.attach(6,764,2000);
  two.attach(5,764,2000);
  three.attach(4,764,2000);
  four.attach(7,764,2000);

  // Required for I/O from Serial monitor
  Serial.begin(9600);
  // Print a startup message
  Serial.println("initializing");
}


void loop()
{
  // If there is incoming value
  if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();
  
    /*
    *  If ch isn't a newline
    *  (linefeed) character,
    *  we will add the character
    *  to the incomingString
    */
    if (ch != 10){
      // Print out the value received
      // so that we can see what is
      // happening
      Serial.print("I have received: ");
      Serial.print(ch, DEC);
      Serial.print('\n');
    
      // Add the character to
      // the incomingString
      incomingString += ch;
    }
    // received a newline (linefeed) character
    // this means we are done making a string
    else
    {
      // print the incoming string
      Serial.println("I am printing the entire string");
      Serial.println(incomingString);
    
      // Convert the string to an integer
      int val = incomingString.toInt();
    
      // print the integer
      Serial.println("Printing the value: ");
      Serial.println(val);
    
      /*
      *  We only want to write an integer between
      *  0 and 180 to the motor. 
      */
      if (val >= 764 && val <= 2000)
     {
       // Print confirmation that the
       // value is between 0 and 180
       Serial.println("Value is between 0 and 180");
       // Write to Servo
       one.writeMicroseconds(val);
       two.writeMicroseconds(val);
       three.writeMicroseconds(val);
       four.writeMicroseconds(val);
       last = val;
     }
     // The value is not between 0 and 180.
     // We do not want write this value to
     // the motor.
     else
     {
       Serial.println("Value is NOT between 0 and 180");
      
       // IT'S a TRAP!
       Serial.println("Error with the input");
     }
    
      // Reset the value of the incomingString
      incomingString = "";
    }
  }
}
