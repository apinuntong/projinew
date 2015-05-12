#include <Wire.h>
 
byte iInput=0;
byte iOutput=0;
 
void setup()
{
  Serial.begin(9600);
  Wire.begin();
}
 
void loop()
{
  Wire.requestFrom(0x32,1);// Begin transmission to PCF8574 with the buttons
  if(Wire.available())   // If bytes are available to be recieved
  {
    iInput = Wire.read();// Read a byte
    Serial.println(iInput);
  }
 
}
