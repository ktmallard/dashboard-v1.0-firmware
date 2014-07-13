/*
Function test for RGB eyes. Fades each color.
*/


int eyeBlue = 3;    // LED connected to digital pin 9
int eyeRed = 6;
int eyeGreen = 11;

void setup()  { 
  // nothing happens in setup 
  Serial.begin(9600);
} 

void loop()  { 
  // fade in from min to max in increments of 5 points:
  for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
    // sets the value (range from 0 to 255):
    analogWrite(eyeGreen, fadeValue);
    Serial.println("foo");    
    // wait for 30 milliseconds to see the dimming effect    
    delay(15);                            
  } 

  // fade out from max to min in increments of 5 points:
  for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=5) { 
    // sets the value (range from 0 to 255):
    analogWrite(eyeGreen, fadeValue);         
    // wait for 30 milliseconds to see the dimming effect    
    delay(15);                            
  } 
}


