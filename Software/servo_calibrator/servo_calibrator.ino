
// Calibration sketch to work out where a servo controlled tap is open or closed.
// John Spencer - 20120227

#include <Servo.h> 

Servo myservo;
byte pos;

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 

  Serial.println("Type a value to set position.");
  Serial.println("Note down when your tap is open and closed.");
  Serial.println("There is normally a 30deg difference either direction between closed and open.");
  Serial.println("e.g. Open=60, Closed=30");
} 

void loop() 
{
  if (Serial.available()) {
    // this will sit here waiting until there's a value.
  pos = serialReadInt();
  move_servo(pos);
  Serial.println("Type a value.");
  }
}

int serialReadInt()
{
   int i, serAva;                           // i is a counter, serAva hold number of serial available
 char inputBytes [7];                 // Array hold input bytes
 char * inputBytesPtr = &inputBytes[0];  // Pointer to the first element of the array
     
 if (Serial.available())            // Check to see if there are any serial input
 {
   delay(5);                              // Delay for terminal to finish transmitted
                                              // 5mS work great for 9600 baud (increase this number for slower baud)
   serAva = Serial.available();  // Read number of input bytes
   for (i=0; i<serAva; i++)       // Load input bytes into array
     inputBytes[i] = Serial.read();
   inputBytes[i] =  '\0';             // Put NULL character at the end
   return atoi(inputBytesPtr);    // Call atoi function and return result
 }
 else
   return -1;                           // Return -1 if there is no input
}

void move_servo(int pos){
  myservo.write(pos);
  delay(300);
  Serial.print("Finished move to :");
  Serial.println(myservo.read());
  delay(10);
}

