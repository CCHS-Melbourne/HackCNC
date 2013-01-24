/*
This is a sketch for quickly testing the vital functions of the hackCNC

*/

#include <digitalWriteFast.h> // http://code.google.com/p/digitalwritefast/
#include <LiquidCrystal_SR_LCD3.h> // https://github.com/marcmerlin/NewLiquidCrystal

#include <Servo.h> 

//Servo settings MUST BE AN INT! BETWENEN 0-180
//servoDownZ is where your pen is engaged - pressing down 
#define servoDownZ 90
//servoUpZ is where your pen is disengaged - let up.
#define servoUpZ 160

// LCD Stuff
const int PIN_LCD_STROBE         =  2;  // Out: LCD IC4094 shift-register strobe
const int PIN_LCD_DATA           =  3;  // Out: LCD IC4094 shift-register data
const int PIN_LCD_CLOCK          =  4;  // Out: LCD IC4094 shift-register clock
LiquidCrystal_SR_LCD3 lcd(PIN_LCD_DATA, PIN_LCD_CLOCK, PIN_LCD_STROBE);

Servo myservo;
byte pos;


#define BAUD 115200

#define stepsPerMmX 1280
#define stepsPerMmY 1280

// step pins 
#define stepPin0 10 //x step D10 - pin 30
#define stepPin1 8 //y step D8 - pin 28

// dir pins 
#define dirPin0 11 //x dir D11 - pin 12
#define dirPin1 9 //y dir D9 - pin 29

#define minStepTime 200

void setup()
{
  //Servo pin 
  myservo.attach(12); //Pin 26 - PD6 or D12
  //Go home.
  move_servo(1);

  lcd.begin(20, 4);               // initialize the lcd 
  lcd.home ();                   // go home
  lcd.setCursor (0, 0);
  lcd.print(F("hackCNC-FunctionTest"));

  // Setup step pins.
  pinMode(stepPin0,OUTPUT);
  pinMode(stepPin1,OUTPUT);

  // Setup dir pins.
  pinMode(dirPin0,OUTPUT);
  pinMode(dirPin1,OUTPUT);


  lcd.setCursor (0, 1);
  lcd.print(F("Startup Complete.   "));
  lcd.setCursor (0, 2);
  lcd.print(F("Waiting For Serial. "));
  lcd.setCursor (0, 3);
  lcd.print(F("                    ")); // 20 spaces! must be a better way

  // Setup serial link.
  Serial.begin(BAUD);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo/32u4
  }
}

void move_servo(float pos){
  int diff=75;
  if(pos<=-1.0){
    myservo.write(servoDownZ);  
  }
  else if (pos>=0.0){
    myservo.write(servoUpZ);    
  }
  else{
    diff = servoUpZ-servoDownZ;
    //doesn't matter if diff is pos/neg - handles just fine.
    myservo.write(servoUpZ+pos*diff);
  }

}

char buff;

void loop()
{

  int i=0;
  // listen for serial commands
  while(Serial.available() > 0) {
    buff = Serial.read();
    if(buff=='s' || buff=='S'){
      lcd.setCursor (0, 3);
      lcd.print(F("Drawing 10mm Square."));
      move_servo(-0.75);
      
/* I HAVE NO IDEA WHY THIS WON'T WORK!!!

      //Y+ Movement
      digitalWriteFast(dirPin1,true);
      for(i=0;i<stepsPerMmY*10;i++){
        digitalWriteFast(stepPin1,HIGH);
        delayMicroseconds(minStepTime);
        digitalWriteFast(stepPin1,LOW);
        delayMicroseconds(minStepTime);
      }

      //X+ Movement
      digitalWriteFast(dirPin0,true);
      for(i=0;i<stepsPerMmX*10;i++){
        digitalWriteFast(stepPin0,HIGH);
        delayMicroseconds(minStepTime);
        digitalWriteFast(stepPin0,LOW);
        delayMicroseconds(minStepTime);
      }

      //Y- Movement
      digitalWriteFast(dirPin1,false);
      for(i=0;i<stepsPerMmY*10;i++){
        digitalWriteFast(stepPin1,HIGH);
        delayMicroseconds(minStepTime);
        digitalWriteFast(stepPin1,LOW);
        delayMicroseconds(minStepTime);
      }

      //X- Movement
      digitalWriteFast(dirPin0,false);
      for(i=0;i<stepsPerMmX*10;i++){
        digitalWriteFast(stepPin0,HIGH);
        delayMicroseconds(minStepTime);
        digitalWriteFast(stepPin0,LOW);
        delayMicroseconds(minStepTime);
      }
*/


      //Y+ Movement
      digitalWrite(dirPin1,true);
      for(i=0;i<stepsPerMmY*10;i++){
        digitalWrite(stepPin1,HIGH);
        delayMicroseconds(minStepTime);
        digitalWrite(stepPin1,LOW);
        delayMicroseconds(minStepTime);
      }

      //X+ Movement
      digitalWrite(dirPin0,true);
      for(i=0;i<stepsPerMmX*10;i++){
        digitalWrite(stepPin0,HIGH);
        delayMicroseconds(minStepTime);
        digitalWrite(stepPin0,LOW);
        delayMicroseconds(minStepTime);
      }

      //Y- Movement
      digitalWrite(dirPin1,false);
      for(i=0;i<stepsPerMmY*10;i++){
        digitalWrite(stepPin1,HIGH);
        delayMicroseconds(minStepTime);
        digitalWrite(stepPin1,LOW);
        delayMicroseconds(minStepTime);
      }

      //X- Movement
      digitalWrite(dirPin0,false);
      for(i=0;i<stepsPerMmX*10;i++){
        digitalWrite(stepPin0,HIGH);
        delayMicroseconds(minStepTime);
        digitalWrite(stepPin0,LOW);
        delayMicroseconds(minStepTime);
      }

      move_servo(0);
      lcd.setCursor (0, 3);
      lcd.print(F("Done Drawing Square."));
    }
    else{
      lcd.setCursor (0, 3);
      lcd.print(F("I ONLY DRAW SQUARES.")); 
    }

  }
}
