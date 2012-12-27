/*
This work is public domain.

Please note: Although there are a LOT pin settings here.
You can get by with as few as TWO pins per Axis. (dir & step)
ie: 3 axies = 6 pins used. (minimum)
    9 axies = 18 pins or an entire UNO (using virtual limits switches only)
    This version is cut down to 3 axis.

Note concerning switches: Be smart!
  AT LEAST use HOME switches.
  Switches are cheap insurance.
  You'll find life a lot easier if you use them entirely.
  
  If you choose to build with threaded rod for lead screws but leave out the switches
  You'll have one of two possible outcomes;
    You'll get tired really quickly of resetting the machine by hand.
    Or worse, you'll forget (only once) to reset it, and upon homing
    it WILL destroy itself while you go -> WTF!? -> OMG! -> PANIC! -> FACEPALM!

 List of axies. All 9 of them.
     AXIS_0 = X (Left/Right)
     AXIS_1 = Y (Near/Far) Lathes use this for tool depth.
     AXIS_2 = Z (Up/Down) Not typically used for lathes. Except lathe/mill combo.


  DYI robot builders: You can monitor/control this sketch via a serial interface.
  Example commands:
  
    jog x200;
    jog x-215.25 y1200 z0.002 a5;
    
    PS: If you choose to control this with your own interface then also modify the
    divisor variable further down.
*/

// You'll need this library. Get the interrupt safe version.
#include <digitalWriteFast.h> // http://code.google.com/p/digitalwritefast/
#include <LiquidCrystal_SR_LCD3.h> // https://github.com/marcmerlin/NewLiquidCrystal

#include <Servo.h> 

//Servo settings MUST BE AN INT! BETWENEN 0-180
//servoDownZ is where your pen is engaged - pressing down 
#define servoDownZ 165
//servoUpZ is where your pen is disengaged - let up.
#define servoUpZ 90

// LCD Stuff
const int PIN_LCD_STROBE         =  2;  // Out: LCD IC4094 shift-register strobe
const int PIN_LCD_DATA           =  3;  // Out: LCD IC4094 shift-register data
const int PIN_LCD_CLOCK          =  4;  // Out: LCD IC4094 shift-register clock
LiquidCrystal_SR_LCD3 lcd(PIN_LCD_DATA, PIN_LCD_CLOCK, PIN_LCD_STROBE);

Servo myservo;
byte pos;

//#define BAUD    (115200)
#define BAUD 115200

// These will be used in the near future.
#define VERSION "00073"      // 5 caracters - 00073 is a variation of 00072
#define ROLE    "hackCNC   " // 10 characters


//Pitch = mm/turn = 1.25 mm/turn
//mm/inch = 25.4
//turns/inch = 25.4/1.25 = 20.32
//steps/turn = 200
//fullsteps/inch = 200*20.32 = 4064
//microstepping, so 8 microsteps/step * 4064 = 32512 steps per in.
//not using stepmode() due to unfavorable comment about it.


#define stepsPerInchX 32512
#define stepsPerInchY 32512
//set very low because it's essentially digital, cutting down on z commands.
#define stepsPerInchZ 1

//#define minStepTime 25 //delay in MICROseconds between step pulses.
//#define minStepTime 625
#define minStepTime 57

// step pins (required)
#define stepPin0 10 //x step D10 - pin 30
#define stepPin1 8 //y step D8 - pin 28
#define stepPin2 -1 // z step

// dir pins (required)
#define dirPin0 11 //x dir D11 - pin 12
#define dirPin1 9 //y dir D9 - pin 29
#define dirPin2 -1 // z dir

// microStepping pins (optional)

#define chanXms1 -1
#define chanXms2 -1
#define chanXms3 -1
#define chanYms1 -1
#define chanYms2 -1
#define chanYms3 -1
#define chanZms1 -1
#define chanZms2 -1
#define chanZms3 -1

#define xEnablePin -1
#define yEnablePin -1
#define zEnablePin -1


#define useEstopSwitch  false
#define usePowerSwitch  false
#define useProbe        false
#define useStartSwitch  false
#define useStopSwitch   false
#define usePauseSwitch  false
#define useResumeSwitch false
#define useStepSwitch   false

// Set to true if your using real switches for MIN positions.
#define useRealMinX false
#define useRealMinY false
#define useRealMinZ false

// Set to true if your using real switches for HOME positions.
#define useRealHomeX false
#define useRealHomeY false
#define useRealHomeZ false

// Set to false if your using real switches for MAX positions.
#define useRealMaxX false
#define useRealMaxY false
#define useRealMaxZ false

// If your using REAL switches you'll need real pins (ignored if using Virtual switches).
// -1 = not used.
#define xMinPin -1
#define yMinPin -1
#define zMinPin -1

#define xHomePin -1
#define yHomePin -1
#define zHomePin -1

#define xMaxPin -1
#define yMaxPin -1
#define zMaxPin -1

//#define powerSwitchIsMomentary true // Set to true if your using a momentary switch.
//#define powerPin    -1 // Power switch. Optional
//#define powerLedPin -1 // Power indicator. Optional

//#define eStopPin         -1 // E-Stop switch. You really, REALLY should have this one.
//#define eStopLedPin      -1 // E-Stop indicator. Optional

//#define probePin  -1 // CNC Touch probe input.     Optional
//#define startPin  -1 // CNC Program start switch.  Optional
//#define stopPin   -1 // CNC Stop program switch.   Optional
//#define pausePin  -1 // CNC Pause program switch.  Optional
//#define resumePin -1 // CNC Resume program switch. Optional
//#define stepPin   -1 // CNC Program step switch.   Optional

// Spindle pin config
//#define spindleEnablePin         -1 // Optional
//#define spindleEnableInverted    false // Set to true if you need +5v to activate.
//#define spindleDirection         -1 // Optional
//#define spindleDirectionInverted false // Set to true if spindle runs in reverse.

//#define spindleTach      -1 // Must be an interrupt pin. Optional.
                            // UNO can use pin 2 or 3.
                            // Mega2560 can use 2,3,18,19,20 or 21.

//#define coolantMistPin   -1 // Controls coolant mist pump.   Optional
//#define coolantFloodPin  -1 // Controls coolant flood pump.  Optional
//#define powerSupplyPin   -1 // Controls power supply ON/OFF. Optional
//#define powerSupplyInverted true // Set to "true" for +5v = ON

// Signal inversion for real switch users. (false = ground trigger signal, true = +5vdc trigger signal.)
// Note: Inverted switches will need pull-down resistors (less than 10kOhm) to lightly ground the signal wires.
#define xMinPinInverted false
#define yMinPinInverted false
#define zMinPinInverted false

#define xHomePinInverted false
#define yHomePinInverted false
#define zHomePinInverted false

#define xMaxPinInverted false
#define yMaxPinInverted false
#define zMaxPinInverted false

#define eStopPinInverted  false
#define powerPinInverted  false
#define probePinInverted  false
#define startPinInverted  false
#define stopPinInverted   false
#define pausePinInverted  false
#define resumePinInverted false
#define stepPinInverted   false

// Where should the VIRTUAL Min switches be set to (ignored if using real switches).
// Set to whatever you specified in the StepConf wizard.
#define xMin -5.1
#define yMin -5.1
#define zMin -5.1

// Where should the VIRTUAL home switches be set to (ignored if using real switches).
// Set to whatever you specified in the StepConf wizard.
#define xHome 0
#define yHome 0
#define zHome 0

// Where should the VIRTUAL Max switches be set to (ignored if using real switches).
// Set to whatever you specified in the StepConf wizard.
#define xMax 15.1
#define yMax 15.1
#define zMax 15.1

#define giveFeedBackX false
#define giveFeedBackY false
#define giveFeedBackZ false

/*
  This indicator led will let you know how hard you pushing the Arduino.

  To test: Issue a G0 in the GUI command to send all axies to near min limits then to near max limits.
  Watch the indicator led as you do this. Adjust "Max Velocity" setting to suit.

  MOSTLY ON  = You can safely go faster.
  FREQUENT BLINK = This is a safe speed. The best choice.
  OCCASIONAL BLINK = Your a speed demon. Pushing it to the limits.
  OFF COMPLETELY = Pushing it too hard. Slow down! The Arduino can't cope, your CNC will break bits and make garbage.
  
*/
#define idleIndicator 13

// Invert direction of movement for an axis by setting to false.
boolean dirState0=true;
boolean dirState1=true;
boolean dirState2=true;

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////END OF USER SETTINGS///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

// This buffer's a single command.
char buffer[128];
int sofar;
 
float pos_x;
float pos_y;
float pos_z;

//float revs_in=0;
//float spindleRPSin=0;

boolean stepState=LOW;
unsigned long stepTimeOld=0;
unsigned long spindleTimeOld=0;
long stepper0Pos=0;
long stepper0Goto=0;
long stepper1Pos=0;
long stepper1Goto=0;
long stepper2Pos=0;
long stepper2Goto=0;

int stepModeX=-1; // don't set these here, look at stepMode() for info.
int stepModeY=-1;
int stepModeZ=-1;

boolean xMinState=false;
boolean yMinState=false;
boolean zMinState=false;

boolean xMinStateOld=false;
boolean yMinStateOld=false;
boolean zMinStateOld=false;

boolean xHomeState=false;
boolean yHomeState=false;
boolean zHomeState=false;

boolean xHomeStateOld=false;
boolean yHomeStateOld=false;
boolean zHomeStateOld=false;

boolean xMaxState=false;
boolean yMaxState=false;
boolean zMaxState=false;

boolean xMaxStateOld=false;
boolean yMaxStateOld=false;
boolean zMaxStateOld=false;


// no support for these at the moment
/*
boolean eStopStateOld=false;
boolean powerStateOld=false;
//boolean probeStateOld=false;
boolean startStateOld=false;
boolean stopStateOld=false;
boolean pauseStateOld=false;
boolean resumeStateOld=false;
boolean stepStateOld=false;
*/

// Soft Limit and Power
boolean eStopState=false;
boolean powerState=false;

// Axis homed
boolean xHomed=false;
boolean yHomed=false;
boolean zHomed=false;

int globalBusy=0;

long divisor=1000000; // input divisor. Our HAL script wont send the six decimal place floats that EMC cranks out.
                      // A simple workaround is to multply it by 1000000 before sending it over the wire.
                      // So here we have to put the decimal back to get the real numbers.
                      // Used in: processCommand()

//boolean psuState=powerSupplyInverted;
//boolean spindleState=spindleDirectionInverted;

float fbx=1;
float fby=1;
float fbz=1;


float fbxOld=0;
float fbyOld=0;
float fbzOld=0;


//void jog(float x, float y, float z, float a, float b, float c, float u, float v, float w)

void jog(float x, float y, float z)
{
  pos_x=x;
  pos_y=y;
  pos_z=z;


  // Handle our limit switches.
    // Compressed to save visual space. Otherwise it would be several pages long!
    
  if(!useRealMinX){if(pos_x > xMin){xMinState=true;}else{xMinState=false;}}else{xMinState=digitalReadFast(xMinPin);if(xMinPinInverted)xMinState=!xMinState;}
  if(!useRealMinY){if(pos_y > yMin){yMinState=true;}else{yMinState=false;}}else{yMinState=digitalReadFast(yMinPin);if(yMinPinInverted)yMinState=!yMinState;}
  if(!useRealMinZ){if(pos_z > zMin){zMinState=true;}else{zMinState=false;}}else{zMinState=digitalReadFast(zMinPin);if(zMinPinInverted)zMinState=!zMinState;}


  if(!useRealMaxX){if(pos_x > xMax){xMaxState=true;}else{xMaxState=false;}}else{xMaxState=digitalReadFast(xMaxPin);if(xMaxPinInverted)xMaxState=!xMaxState;}
  if(!useRealMaxY){if(pos_y > yMax){yMaxState=true;}else{yMaxState=false;}}else{yMaxState=digitalReadFast(yMaxPin);if(yMaxPinInverted)yMaxState=!yMaxState;}
  if(!useRealMaxZ){if(pos_z > zMax){zMaxState=true;}else{zMaxState=false;}}else{zMaxState=digitalReadFast(zMaxPin);if(zMaxPinInverted)zMaxState=!zMaxState;}

  if(!useRealHomeX){if(pos_x > xHome){xHomeState=true;}else{xHomeState=false;}}else{xHomeState=digitalReadFast(xHomePin);if(xHomePinInverted)xHomeState=!xHomeState;}
  if(!useRealHomeY){if(pos_y > yHome){yHomeState=true;}else{yHomeState=false;}}else{yHomeState=digitalReadFast(yHomePin);if(yHomePinInverted)yHomeState=!yHomeState;}
  if(!useRealHomeZ){if(pos_z > zHome){zHomeState=true;}else{zHomeState=false;}}else{zHomeState=digitalReadFast(zHomePin);if(zHomePinInverted)zHomeState=!zHomeState;}

  if(xMinState != xMinStateOld){xMinStateOld=xMinState;Serial.print("x");Serial.print(xMinState);}
  if(yMinState != yMinStateOld){yMinStateOld=yMinState;Serial.print("y");Serial.print(yMinState);}
  if(zMinState != zMinStateOld){zMinStateOld=zMinState;Serial.print("z");Serial.print(zMinState);}

  if(xHomeState != xHomeStateOld){xHomeStateOld=xHomeState;Serial.print("x");Serial.print(xHomeState+4);}
  if(yHomeState != yHomeStateOld){yHomeStateOld=yHomeState;Serial.print("y");Serial.print(yHomeState+4);}
  if(zHomeState != zHomeStateOld){zHomeStateOld=zHomeState;Serial.print("z");Serial.print(zHomeState+4);}

  if(xMaxState != xMaxStateOld){xMaxStateOld=xMaxState;Serial.print("x");Serial.print(xMaxState+1);}
  if(yMaxState != yMaxStateOld){yMaxStateOld=yMaxState;Serial.print("y");Serial.print(yMaxState+1);}
  if(zMaxState != zMaxStateOld){zMaxStateOld=zMaxState;Serial.print("z");Serial.print(zMaxState+1);}

  if(xMinState && !xMaxState)stepper0Goto=pos_x*stepsPerInchX*2;
  if(yMinState && !yMaxState)stepper1Goto=pos_y*stepsPerInchY*2;
  if(zMinState && !zMaxState)stepper2Goto=pos_z*stepsPerInchZ*2; // we need the *2 as we're driving a flip-flop routine (in stepLight function)

}

void processCommand()
{
    float xx=pos_x;
    float yy=pos_y;
    float zz=pos_z;
//  float ss=revs_in;

  char *ptr=buffer;
  while(ptr && ptr<buffer+sofar)
  {
    ptr=strchr(ptr,' ')+1;
    switch(*ptr) {
      
      // These are axis move commands
      case 'x': case 'X': xx=atof(ptr+1); xx=xx/divisor; break;
      case 'y': case 'Y': yy=atof(ptr+1); yy=yy/divisor; break;
      case 'z': case 'Z': zz=atof(ptr+1); zz=zz/divisor; move_servo(zz); break;
//      case 'z': case 'Z': zz=atof(ptr+1); break;

      // Spindle speed command. In revs per second
//      case 's': case 'S': ss=atof(ptr+1); spindleRPSin=ss; break;

    default: ptr=0; break;
    }
  }
  
  jog(xx,yy,zz);
  
  if(globalBusy<15)
  {
    // Insert LCD call here. (Updated when mostly idle.) Future project.
    lcd.setCursor (0, 3);
    lcd.print(F("X:"));
    lcd.print(pos_x);
    lcd.setCursor (10, 3);
    lcd.print(F("Y:"));
    lcd.print(pos_y);
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


void stepLight() // Set by jog() && Used by loop()
{
  unsigned long curTime=micros();
  if(curTime - stepTimeOld >= minStepTime)
  {
    stepState=!stepState;
    int busy=0;

    // so, the stepper position hasn't reached it's destination.
    // 
    if(stepper0Pos != stepper0Goto){
      busy++;
      if(stepper0Pos > stepper0Goto){
        digitalWriteFast(dirPin0,!dirState0);
        digitalWriteFast(stepPin0,stepState);
        stepper0Pos--;
      }
      else{
        digitalWriteFast(dirPin0, dirState0);
        digitalWriteFast(stepPin0,stepState);
        stepper0Pos++;
      }
    }
    if(stepper1Pos != stepper1Goto){busy++;if(stepper1Pos > stepper1Goto){digitalWriteFast(dirPin1,!dirState1);digitalWriteFast(stepPin1,stepState);stepper1Pos--;}else{digitalWriteFast(dirPin1, dirState1);digitalWriteFast(stepPin1,stepState);stepper1Pos++;}}
    // if(stepper2Pos != stepper2Goto){busy++;if(stepper2Pos > stepper2Goto){digitalWriteFast(dirPin2,!dirState2);digitalWriteFast(stepPin2,stepState);stepper2Pos--;}else{digitalWriteFast(dirPin2, dirState2);digitalWriteFast(stepPin2,stepState);stepper2Pos++;}}


    // we have a busy value, so we're still moving.
    if(busy){
      digitalWriteFast(idleIndicator,LOW);
      // increment to stay we're still busy.
      if(globalBusy<255){
        globalBusy++;
      }
    }
    else{
      // not busy, time to work on some 
      digitalWriteFast(idleIndicator,HIGH);
      if(globalBusy>0){
        globalBusy--;
      }
      
      // this transmits we're not ready to finish.
      // but isn't used upstream in the HAL
      if(giveFeedBackX){
        fbx=stepper0Pos/4/(stepsPerInchX*0.5);
        
        // We never get to this!
        if(!busy){
          if(fbx!=fbxOld){
            fbxOld=fbx;
            Serial.print("fx");
            Serial.println(fbx,6);
          }
        }
      }
      if(giveFeedBackY){fby=stepper1Pos/4/(stepsPerInchY*0.5);if(!busy){if(fby!=fbyOld){fbyOld=fby;Serial.print("fy");Serial.println(fby,6);}}}
      // if(giveFeedBackZ){fbz=stepper2Pos/4/(stepsPerInchZ*0.5);if(!busy){if(fbz!=fbzOld){fbzOld=fbz;Serial.print("fz");Serial.println(fbz,6);}}}

    }
    
    stepTimeOld=curTime;
  }
}

/*
void stepMode(int axis, int mode) // May be omitted in the future. (Undecided)
{
  // called just once during setup()
  
  // This works OPPOSITE of what you might think.
  // Mode 1 = 1/16 step.
  // Mode 2 = 1/8 step.
  // Mode 4 = 1/4 step.
  // Mode 8 = 1/2 step.
  // Mode 16 = Full step.
  // Its simular to a car's gearbox with gears from low to high as in 1,2,4,8,16

  // Originally intended for dynamic microstepping control to reduce mpu overhead and speed steppers when moving large distances.
  // Real world result: Increased overhead and slowed steppers while juggling unessessary math and pin commands.
  
  boolean ms1;
  boolean ms2;
  boolean ms3;
  int count;
  if(mode>=16){ms1=LOW;ms2=LOW;ms3=LOW;count=16;}
  if(mode>=8 && mode<=15){ms1=HIGH;ms2=LOW;ms3=LOW;count=8;}
  if(mode>=4 && mode<=7){ms1=LOW;ms2=HIGH;ms3=LOW;count=4;}
  if(mode>=2 && mode<=3){ms1=HIGH;ms2=HIGH;ms3=LOW;count=2;}
  if(mode<=1){ms1=HIGH;ms2=HIGH;ms3=HIGH;count=1;}
  if(axis == 0 || 9){if(mode!=stepModeX){digitalWriteFast(chanXms1,ms1);digitalWriteFast(chanXms2,ms2);digitalWriteFast(chanXms3,ms3);stepModeX=count;}}
  if(axis == 1 || 9){if(mode!=stepModeY){digitalWriteFast(chanYms1,ms1);digitalWriteFast(chanYms2,ms2);digitalWriteFast(chanYms3,ms3);stepModeY=count;}}
  if(axis == 2 || 9){if(mode!=stepModeZ){digitalWriteFast(chanZms1,ms1);digitalWriteFast(chanZms2,ms2);digitalWriteFast(chanZms3,ms3);stepModeZ=count;}}
  if(axis == 3 || 9){if(mode!=stepModeA){digitalWriteFast(chanAms1,ms1);digitalWriteFast(chanAms2,ms2);digitalWriteFast(chanAms3,ms3);stepModeA=count;}}
  if(axis == 4 || 9){if(mode!=stepModeB){digitalWriteFast(chanBms1,ms1);digitalWriteFast(chanBms2,ms2);digitalWriteFast(chanBms3,ms3);stepModeB=count;}}
  if(axis == 5 || 9){if(mode!=stepModeC){digitalWriteFast(chanCms1,ms1);digitalWriteFast(chanCms2,ms2);digitalWriteFast(chanCms3,ms3);stepModeC=count;}}
  if(axis == 6 || 9){if(mode!=stepModeU){digitalWriteFast(chanUms1,ms1);digitalWriteFast(chanUms2,ms2);digitalWriteFast(chanUms3,ms3);stepModeU=count;}}
  if(axis == 7 || 9){if(mode!=stepModeV){digitalWriteFast(chanVms1,ms1);digitalWriteFast(chanVms2,ms2);digitalWriteFast(chanVms3,ms3);stepModeV=count;}}
  if(axis == 8 || 9){if(mode!=stepModeW){digitalWriteFast(chanWms1,ms1);digitalWriteFast(chanWms2,ms2);digitalWriteFast(chanWms3,ms3);stepModeW=count;}}

}
*/
/*
int determinInterrupt(int val)
{
  if(val<0) return -1;
  if(val==2) return 0;
  if(val==3) return 1;
  if(val==18) return 5;
  if(val==19) return 4;
  if(val==20) return 3;
  if(val==21) return 2;
}
*/

/*
volatile unsigned long spindleRevs=0;
float spindleRPS=0;
float spindleRPM=0;

void countSpindleRevs()
{
  spindleRevs++;
}

float updateSpindleRevs()
{
  unsigned long spindleTime=millis();
  if(spindleTime - spindleTimeOld >= 100)
  {
    spindleRPS=spindleRevs*10.0;
    spindleRPM=spindleRPS*60.0;
    spindleRevs=0;
  }
}

boolean spindleEnabled=false;
boolean spindleEnableState=spindleEnableInverted;

boolean spindleAtSpeed()
{
  if(spindleTach>0)
  {
    if(spindleRPSin<spindleRPS)
    {
      if(spindleRPSin*1.05<spindleRPS || !spindleEnabled)
      { // Slow down. 
        if(spindleEnablePin>0){digitalWriteFast(spindleEnablePin,!spindleEnableState);}
      }else{
        if(spindleEnabled)
        { // Speed up. 
          if(spindleEnablePin>0){digitalWriteFast(spindleEnablePin,spindleEnableState);}
        }
      }
      return true;
    }else{
      return false;
    }
  }else{
    return spindleEnabled; // No tach? We'll fake it.
  }
}
*/

void setup()
{
  //Servo pin 
  myservo.attach(12); //Pin 26 - PD6 or D12
  //Go home.
  move_servo(1);
  
  lcd.begin(20, 4);               // initialize the lcd 
  lcd.home ();                   // go home
  lcd.setCursor (0, 0);
  lcd.print(F("hackCNC             "));
 
  // If using a spindle tachometer, setup an interrupt for it.
//  if(spindleTach>0){int result=determinInterrupt(spindleTach);attachInterrupt(result,countSpindleRevs,FALLING);}
  
  // Setup Min limit switches.
  if(useRealMinX){pinMode(xMinPin,INPUT);if(!xMinPinInverted)digitalWriteFast(xMinPin,HIGH);}
  if(useRealMinY){pinMode(yMinPin,INPUT);if(!yMinPinInverted)digitalWriteFast(yMinPin,HIGH);}
  if(useRealMinZ){pinMode(zMinPin,INPUT);if(!zMinPinInverted)digitalWriteFast(zMinPin,HIGH);}

  // Setup Max limit switches.
  if(useRealMaxX){pinMode(xMaxPin,INPUT);if(!xMaxPinInverted)digitalWriteFast(xMaxPin,HIGH);}
  if(useRealMaxY){pinMode(yMaxPin,INPUT);if(!yMaxPinInverted)digitalWriteFast(yMaxPin,HIGH);}
  if(useRealMaxZ){pinMode(zMaxPin,INPUT);if(!zMaxPinInverted)digitalWriteFast(zMaxPin,HIGH);}

  // Setup Homing switches.
  if(useRealHomeX){pinMode(xHomePin,INPUT);if(!xHomePinInverted)digitalWriteFast(xHomePin,HIGH);}
  if(useRealHomeY){pinMode(yHomePin,INPUT);if(!yHomePinInverted)digitalWriteFast(yHomePin,HIGH);}
  if(useRealHomeZ){pinMode(zHomePin,INPUT);if(!zHomePinInverted)digitalWriteFast(zHomePin,HIGH);}


  // Enable stepper drivers.
  //pinMode(xEnablePin,OUTPUT);digitalWrite(xEnablePin,LOW);
  //pinMode(yEnablePin,OUTPUT);digitalWrite(yEnablePin,LOW);
  //pinMode(zEnablePin,OUTPUT);digitalWrite(zEnablePin,LOW);

  // Setup step pins.
  pinMode(stepPin0,OUTPUT);
  pinMode(stepPin1,OUTPUT);
  // pinMode(stepPin2,OUTPUT);
  
  // Setup dir pins.
  pinMode(dirPin0,OUTPUT);
  pinMode(dirPin1,OUTPUT);
//  pinMode(dirPin2,OUTPUT);


  // Setup microStepping pins.

  //pinMode(chanXms1,OUTPUT);pinMode(chanXms2,OUTPUT);pinMode(chanXms3,OUTPUT);
  //pinMode(chanYms1,OUTPUT);pinMode(chanYms2,OUTPUT);pinMode(chanYms3,OUTPUT);
  //pinMode(chanZms1,OUTPUT);pinMode(chanZms2,OUTPUT);pinMode(chanZms3,OUTPUT);
  
  // Setup eStop, power, start, stop, pause, resume, program step, spindle, coolant, LED and probe pins.
  //if(useEstopSwitch){pinMode(eStopPin,INPUT);if(!eStopPinInverted){digitalWriteFast(eStopPin,HIGH);}}
  //if(usePowerSwitch){pinMode(powerPin,INPUT);if(!powerPinInverted){digitalWriteFast(powerPin,HIGH);}}
  //if(useProbe){pinMode(probePin,INPUT);if(!probePinInverted){digitalWriteFast(probePin,HIGH);}}
  //if(useStartSwitch){pinMode(startPin,INPUT);if(!startPinInverted){digitalWriteFast(startPin,HIGH);}}
  //if(useStopSwitch){pinMode(stopPin,INPUT);if(!stopPinInverted){digitalWriteFast(stopPin,HIGH);}}
  //if(usePauseSwitch){pinMode(pausePin,INPUT);if(!pausePinInverted){digitalWriteFast(pausePin,HIGH);}}
  //if(useResumeSwitch){pinMode(resumePin,INPUT);if(!resumePinInverted){digitalWriteFast(resumePin,HIGH);}}
  //if(useStepSwitch){pinMode(stepPin,INPUT);if(!stepPinInverted){digitalWriteFast(stepPin,HIGH);}}
  //if(powerLedPin > 0){pinMode(powerLedPin,OUTPUT);digitalWriteFast(powerLedPin,HIGH);}
  //if(eStopLedPin>0){pinMode(eStopLedPin,OUTPUT);digitalWriteFast(eStopLedPin,LOW);}

  //if(spindleEnablePin>0){pinMode(spindleEnablePin,OUTPUT);digitalWriteFast(spindleEnablePin,HIGH);}
  //if(spindleDirection>0){pinMode(spindleDirection,OUTPUT);digitalWriteFast(spindleDirection,LOW);}
  //if(spindleTach>0){pinMode(spindleTach,INPUT);digitalWriteFast(spindleTach,HIGH);}
  //if(coolantMistPin>0){pinMode(coolantMistPin,OUTPUT);digitalWriteFast(coolantMistPin,LOW);}
  //if(coolantFloodPin>0){pinMode(coolantFloodPin,OUTPUT);digitalWriteFast(coolantFloodPin,LOW);}

  //if(powerSupplyPin>0){pinMode(powerSupplyPin,OUTPUT);digitalWriteFast(powerSupplyPin,psuState);}

  // Setup idle indicator led.
  pinMode(idleIndicator,OUTPUT);

  // Actually SET our microStepping mode. (If you must change this, re-adjust your stepsPerInch settings.)
//  stepMode(9,16);
  
  lcd.setCursor (0, 1);
  lcd.print(F("Startup Complete.   "));
  lcd.setCursor (0, 2);
  lcd.print(F("Waiting For LinuxCNC"));
  lcd.setCursor (0, 3);
  lcd.print(F("                    ")); // 20 spaces! must be a better way
  
  // Setup serial link.
  Serial.begin(BAUD);
  

  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo/32u4
  }
  
  // nothing will happen until we have a valid serial connection.
  lcd.setCursor (0, 1);
  lcd.print(F("LinuxCNC Connected. "));
  lcd.setCursor (0, 2);
  lcd.print(F("                    "));
  lcd.setCursor (15, 0);
  lcd.print(F("_____"));

  // Initialize serial command buffer.
  sofar=0;
}

boolean spindleAtSpeedStateOld;

void loop()
{
  // These are all for physical switches, which we don't have
  //if(useEstopSwitch==true){boolean eStopState=digitalReadFast(eStopPin);if(eStopPinInverted){eStopState=!eStopState;}if(eStopState != eStopStateOld || eStopStateOld){eStopStateOld=eStopState;Serial.print("e");Serial.println(eStopState);delay(500);}}
  //if(usePowerSwitch==true){boolean powerState=digitalReadFast(powerPin);if(powerPinInverted){powerState=!powerState;}if(powerState != powerStateOld){powerStateOld=powerState;if(powerSwitchIsMomentary){Serial.println("pt");}else{Serial.print("p");Serial.println(powerState);}}}
  //if(useProbe==true){boolean probeState=digitalReadFast(probePin);if(probePinInverted){probeState=!probeState;}if(probeState != probeStateOld){probeStateOld=probeState;Serial.print("P");Serial.println(probeState);}}
  //if(useStartSwitch==true){boolean startState=digitalReadFast(startPin);if(startPinInverted){startState=!startState;}if(startState != startStateOld){startStateOld=startState;Serial.print("h");Serial.println(startState);}}
  //if(useStopSwitch==true){boolean stopState=digitalReadFast(stopPin);if(stopPinInverted){stopState=!stopState;}if(stopState != stopStateOld){stopStateOld=stopState;Serial.print("h");Serial.println(stopState+2);}}
  //if(usePauseSwitch==true){boolean pauseState=digitalReadFast(pausePin);if(pausePinInverted){pauseState=!pauseState;}if(pauseState != pauseStateOld){pauseStateOld=pauseState;Serial.print("h");Serial.println(pauseState+4);}}
  //if(useResumeSwitch==true){boolean resumeState=digitalReadFast(resumePin);if(resumePinInverted){resumeState=!resumeState;}if(resumeState != resumeStateOld){resumeStateOld=resumeState;Serial.print("h");Serial.println(resumeState+6);}}
  //if(useStepSwitch==true){boolean stepState=digitalReadFast(stepPin);if(stepPinInverted){stepState=!stepState;}if(stepState != stepStateOld){stepStateOld=stepState;Serial.print("h");Serial.println(stepState+8);}}


  // listen for serial commands
  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read();
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions
  }
 
  // Received a "+" turn something on.
  if(sofar>0 && buffer[sofar-3]=='+') {

    //if(sofar>0 && buffer[sofar-2]=='P') { /* Power LED & PSU   ON */ if(powerLedPin>0){digitalWriteFast(powerLedPin,HIGH);}if(powerSupplyPin>0){digitalWriteFast(powerSupplyPin,psuState);}}
    if(sofar>0 && buffer[sofar-2]=='E') { eStopState=true; lcd.setCursor (0, 1); lcd.print(F("eStop Open.         "));lcd.setCursor (15, 0); lcd.print(F("E"));};
    if(sofar>0 && buffer[sofar-2]=='P') { powerState=true; lcd.setCursor (0, 1); lcd.print(F("Power Initialised.  "));lcd.setCursor (16, 0); lcd.print(F("P"));};
    //if(sofar>0 && buffer[sofar-2]=='E') { /* E-Stop Indicator  ON */ if(eStopLedPin>0){digitalWriteFast(eStopLedPin,HIGH);}}
    
//    if(sofar>0 && buffer[sofar-2]=='S') { /* Spindle power     ON */ spindleEnabled=true;}
//    if(sofar>0 && buffer[sofar-2]=='D') { /* Spindle direction CW */ if(spindleDirection>0){digitalWriteFast(spindleDirection,spindleState);}}
//    if(sofar>0 && buffer[sofar-2]=='M') { /* Coolant Mist      ON */ if(coolantMistPin>0){digitalWriteFast(coolantMistPin,HIGH);}}
//    if(sofar>0 && buffer[sofar-2]=='F') { /* Coolant Flood     ON */ if(coolantFloodPin>0){digitalWriteFast(coolantFloodPin,HIGH);}}

    // Running code state
    // These are mutually exclusive so don't need a - set.
    // Should be a better way
    if(sofar>0 && buffer[sofar-2]=='r') { lcd.setCursor (0, 1); lcd.print(F("Running gCode       "));lcd.setCursor (0, 2);lcd.print(F("Touch to PAUSE      "));lcd.setCursor (12, 0); lcd.print(F("R"));};
    if(sofar>0 && buffer[sofar-2]=='s') { lcd.setCursor (0, 1); lcd.print(F("Run Stopped!        "));lcd.setCursor (0, 2);lcd.print(F("                    "));lcd.setCursor (12, 0); lcd.print(F("S"));};
    if(sofar>0 && buffer[sofar-2]=='p') { lcd.setCursor (0, 1); lcd.print(F("Run Paused!         "));lcd.setCursor (0, 2);lcd.print(F("Touch to RESUME     "));lcd.setCursor (12, 0); lcd.print(F("P"));};

    // homing
    if(sofar>0 && buffer[sofar-2]=='0') { xHomed=true; lcd.setCursor (0, 1); lcd.print(F("X axis Homed        "));lcd.setCursor (17, 0); lcd.print(F("X"));};
    if(sofar>0 && buffer[sofar-2]=='1') { yHomed=true; lcd.setCursor (0, 1); lcd.print(F("Y axis Homed        "));lcd.setCursor (18, 0); lcd.print(F("Y"));};
    if(sofar>0 && buffer[sofar-2]=='2') { zHomed=true; lcd.setCursor (0, 1); lcd.print(F("Z axis Homed        "));lcd.setCursor (19, 0); lcd.print(F("Z"));};

      
      lcd.setCursor (0, 3);
      lcd.print(F("                    "));
      
      // reset the buffer
      sofar=0;  
  }

  // Received a "-" turn something off.
  // we have no physical pins, just software
  // so this is mostly not needed.
  if(sofar>0 && buffer[sofar-3]=='-') {
    //if(sofar>0 && buffer[sofar-2]=='P') { /* Power LED & PSU   OFF */ if(powerLedPin>0){ digitalWriteFast(powerLedPin,LOW);}if(powerSupplyPin>0){digitalWriteFast(powerSupplyPin,!psuState);}}
    if(sofar>0 && buffer[sofar-2]=='E') { eStopState=false; lcd.setCursor (0, 1); lcd.print(F("eStop ACTIVE! Halted"));lcd.setCursor (15, 0); lcd.print(F("_"));};
    if(sofar>0 && buffer[sofar-2]=='P') { powerState=false; lcd.setCursor (0, 1); lcd.print(F("Power Down.         "));lcd.setCursor (16, 0); lcd.print(F("_"));};
    //if(sofar>0 && buffer[sofar-2]=='E') { /* E-Stop Indicator  OFF */ if(eStopLedPin>0){digitalWriteFast(eStopLedPin,LOW);}}
    
//    if(sofar>0 && buffer[sofar-2]=='S') { /* Spindle power     OFF */ spindleEnabled=false;}
//    if(sofar>0 && buffer[sofar-2]=='D') { /* Spindle direction CCW */ if(spindleDirection>0){digitalWriteFast(spindleDirection,!spindleState);}}
//    if(sofar>0 && buffer[sofar-2]=='M') { /* Coolant Mist      OFF */ if(coolantMistPin>0){digitalWriteFast(coolantMistPin,LOW);}}
//    if(sofar>0 && buffer[sofar-2]=='F') { /* Coolant Flood     OFF */ if(coolantFloodPin>0){digitalWriteFast(coolantFloodPin,LOW);}}

    if(sofar>0 && buffer[sofar-2]=='0') { xHomed=true; lcd.setCursor (0, 1); lcd.print(F("X axis Unhomed      "));lcd.setCursor (17, 0); lcd.print(F("_"));};
    if(sofar>0 && buffer[sofar-2]=='1') { yHomed=true; lcd.setCursor (0, 1); lcd.print(F("Y axis Unhomed      "));lcd.setCursor (18, 0); lcd.print(F("_"));};
    if(sofar>0 && buffer[sofar-2]=='2') { zHomed=true; lcd.setCursor (0, 1); lcd.print(F("Z axis Unhomed      "));lcd.setCursor (19, 0); lcd.print(F("_"));};

    // reset the buffer.
    sofar=0;
  }
 
  // Received a "?" about something give an answer.
  if(sofar>0 && buffer[sofar-3]=='?') {
    if(sofar>0 && buffer[sofar-2]=='V') { 
      /* Report version */
      Serial.println(VERSION);
      lcd.setCursor (0, 3);
      lcd.print(F(VERSION));
      lcd.print(F("               ")); // match the Version length.  Cheating
    }
    if(sofar>0 && buffer[sofar-2]=='R') {
     /* Report role    */
     Serial.println(ROLE);
      lcd.setCursor (0, 3);
      lcd.print(F(ROLE));
      lcd.print(F("          ")); // match the Role length.  Cheating
    }
    
    // reset the buffer.
    sofar=0;
  }
  
  // if we hit a semi-colon, assume end of instruction.
  // Do NOT EXECUTE instructions if E-STOP or Power is off!
  // eStopState is not implemented yet.   awesome :(
  if(powerState && sofar>0 && buffer[sofar-1]==';') {
 
    buffer[sofar]=0;
    
    // output the command
    //Serial.println(buffer);
    // this stuff seems to really slow down the processing.
    //lcd.setCursor (0, 2); // could have sworn this should be 0,1
    //lcd.print(F("                                                            ")); // three lines of spaces
    //lcd.setCursor (0, 2);
    //lcd.print(buffer);
 
    // do something with the command
    processCommand();
 
    // reset the buffer
    sofar=0;
  }
/*
  updateSpindleRevs();
  if(!globalBusy){boolean spindleAtSpeedState=spindleAtSpeed();if(spindleAtSpeedState != spindleAtSpeedStateOld){spindleAtSpeedStateOld=spindleAtSpeedState;Serial.print("S");Serial.println(spindleAtSpeedState);}}
*/
  stepLight(); // call every loop cycle to update stepper motion.
}
