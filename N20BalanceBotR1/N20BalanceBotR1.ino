// I2C device class (I2Cdev) Arduino library for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
/*
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
This sketch provides an interface for a Arduino Lenardo to 
drive a L298 Dual Motor Control Board and a 16x2 LCD/Keypad SAINSMART Shield.
The Code is also setup to read dmp data from a MPU6050 digital gyro.
The prime purpose of this code is to control a two wheeled balance-bot type car
The PID constants have been optimized for 'N20' gear motors, powered by 8 AA NiMh rechargable batteries. 
The LCD Display/Keypad provides a limited user interface. [Select Aout Balance Point ON/OFF & Twirl or Stationary Modes]
This code also includes for the use of an IR Distance Detector
When an object, such as a wall is within the detection range (~8"), the code
shifts the pitch effective set-point backwards, until the object is beyond
the range of detection. This in-turn fools the PID into the balance point is behind
the true balance point and as a result the car move backwards trying to maintain its
equilibrium.
*******************************************************************************************
* This code was written to work with a car whose components are arranged as described in: * 
* BalanceBotWiringDiagram.pdf                                                             * 
* and a L298N Break-Out Board configured as shown in:                                     *
* L298NdualHbridge.JPG                                                                    *
*******************************************************************************************
*/

//#include <string.h>
#include <LiquidCrystal.h>
//#include <LCDKeypad.h>
#include "LCDKeypadR1.h"
//#include <LCD4Bit_mod.h> 

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


//#define OutPut_READABLE_YAWPITCHROLL
#define LED_PIN 12 

#define FORWARD false //true
#define REVERSE true //false
#define RIGHTWHEEL 0
#define LEFTWHEEL 1  //Right Wheel Handle
#define IRSig 1

byte error;

byte address = 0x68;
bool blinkState = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



//create object to control an LCD.  
// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
LCDKeypad Keypad;

// Pin 13 has an LED connected on most Arduino boards.
// When the pin is HIGH value, the LED is on, when the pin is LOW, it's off.
// give it a name:
//int led = 13;
int In1 = A5;//1;      // L298 Left Motor Dir Control In1 connected to Arduino digital pin 1
int In2 = A2;//3;      // L298 Left Motor Dir Control In2 connected to Arduino Analog pin 2
int In3 = A4;//0;      // L298 Right Motor Dir Control In3 connected to Arduino digital pin 0
int In4 = A3;//3;      // L298 Right Motor Dir Control In4 connected to Arduino Analog pin 3

int EnA = 11; //11;     // L298 Left Motor Control EnA connected to Digital PWM pin 11
int EnB = 13; //13;     // L298 Right Motor Control EnB connected to Digital PWM pin 13; Note also controls On-Board LED

int SpeedL = 0;         // PWM Value for Motor A (Left Wheel)
int SpeedR = 0;         // PWM Value for Motor B (Right Wheel)
int Speed;
int LastSpeed =0;
int Steer = 0;
double RunningErr = 0.0;
double LastRunningErr = 0.0;
double RunningSpd = 0.0;// Used as part of auto Speed based Balance Correction
double LastRunningSpd = 0.0;// Used as part of auto Speed based Balance Correction
double BPtDif = 0.0;// Used as part of auto Speed based Balance Correction
int loopCnt = 0;
int loopCnt2 = 1;
int loopCnt1 = 0; // auto yaw zero
int loopCnt3 = 0; // auto pitch zero
int loopCnt4 = 0; // auto Speed based Balance Correction
boolean LoopComplete = true;
boolean Gud2Go = false;
boolean Fgr8Mode = false;
boolean LCD_On = false;
boolean SkipUpdate = false;
boolean Rpt_IR = true;
boolean AutoBalPt = true;

unsigned long LastShw = millis();


int buttonPressed;

// IR Distance Detector GP2D15 related Variables
int val = 0; 
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// PID variables
/*working variables*/
unsigned long lastTime;
double InputP, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
double Kp = 0.32;//0.36; //0.3; 0.0;//
double Ki = 5.4; //3.6;
double Kd = 0.052; //0.042;
double OS = 18;
double BrkPt = 22;
double X = 0.0;
//double gain = 1.0;
double PIDOut = 0.0;
double DeadBnd = 45.0; //62;; DeadBnd = OffSet Lower Limit; used in AplyOffSet routine 
                         // to eliminate Motor deadband throttle values
unsigned long lastTimeS;
double InputY, SetpointS;
double ITermS, lastInputS;
double kpS, kiS, kdS;
double KpS = 1.0;//0.2;
double KiS = 0.3;//0.02;
double KdS = 0.022;//0.022;
double PIDOutS = 0.0;
double DeltaStPt = 10.0; //fixed steering incremental change 
int SampleTime = 25; //1 sec
int IdealSmplTm = SampleTime;
unsigned long SampleTimeP, SampleTimeY;
double Min = -254.0;
double Max = 254.0;
double outMin, outMax;
double PitchBalancePt = 2.29;//2.22;//2.45 A larger positive number makes the car lean further forward
double PtchBalPtRef = PitchBalancePt;
double BackAngleRef = 0.1;
double BackAngle = 0.0;
boolean BackAway = false;
double YawBalancePt = 0.0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// LCD message
char msgs[9][17] = {"FORWARD Spd", 
                    "IR Dect", 
                    "BAL PT     ", 
                    "TURN RIGHT   ", 
                    "TWIRL 360    ",
                    "STRAIGHT LINE",
                    "PRESS SELECT",
                    "AUTO-BAL PT ON ",
                    "AUTO-BAL PT OFF"};

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++




// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  //setup PWM frequency to ~60Hz for OutPut pins 13 & 11
  TCCR4B = TCCR4B & 0b11111000 | 0x06; //Set Digital pin 13 Right Wheel PWM period @ ~2ms
  digitalWrite(A1, HIGH);  // set pullup on analog pin 1 
  // initialize the Arduino digital pins as an OutPuts.
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  lcd.begin(16, 2); 
  ConfigRunMode(); 
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #endif
      LCD_On = true;
      lcd.clear();
      lcd.print("Init MPU6050..");
      mpu.initialize();
      Serial.begin(9600);
    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    //Sensor readings with offsets:	4	4	16380	1	0	0
    //Your offsets:	-1200	-2080	1288	35	-28	-39
    //Data is printed as: acelX acelY acelZ giroX giroY giroZ
    //Check that your sensor readings are close to 0 0 16384 0 0 0

    mpu.setXGyroOffset(35);
    mpu.setYGyroOffset(-28);
    mpu.setZGyroOffset(-39);

    mpu.setXAccelOffset(-1200);
    mpu.setYAccelOffset(-2080);
    mpu.setZAccelOffset(1288);

    if (devStatus == 0) 
     {
          lcd.clear();
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for OutPut
    pinMode(LED_PIN, OUTPUT);
    //initialize PID
    X =  16.0/BrkPt+(OS/(Kp*16.0));
    Setpoint = 0.00; // Robot heading [0 = straight ahead]
    SetTunings(Kp, Ki, Kd);
    SetTuningsS(KpS, KiS, KdS);
    SampleTimeY = (unsigned long)SampleTime;
    SampleTimeP = (unsigned long)SampleTime;
    PIDOut = 0.0;
    PIDOutS = 0.0;
    PitchBalancePt = (M_PI/180)*PitchBalancePt;
    PtchBalPtRef = PitchBalancePt;
    BackAngleRef = (M_PI/180)*BackAngleRef;
    BackAngle = BackAngleRef;
    
    
}
// End Setup


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
     
    // if programming failed, don't try to do anything
    // check for blocking object
    
    val = analogRead(IRSig); // take a distance reading
    //val = 0; //Disable IR detector 
    if (val > 500)
    {
     if(!BackAway)
     {
       BackAway = !BackAway;
     }
    }
    else 
    {
     if(BackAway)
     {
       RunningSpd = 0.0;
       ITerm = 0.0;
       RunningErr = -150.05;
       BackAway = !BackAway;
     }
    }
    
    do // test to see if the MPU answers back [with its address]
    {
     Wire.beginTransmission(address);
     error = Wire.endTransmission();

     if (error != 0)
     {
      //Serial.print("I2C Addr Pole Returned Error ");
      //Serial.println(error,HEX);
      //ShowMessage("I2C Err" ,(int) error, 1, 0);
     }
    }
    while  (error != 0);
    fifoCount = 0;
    while (fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }
    // get INT_STATUS byte
    mpuIntStatus = mpu.getIntStatus();
    //digitalWrite(LED_PIN, true);
    fifoCount = mpu.getFIFOCount();
    //digitalWrite(LED_PIN, false);
    // check for overflow (this should never happen unless our code is too inefficient)
    // sometimes quits before getting here
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) { //  || fifoCount > 2*packetSize
        // reset so we can continue cleanly
        //lcd.clear();
        //lcd.printIn("FIFO overflow!");
        //ShowMessage("FIFO!", fifoCount, 1, 0);
        mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) { // never gets here
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
         fifoCount = mpu.getFIFOCount();
        }
        //Serial.print(fifoCount);
        //Serial.print("\t");
        //Serial.println(packetSize);
        
        
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        //digitalWrite(LED_PIN, false);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        if (fifoCount ==0)
        {
            
              // display Euler angles in degrees
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetGravity(&gravity, &q);
              mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
              InputP = (ypr[2])-PitchBalancePt+BPtDif; // Remember here Pitch reading is in Radians, not Degrees
              // if we're running in "normal" mode, build an average reading of the car's pitch speed
              //using the next 30 reported values
             if(loopCnt4 >= 30) //Based on a sample period of 25ms, a loopCnt4 of 40 is ~1 second; note - loopCnt4 gets incremented in the Pitch PID 
              {
                RunningSpd = RunningSpd/loopCnt4;//; note - RunningSpd gets accumulated in the Pitch PID
                double oldBP = PitchBalancePt;
                InputP = InputP+PitchBalancePt-BPtDif; //remove old Balance Pt offset
                //Be aware, positive speed numbers actually causes the car to move backwards.
                // While a larger Positive Balance Point number will make the car lean further forward
                 
                 //calc the current proportional correction component of the Balance point 
                double SpdDif = RunningSpd-LastRunningSpd;
                RunningErr += RunningSpd; 
                if(AutoBalPt)
                 {
                  double BPtCor = 0.0;
                  if (Fgr8Mode) //if (abs(SpdDif)>25) //
                  {
                    BPtDif = -0.00019*SpdDif;
                    BPtCor = 1.9*(1.2*RunningSpd+0.25*RunningErr)/20000;
                  
                  }
                  else
                  {
                    BPtDif = -0.00004*SpdDif;// last used
                    BPtCor = (1.2*RunningSpd+0.25*RunningErr)/40000;;// last used
                  }
                 //Now calculate the new diferential component in the Balance point correction factor
                  LastRunningSpd = RunningSpd;
                // if true, user selected, run in Autobal Pt mode
                  PitchBalancePt += (BPtCor- (0.5*BPtDif));
                  PitchBalancePt = (PitchBalancePt+oldBP)/2.0;
                   //lcd.cursorTo(1, 0);
                   //lcd.printIn(msgs[2]);
                 }      
                 if(!SkipUpdate)
                   {
                     ShowMessage("SPD" ,(int) (RunningSpd), 0, 0); //(Line 0, Pos =0)
                     ShowMessage("Err" ,(int) (RunningErr), 0, 8); //(Line 0, Pos =0)
                     //ShowMessage("TPD" ,(int) (TotPtchDif*10), 0, 8);
                     //ShowMessage("ITm" ,(int) ITerm, 0, 8);
                     //ShowMessage("Her" ,(int) (100*HomErr * 180/M_PI), 0, 8);
                     //ShowMessage("Dif" ,(int) SpdDif, 0, 8);
                     //ShowMessage("Dif" ,(int) (RunningErr-LastRunningErr), 0, 8);
                     ShowMessage("Bpt" ,(int) (100*PitchBalancePt * 180/M_PI), 1, 0); // Ist charater pos; 2nd row
                     SkipUpdate = true;
                   }
                InputP = InputP-PitchBalancePt+BPtDif; // now add the new improved Bal Pt back in
                RunningSpd = 0.0;
                loopCnt4 = 0;
                
              }

              InputY= (ypr[0] * 180/M_PI); // convert Yaw Radian value to Degrees
              //Serial.println(ypr[2] * 180/M_PI);
              if (abs((int) InputP* 180/M_PI) < 1)// If BalanceBot is within 1 degree of vertical, then recallibrate Yaw offset
               { 
                 loopCnt1 += 1;
                 if (loopCnt1 == 900)// its vertical long enough re-zero the yaw heading 
                 {
                   loopCnt1 = 0;
                   if(!Gud2Go)
                    {
                     // lock in initial [current] heading 
                     YawBalancePt = InputY; 
                     //Reset any residual integral PID values 
                     ITerm = 0.0;
                     ITermS = 0.0;
                     BPtDif = 0.0;
                     LastRunningSpd = 0.0;
                    } 
                   Gud2Go = true;
                 }
               }
              else
               {
                 loopCnt1 = 0; 
               }
               unsigned long CurTm = millis();
               int DeltaT = (CurTm - LastShw);
               if(DeltaT>=250)
               {
               loopCnt2 -=1;
               if (loopCnt2==0) // if true; One second has elapsed since the last diaplay update
                 {
                   loopCnt2 +=4;
                   if (Fgr8Mode && (abs((int)RunningErr)<30 || !AutoBalPt)) // <20
                    {
                     YawBalancePt = YawBalancePt - DeltaStPt;
                     if(YawBalancePt <= -180.0 || YawBalancePt > +180.0)
                      {
                       DeltaStPt = - DeltaStPt; 
                       YawBalancePt = YawBalancePt - DeltaStPt;
                      }
                    }
                   ShowMessage("Yaw" ,(int) (InputY), 1, 8); 
                   if(!SkipUpdate)
                    { 
                    ShowMessage("P" ,(int) (InputP* 180/M_PI), 1, 0);
                    }
                    
                   else SkipUpdate = false;
                   // now, if Speed Signal is small, then its probably safe to take the time to Update the Yaw Display reading 
                   Rpt_IR = true; //reset IR Flag to enable new report
                 }
                LastShw = CurTm;
               }
              Speed = CalcPtchPID(InputP);//ComputePID(InputP-HomErr);// Calculate new PID Balancing value
              // the Yaw Output ranges from -180 deg to +180 deg.
              // Test for yaw corrections trying to negotiating this +/- crossing point, &
              // as needed, temporarily extend  the yaw range to bridge the gap
              if (InputY-YawBalancePt < -180.0) InputY = InputY + 360.0;
              if (InputY-YawBalancePt > +180.0) InputY = InputY - 360.0;        
              Steer = ComputePIDS(InputY-YawBalancePt);
              //ShowMessage("STR" ,(int) (Steer), 0, 8);
              
//              Serial.print("Pitch\t");
//              Serial.print(InputY* 180/M_PI); 
              SpeedL= AplyOffSet(Speed - Steer, SpeedL);
              SpeedR= AplyOffSet(Speed + Steer, SpeedR);
              //ShowMessage("Lft" ,(int) (SpeedL), 0, 8);
//              Serial.print("\tSpeedL\t");
//              Serial.println(SpeedL);
              if (Gud2Go)
               {
                SetMotorSpeed(RIGHTWHEEL, SpeedR);
                SetMotorSpeed(LEFTWHEEL, SpeedL);
               }
  
              SampleTime =IdealSmplTm;
              SampleTimeY = (unsigned long)SampleTime;
              SampleTimeP = (unsigned long)SampleTime; 
              SetTunings(Kp, Ki, Kd);
              SetTuningsS(KpS, KiS, KdS);
          // blink LED to indicate activity
          blinkState = !blinkState;
          //digitalWrite(LED_PIN, blinkState);
        }
        else
        {
          mpu.resetFIFO();
        }
    }
 
}

//###########################################################################
void ConfigRunMode()
{
  lcd.clear();
  lcd.print("SELECT RUN MODE");
  // Wait 4 seconds before starting Main Loop
  bool ShwBpMode = true; 
  do
    {
     if (ShwBpMode)
     {
       ShwBpMode = !ShwBpMode;
       lcd.setCursor(0,1);
        if (AutoBalPt)
         {
           lcd.print(msgs[7]);
         }
        else
         {
           lcd.print(msgs[8]);
         }
         delay(300);
     }
      buttonPressed=waitButton();
      if(buttonPressed==KEYPAD_SELECT)
      {
        AutoBalPt =!AutoBalPt;
        ShwBpMode = true;
      }  
    }
   while(!(buttonPressed==KEYPAD_UP || buttonPressed==KEYPAD_DOWN || buttonPressed==KEYPAD_LEFT || buttonPressed==KEYPAD_RIGHT)); 
  if (buttonPressed==KEYPAD_UP || buttonPressed==KEYPAD_DOWN)
   {
     Fgr8Mode = false;
   }
  else
   {
     Fgr8Mode = true;
   }
  lcd.clear();
  lcd.setCursor(0,1);
  if (Fgr8Mode)
  {
   lcd.print(msgs[4]);
  }
  else
  {
   lcd.print(msgs[5]);
  } 
  char buf [16];
  int downCnt = 4;
  do
  {
  sprintf (buf, "ROBY STARTS IN %i",downCnt);
  lcd.setCursor(0,0); 
  lcd.print(buf); 
  delay(1000);
  downCnt = downCnt-1;
  }
  while (downCnt>0);
}
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
double SqrVal(double x)
{
  double sign = 1.0; 
  if(x<1) sign = -1.0;
  if (x!=0) x = x/20.0;
  x = sign*x*x;
  return x;
}
// &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//###########################################################################
// Begin PID computations
int CalcPtchPID(float Input)
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTimeP)
   {
      SetSampleTime(timeChange);
      /*Compute all the working error variables*/
      if (BackAway)
       {
        if (Rpt_IR)
       { 
        lcd.setCursor(0,0); //(pos,line)
        lcd.print(msgs[1]);
        Rpt_IR = !Rpt_IR;
       } 
        Input = Input+ BackAngle;
        BackAngle += 0.1*BackAngleRef;
        if (BackAngle>0.08) BackAngle = 0.04; //limit reversing anlge to a Max of 2.5deg (note 0.08 Radians = ~5 deg
       }
      else BackAngle = BackAngleRef;
      Input = 1530.0*sin(Input);// 255.0*6*sin(Input);
      double error = Setpoint - Input;
      double dInput = (Input - lastInput);
      /*
      Angle: 0.1 Error: 2.6703523998244125
      Angle: 0.2 Error: 5.3406966652923105
      Angle: 0.3 Error: 8.011024662071957
      Angle: 0.4 Error: 10.681328255881175
      Angle: 0.5 Error: 13.35159931251212
      Angle: 0.6 Error: 16.021829697856063
      Angle: 0.7 Error: 18.692011277928167
      Angle: 0.8 Error: 21.362135918892267
      Angle: 0.9 Error: 24.032195487085634
      Angle: 1.0 Error: 26.70218184904377
      Angle: 1.1 Error: 29.372086871525195
      Angle: 1.2 Error: 32.04190242153614
      Angle: 1.3 Error: 34.711620366355476
      Angle: 1.4 Error: 37.38123257355932
      Angle: 1.5 Error: 40.050730911045925
      Angle: 1.6 Error: 42.72010724706039
      Angle: 1.7 Error: 45.38935345021946
      Angle: 1.8 Error: 48.05846138953628
      Angle: 1.9 Error: 50.7274229344452
      Angle: 2.0 Error: 53.39622995482648
      */
      
      
     double offset = 0.0; 
     if(abs((int) error)>16)
      {
        // Square the error term
        double sign = 1.0;
        offset = OS;
        if(error<1)
        {
          sign = -1.0;
          offset = -OS;
        }
        if (error!=0) error = error/BrkPt;
        error = sign*BrkPt*error*error;
      }
     else
      {
        kp= kp*2.977;
      }
     error= error+offset;
     double DtI = ki*error;
     double Cap = 10.0;
     if (DtI >Cap) DtI = Cap;
     else if (DtI <-Cap) DtI = -Cap;
     ITerm+= DtI;


      /*Compute PID PIDOut*/
      PIDOut = kp*error + ITerm - kd * dInput;
//      Serial.print("ITerm ");
//      Serial.println(ITerm);
//      Serial.print("\tPIDOut ");
//      Serial.println(PIDOut);
      SetPIDOutLimits(-Max, Max);
      if(Gud2Go && !BackAway)
      {
        RunningSpd += PIDOut;
        loopCnt4 += 1;
      }
       /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
   }
   return (int) PIDOut;
   
}   
 
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTimeP = (unsigned long)NewSampleTime;
   }
}

void SetPIDOutLimits(double Min, double Max)
{
   if(Min > Max) return;

   if (PIDOut > Max)
    {
      ITerm -= PIDOut - Max;
      PIDOut = Max;
    }
   else if (PIDOut < Min)
    {
      ITerm  -=(PIDOut - Min);
      PIDOut = Min;
    }
}
 

// End PID computations
//#######################################################################

// Begin PID Steering computations
int ComputePIDS(float Input)
{
   unsigned long now = millis();
   int timeChange = (now - lastTimeS);
   if(timeChange>=SampleTimeY)
   {
      SetSampleTimeS(timeChange);
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      double dInput = (Input - lastInputS);
      double outMax = 255;
      double outMin = -255;
      ITermS+= (kiS * error);
      if(ITermS> outMax) ITermS= outMax;
      else if(ITermS< outMin) ITermS= outMin;
      
 
      /*Compute PID PIDOut*/
      PIDOutS = kpS * error + ITermS - kdS * dInput;
      if(PIDOutS > outMax)
      {
        PIDOutS = outMax;
      }
      else if(PIDOutS < outMin)
      {
        PIDOutS = outMin;
      }
       /*Remember some variables for next time*/
      lastInputS = Input;
      lastTimeS = now;
//      Serial.print("PIDOutS\t");
//      Serial.print(PIDOutS);
   }
   
//   Serial.print("\tYAW");
//   Serial.println(Input);
   return (int) PIDOutS;
   
}   
 
void SetTuningsS(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kpS = Kp;
   kiS = Ki * SampleTimeInSec;
   kdS = Kd / SampleTimeInSec;
}
 
void SetSampleTimeS(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      kiS *= ratio;
      kdS /= ratio;
      SampleTimeY = (unsigned long)NewSampleTime;
   }
}
 


// End PID Steer computations
//#######################################################################

int AplyOffSet( int SpdSgnl, int LastSpeed)
{
 if (SpdSgnl <= LastSpeed && SpdSgnl >0) 
   {
    return SpdSgnl;
   } 
 else if (SpdSgnl >= LastSpeed && SpdSgnl <0) 
   {
    return SpdSgnl;
   }
 else if (SpdSgnl ==0)
   {
    return SpdSgnl;
   }  
   
 if (SpdSgnl <0)
   {
    SpdSgnl = -DeadBnd + SpdSgnl;
   }
 else SpdSgnl = +DeadBnd + SpdSgnl;
 
 if(SpdSgnl > Max) SpdSgnl = Max;
   else if(SpdSgnl < -Max) SpdSgnl = -Max;
 return SpdSgnl;   
}
//#######################################################################
// normally called by the SetMotorSpeed routine [see below] 
void SetMotorDir(int Motor , boolean Dir)
{
  int InPin;
  int InPin_;
  if( Motor == LEFTWHEEL)
   {
    InPin = In1;
    InPin_ = In2;
   }
  else
   {
    InPin = In3;
    InPin_ = In4;
   } 
  if( Dir) // if true set up forward; Note as the car is currently wired the forward command actually makes the car move backwards 
   {
    digitalWrite(InPin, HIGH);
    digitalWrite(InPin_, LOW);

   }
  else
   {
    digitalWrite(InPin, LOW);
    digitalWrite(InPin_, HIGH);
   }
}

void SetMotorSpeed(int Motor,int Speed)
{
  int PWMpin;
  boolean Dir;
  if (Speed > 0)
   {
     Dir = FORWARD;
   }
  else
   {
     Dir = REVERSE;
     Speed = - Speed;
   }
  SetMotorDir(Motor , Dir); 
  if (Motor == LEFTWHEEL)
  {
   PWMpin = EnA;
  }
  else
  {
   PWMpin = EnB;
  }
  analogWrite(PWMpin, Speed);
//  Serial.print(PWMpin);
//  Serial.print("\t");
//  Serial.print(Speed);
//  Serial.print("\t");
//  Serial.println(Steer);
}  

//########################################################################
void ShowMessage(char* text, int value, int line, int pos)
{
  char buf [16];
  sprintf (buf, "%s %03i ", text, value); // Concatenate text message with interger speed value 
  lcd.setCursor(pos,line);
  lcd.print(buf); 
} 
//########################################################################
int waitButton()
{
  int buttonPressed; 
  waitReleaseButton;
  Keypad.blink();
  while((buttonPressed=Keypad.button())==KEYPAD_NONE)
  {
  }
  delay(50);  
  Keypad.noBlink();
  return buttonPressed;
}

void waitReleaseButton()
{
  delay(50);
  while(Keypad.button()!=KEYPAD_NONE)
  {
  }
  delay(50);
}

