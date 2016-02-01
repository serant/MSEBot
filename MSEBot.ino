/*

 MSE 2202 MSEBot base code for Labs 3 and 4
 Language: Arduino
 Authors: Michael Naish and Eugen Porter
 Date: 16/01/17
 
 Rev 1 - Initial version
 Rev 2 - Update for MSEduino v. 2
 
 */
// Matt changes 
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;


// Uncomment keywords to enable debugging output

//define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION
//#define DEBUG_LIGHT_SENSOR
boolean bt_Motors_Enabled = true;
boolean followLineInit = false;
unsigned int leftSpeed;
unsigned int rightSpeed;
unsigned int lastTurn;
long searchtimer;
long searchtimer2 = 0;

int stopCounter = 1;
bool flag1 = true;
bool flag2 = false;

//port pin constants
const int ci_Ultrasonic_Ping = 2;   //input plug
const int ci_Ultrasonic_Data = 3;   //output plug
const int ci_Charlieplex_LED1 = 4;
const int ci_Charlieplex_LED2 = 5;
const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor = 8;
const int ci_Left_Motor = 9;
const int ci_Arm_Motor = 10;
const int ci_Grip_Motor = 11;
const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;
const int ci_Right_Line_Tracker_LED = 6;
const int ci_Middle_Line_Tracker_LED = 9;
const int ci_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int ci_Left_Line_Tracker_Dark_Address_L = 0;
const int ci_Left_Line_Tracker_Dark_Address_H = 1;
const int ci_Left_Line_Tracker_Light_Address_L = 2;
const int ci_Left_Line_Tracker_Light_Address_H = 3;
const int ci_Middle_Line_Tracker_Dark_Address_L = 4;
const int ci_Middle_Line_Tracker_Dark_Address_H = 5;
const int ci_Middle_Line_Tracker_Light_Address_L = 6;
const int ci_Middle_Line_Tracker_Light_Address_H = 7;
const int ci_Right_Line_Tracker_Dark_Address_L = 8;
const int ci_Right_Line_Tracker_Dark_Address_H = 9;
const int ci_Right_Line_Tracker_Light_Address_L = 10;
const int ci_Right_Line_Tracker_Light_Address_H = 11;
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 170;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Closed = 90;        //  "
const int ci_Arm_Servo_Retracted = 55;      //  "
const int ci_Arm_Servo_Extended = 120;      //  "
const int ci_Arm_Servo_Search = 90;
const int ci_Display_Time = 500;
const int ci_Line_Tracker_Calibration_Interval = 100;
const int ci_Line_Tracker_Cal_Measures = 20;
const int ci_Line_Tracker_Tolerance = 50;   // May need to adjust this
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
unsigned int ui_Left_Line_Tracker_Data;
unsigned int ui_Middle_Line_Tracker_Data;
unsigned int ui_Right_Line_Tracker_Data;
unsigned int ui_Motors_Speed = 1900;        // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;
unsigned int ui_Left_Line_Tracker_Dark;
unsigned int ui_Left_Line_Tracker_Light;
unsigned int ui_Middle_Line_Tracker_Dark;
unsigned int ui_Middle_Line_Tracker_Light;
unsigned int ui_Right_Line_Tracker_Dark;
unsigned int ui_Right_Line_Tracker_Light;
unsigned int ui_Line_Tracker_Tolerance;
unsigned int switchDirection= 50;

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
boolean beginRightTurn = true;//when this value is true, the robot will search right
boolean foundFlag = false; //when this value is true, it means the flag has been found by the robot
boolean turning = false; //true when robot is making a 90 degree turn for example

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(ci_Charlieplex_LED1,ci_Charlieplex_LED2,
                       ci_Charlieplex_LED3,ci_Charlieplex_LED4,ci_Mode_Button);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(ci_Arm_Motor);
  pinMode(ci_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(ci_Grip_Motor);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // set up line tracking sensors
  pinMode(ci_Right_Line_Tracker, INPUT);
  pinMode(ci_Middle_Line_Tracker, INPUT);
  pinMode(ci_Left_Line_Tracker, INPUT);
  ui_Line_Tracker_Tolerance = ci_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte); 
  b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
  ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
  ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if(CharliePlexM::ui_Btn)
  {
    if(bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes 
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight.
  switch(ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
    {
      readLineTrackers();
      Ping();
      servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
      servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
      servo_ArmMotor.write(ci_Arm_Servo_Retracted);
      servo_GripMotor.write(ci_Grip_Motor_Closed);
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      ui_Mode_Indicator_Index = 0;
      break;
    } 
  
    case 1:    //Robot Run after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        readLineTrackers();
        Ping();
        

#ifdef DEBUG_ENCODERS           
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

        Serial.print("Encoders L: ");
        Serial.print(l_Left_Motor_Position);
        Serial.print(", R: ");
        Serial.println(l_Right_Motor_Position);
#endif

       /***************************************************************************************
         Add line tracking code here. 
         Adjust motor speed according to information from line tracking sensors and 
         possibly encoder counts.
         
         DESCRIPTION OF EVERYTHING KIND OF
         
         when you first enter the line tracking, there are three conditionals: if all three sensors are light, line tracking if the robot isn't in the middle of turn/retrieve/place, line tracking
           if it is in the middle of something
           
         POINT A
         this loop occurs if all three sensors are on the line (i.e. if at a stop point)
         the turning varaible is essentially what switches it from a mode that involves tracking the line (POINT B) and the mode where it does retrieval/turning stuff (POINT C)
         the stopCounter variable is used to determine what stage it's at; case 1 (stopCounter = 1) is when it's the first time it hits the first stop point
         case 2 (stopCounter = 2) is when the robot finds the line that leads to the box. This is when it sends the robot through the Search(); function
         case 3 will be the journey to the dropoff point..... but the search function isn't working yet
         
         POINT B: this is the normal line tracking stuff from lab 3. So essentially when turning == false, it will go to these statements, meaning that it's just regular line tracking going on atm
         POINT C: this is essentially things that work in conjunction to POINT A when the sensors are not all on the stop point but the robot is in the middle of something. The reason the turn wasn't
         working well before was because the robot would lose track of what to do once one of the sensors move off the yellow because then it goes to the general line tracking stuff. So the idea here is 
         to split the line tracking code in POINT B from the stuff in POINT C. Now when the robot turns to face the box, it still knows to keep turning when the sensors go off of the yellow tape
         There's a really life if statement in POINT C that tells the robot to go back to tracking the line when the middle sensor finds the line and the box is within a certain distance. This is what
         lets the robot run on any track since it will only continue when it find a line that is near the box
         
         What the next step is:
         
         The Search() function at the bottom doesn't work yet. Essentially all it needs to do is make the robot look left and right until the light value for the LED sensor is less than 140 (indicating
         the flag is in sight). If you look at my code right now in the search function, i multiple a variable by -1. So if you go through the algorithm slowly you'll see that it flips the speed
         every time the loop is run.
         
         So essentially:
         
         the robot is going to turn right and look for the LED flag until the ultra sonic sensor notices that there is more than 4cm of distance away from it (indicating that it's turned passed the box)
         once this happens, it'll run through the code in the if statement which flips the speed and the robot will start going the other way
         
         the problem is that when it flips the speeds, the robot doesn't have a chance to move before the loop runs again and the ultra sonic sensor notices that the box is still out of view, so it 
         flips the speeds again -> this literally just keeps happening forever and ever and it looks like a seizure
         
         so if we can figure a way to get it so that it gives the robot a chance to correct its mistake before reanalyzing the distance from the box again, it should work!
       /*************************************************************************************/
                
        //POINT A
        if ((ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))  && (ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && (ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark- ui_Line_Tracker_Tolerance))){
         turning = true;
         switch(stopCounter){
           case 1:
           {
             //robot spins
             leftSpeed = 1700;//makes left wheel go forward
             rightSpeed = 1300;//makes right wheel go backwards
             break;
           }
           
           case 2:
           {
             Grab();
             //servo_ArmMotor.write(ci_Arm_Servo_Search);
             //servo_GripMotor.write(ci_Grip_Motor_Open);
             break;
           }
           
           case 3:
           {
             leftSpeed = ui_Left_Motor_Speed = 1300;
             rightSpeed = ui_Right_Motor_Speed = 1700;
             servo_LeftMotor.writeMicroseconds(leftSpeed); 
             servo_RightMotor.writeMicroseconds(rightSpeed);
             break;
           }
         }
        }
        
        //POINT B
        if(!turning){
          if(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance) && ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)){
          // Move Right a little bit
          bt_Motors_Enabled = true;
          leftSpeed = ui_Left_Motor_Speed = 1615;
          rightSpeed = ui_Right_Motor_Speed = 1600;
          lastTurn = 1;
          
          }
          else if(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance) && ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)){
            // Move left a little bit
            bt_Motors_Enabled = true;
            leftSpeed = ui_Left_Motor_Speed = 1600;
            rightSpeed = ui_Right_Motor_Speed = 1615;
            lastTurn = 0;
            
          }
        
          else if(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            // Move right a lot
            bt_Motors_Enabled = true;
            leftSpeed = ui_Left_Motor_Speed = 1605;
            rightSpeed = ui_Right_Motor_Speed = 1505;
            lastTurn = 1;
            
          }
  
          else if(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            // Stay stright 
            bt_Motors_Enabled = true;
            leftSpeed = ui_Left_Motor_Speed = 1650;
            rightSpeed = ui_Right_Motor_Speed = 1650;
            flag1 = true;    
          }
            
          else if(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
          {
            // Move left a lot
            bt_Motors_Enabled = true;
            leftSpeed = ui_Left_Motor_Speed = 1505;
            rightSpeed = ui_Right_Motor_Speed = 1605;
            lastTurn= 0;
            
          }
        }
        
        //POINT C
        else{
          if((ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && !(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance)) && !(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Data - ui_Line_Tracker_Tolerance)) && ((ul_Echo_Time/58)<20)&& (stopCounter == 1)){
            turning = false;
            stopCounter = 2;
          }
        }
        //END OF POINT C
                
        if(bt_Motors_Enabled)
        {
          servo_LeftMotor.writeMicroseconds(leftSpeed);
          servo_RightMotor.writeMicroseconds(rightSpeed);
        }
        else
        {  
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
        }
#ifdef DEBUG_MOTORS
        Serial.print("Motors enabled: ");
        Serial.print(bt_Motors_Enabled);
        Serial.print(", Default: ");
        Serial.print(ui_Motors_Speed);
        Serial.print(", Left = ");
        Serial.print(ui_Left_Motor_Speed);
        Serial.print(", Right = ");
        Serial.println(ui_Right_Motor_Speed);
#endif    
        ui_Mode_Indicator_Index = 1;
      }
      break;
    } 
    
    case 2:    //Calibrate line tracker light levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          ui_Left_Line_Tracker_Light = 0;
          ui_Middle_Line_Tracker_Light = 0;
          ui_Right_Line_Tracker_Light = 0;
          ul_Calibration_Time = millis();
          ui_Cal_Count = 0;
        }
        else if((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
        {
          ul_Calibration_Time = millis();
          readLineTrackers();
          ui_Left_Line_Tracker_Light += ui_Left_Line_Tracker_Data;
          ui_Middle_Line_Tracker_Light += ui_Middle_Line_Tracker_Data;
          ui_Right_Line_Tracker_Light += ui_Right_Line_Tracker_Data;
          ui_Cal_Count++;
        }
        if(ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
        {
          ui_Left_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
          ui_Middle_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
          ui_Right_Line_Tracker_Light /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Light Levels: Left = ");
          Serial.print(ui_Left_Line_Tracker_Light,DEC);
          Serial.print(", Middle = ");
          Serial.print(ui_Middle_Line_Tracker_Light,DEC);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Line_Tracker_Light,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_L, lowByte(ui_Left_Line_Tracker_Light));
          EEPROM.write(ci_Left_Line_Tracker_Light_Address_H, highByte(ui_Left_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_L, lowByte(ui_Middle_Line_Tracker_Light));
          EEPROM.write(ci_Middle_Line_Tracker_Light_Address_H, highByte(ui_Middle_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_L, lowByte(ui_Right_Line_Tracker_Light));
          EEPROM.write(ci_Right_Line_Tracker_Light_Address_H, highByte(ui_Right_Line_Tracker_Light));
          ui_Robot_State_Index = 0;    // go back to Mode 0
        }
        ui_Mode_Indicator_Index = 2; 
      }
      break;
    }
    
    case 3:    // Calibrate line tracker dark levels after 3 seconds
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          ui_Left_Line_Tracker_Dark = 0;
          ui_Middle_Line_Tracker_Dark = 0;
          ui_Right_Line_Tracker_Dark = 0;
          ul_Calibration_Time = millis();
          ui_Cal_Count = 0;
        }
        else if((millis() - ul_Calibration_Time) > ci_Line_Tracker_Calibration_Interval)
        {
          ul_Calibration_Time = millis();
          readLineTrackers();
          ui_Left_Line_Tracker_Dark += ui_Left_Line_Tracker_Data;
          ui_Middle_Line_Tracker_Dark += ui_Middle_Line_Tracker_Data;
          ui_Right_Line_Tracker_Dark += ui_Right_Line_Tracker_Data;
          ui_Cal_Count++;
        }
        if(ui_Cal_Count == ci_Line_Tracker_Cal_Measures)
        {
          ui_Left_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
          ui_Middle_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
          ui_Right_Line_Tracker_Dark /= ci_Line_Tracker_Cal_Measures;
#ifdef DEBUG_LINE_TRACKER_CALIBRATION
          Serial.print("Dark Levels: Left = ");
          Serial.print(ui_Left_Line_Tracker_Dark,DEC);
          Serial.print(", Middle = ");
          Serial.print(ui_Middle_Line_Tracker_Dark,DEC);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Line_Tracker_Dark,DEC);
#endif           
          EEPROM.write(ci_Left_Line_Tracker_Dark_Address_L, lowByte(ui_Left_Line_Tracker_Dark));
          EEPROM.write(ci_Left_Line_Tracker_Dark_Address_H, highByte(ui_Left_Line_Tracker_Dark));
          EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_L, lowByte(ui_Middle_Line_Tracker_Dark));
          EEPROM.write(ci_Middle_Line_Tracker_Dark_Address_H, highByte(ui_Middle_Line_Tracker_Dark));
          EEPROM.write(ci_Right_Line_Tracker_Dark_Address_L, lowByte(ui_Right_Line_Tracker_Dark));
          EEPROM.write(ci_Right_Line_Tracker_Dark_Address_H, highByte(ui_Right_Line_Tracker_Dark));
          ui_Robot_State_Index = 0;    // go back to Mode 0
        }
        ui_Mode_Indicator_Index = 3;
      }
      break;
    }
   
    case 4:    //Calibrate motor straightness after 3 seconds.
    {
      if(bt_3_S_Time_Up)
      {
        if(!bt_Cal_Initialized)
        {
          bt_Cal_Initialized = true;
          encoder_LeftMotor.zero();
          encoder_RightMotor.zero();
          ul_Calibration_Time = millis();
          servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
          servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
        }
        else if((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time) 
        {
          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); 
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop); 
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
          if(l_Left_Motor_Position > l_Right_Motor_Position)
          {
           // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = 0;
            ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;  
          }
          else
          {
           // May have to update this if different calibration time is used
            ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
            ui_Left_Motor_Offset = 0;
          }
          
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Motor Offsets: Left = ");
          Serial.print(ui_Left_Motor_Offset);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Offset);
#endif              
          EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
          EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));
          
          ui_Robot_State_Index = 0;    // go back to Mode 0 
        }
#ifdef DEBUG_MOTOR_CALIBRATION           
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif        
        ui_Mode_Indicator_Index = 4;
      } 
      break;
    }   
  }

  if((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY  
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
} 

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED,!(ui_Mode_Indicator[ui_Mode_Indicator_Index] & 
                      (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// read values from line trackers and update status of line tracker LEDs
void readLineTrackers()
{
  ui_Left_Line_Tracker_Data = analogRead(ci_Left_Line_Tracker);
  ui_Middle_Line_Tracker_Data = analogRead(ci_Middle_Line_Tracker);
  ui_Right_Line_Tracker_Data = analogRead(ci_Right_Line_Tracker);

  if(ui_Left_Line_Tracker_Data < (ui_Left_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Left_Line_Tracker_LED, LOW);
  }
  if(ui_Middle_Line_Tracker_Data < (ui_Middle_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Middle_Line_Tracker_LED, LOW);
  }
  if(ui_Right_Line_Tracker_Data < (ui_Right_Line_Tracker_Dark - ui_Line_Tracker_Tolerance))
  {
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, HIGH);
  }
  else
  { 
    CharliePlexM::Write(ci_Right_Line_Tracker_LED, LOW);
  }

#ifdef DEBUG_LINE_TRACKERS
  Serial.print("Trackers: Left = ");
  Serial.print(ui_Left_Line_Tracker_Data,DEC);
  Serial.print(", Middle = ");
  Serial.print(ui_Middle_Line_Tracker_Data,DEC);
  Serial.print(", Right = ");
  Serial.println(ui_Right_Line_Tracker_Data,DEC);
#endif

}

// measure distance to target using ultrasonic sensor  
void Ping()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  ul_Echo_Time = pulseIn(ci_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
} 
void Grab(){
  if ((ul_Echo_Time/58)  > 3){
    leftSpeed = 1600;
    rightSpeed = 1600;
  //  servo_LeftMotor.writeMicroseconds(leftSpeed); 
  //  servo_RightMotor.writeMicroseconds(rightSpeed);
  } else {
  //  servo_LeftMotor.writeMicroseconds(1500);
  //  servo_RightMotor.writeMicroseconds(1500);
    servo_ArmMotor.write(ci_Arm_Servo_Search);
    servo_GripMotor.write(ci_Grip_Motor_Open);
    Search();
  }
}    
void Search()
{
  searchtimer = millis();
  searchtimer2 = millis();
  while ((( searchtimer - searchtimer2) <1000) && (analogRead(ci_Light_Sensor) > 130 )){ // turns for 3 seconds or until light sensor is activated
  leftSpeed = 1575;
  rightSpeed = 1425;
  servo_LeftMotor.writeMicroseconds(leftSpeed); 
  servo_RightMotor.writeMicroseconds(rightSpeed);
  searchtimer = millis();
  }
  while (((searchtimer - searchtimer2) <4000) && (analogRead(ci_Light_Sensor) > 130)){ // turns for 5 seconds in the other direction or until light sensor is activated 
    leftSpeed = 1425;
    rightSpeed = 1575;
    servo_LeftMotor.writeMicroseconds(leftSpeed); 
    servo_RightMotor.writeMicroseconds(rightSpeed);
    searchtimer = millis();
  }
  
  /*if(ul_Echo_Time/58 > 4){ // move forward if 
    leftSpeed = 1515;
    rightSpeed = 1515;
  } */
  
  if ( analogRead(ci_Light_Sensor) < 130){
    leftSpeed = ci_Left_Motor_Stop;
    rightSpeed = ci_Right_Motor_Stop;
    servo_LeftMotor.writeMicroseconds(leftSpeed); 
    servo_RightMotor.writeMicroseconds(rightSpeed);
    servo_ArmMotor.write(ci_Arm_Servo_Extended);
    delay(2000);
    servo_GripMotor.write(ci_Grip_Motor_Closed);
    delay(1000);
    servo_ArmMotor.write(ci_Arm_Servo_Search);
    delay(1000);
    stopCounter++;
  }
}

/*void Release(){
  if ((ul_Echo_Time/58)  > 4){
    leftSpeed = 1600;
    rightSpeed = 1600;
    servo_LeftMotor.writeMicroseconds(leftSpeed); 
    servo_RightMotor.writeMicroseconds(rightSpeed);
  } else {
    servo_ArmMotor.write(ci_Arm_Servo_Extended);
    delay (2000);
    servo_GripMotor.write(ci_Grip_Motor_Open);
    delay (1000);
    servo_ArmMotor.write(ci_Arm_Servo_Retracted);
  }
}*/
