#include <CNCServo.h>
#include <SPI.h>

// serial cmds
unsigned int setSpeedCmd = 'ss';  //set target SS motor speed
unsigned int getStateCmd = 'fs';     //get current FSM state
unsigned int getSpeedCmd = 'gs';      //get left and right servo speeds in rpm
unsigned int startDefenseCmd = 'sd';  //command robot to a single point, used in defense
unsigned int startAttackCmd = 'sa';   //command robot to attack at specified point, time
unsigned int sayHelloCmd = 'sh';   //send test message, get a response
unsigned int defendAttackCmd = 'da';  //set defense and attack points
unsigned int recenterCmd = 'rc';   //recenter (move to 4,18)

// vars
float encoderPPR = 195;
//float encoderPPR = 100;
int leftDir = 1;
//float rampRate = 20000;
//float xRampRate = 2250, yRampRate = 8000;   //motor acceleration in rpm/s2
float xRampRate = 2250, yRampRate = 7000;   //motor acceleration in rpm/s2
float xRampLow = 2250, xRampHigh = 2250;
//float xRampRate = 2000, yRampRate = 5000;   //motor acceleration in rpm/s2
//float xRampLow = 2000, xRampHigh = 2500;
float xRampRateTicksS2 = 0, xRampHighTicksS2 = 0, yRampRateTicksS2 = 0;
float xSpeedStep = 0, xSpeedStepIns2 = 0;  //speed change in one timestep
float ySpeedStep = 0, ySpeedStepIns2 = 0;  //speed change in one timestep
float microstep = 10;    //microstep setting on stepper drivers
CNCServo xServo(3,xRampRate,leftDir,encoderPPR, 1/microstep);   //timer 3, pins 2/22
CNCServo yServo(4,yRampRate,leftDir,encoderPPR, 1/microstep);   //timer 4, pins 7/23
float xSpd = 0, ySpd = 0;
float xSpd_max = 800, ySpd_max = 900;
float xR = 0, yR = 0;  //robot position
float xCenter, yCenter;
float rp = 1.40;       //pulley radius
float paddleXoffset = -2.45, paddleYoffset = -2.25;  //-1*(paddle_offset) from table 0,0
float homeSpeed = 20, centerSpeed = 100;
float oscSpd = 1100, xMinTicks = 60, xMaxTicks = 6900, yMaxTicks = 7400, yMinTicks = 200;
float xBoundLow, xBoundHigh, yBoundLow, yBoundHigh;
int xDriveEnablePin1 = 24, xDriveEnablePin2 = 26, yDriveEnablePin = 25;
int x1homePin = 34, yCenterPin = 19, yhomePin = 32, goalScoredPin = 8;
volatile long yTickCorrection = 0;  //correction factor, updated when y-axis center limit switch is hit
bool defend = false, attack = false;
float xT1 = 0, yT1 = 0, tT1 = 0, xT2 = 0, yT2 = 0, tT2 = 0, xTau2 = 0, yTau2 = 0;
long dt = 10;  //iteration rate in ms
unsigned long prevTime = 0;

int state = 3, prev_state = -1;
int recenterState = 0, prev_recenterState = -1;
bool recenterRequest = false, goalScored = false, firstLoop = true, centerHomeY = true;
bool rxHomed = false, ryCentered = false, ryHomed = false;
bool xRampDown = false, yRampDown = false;

//motor data
float xFreq, yFreq, xRampDownTicks, yRampDownTicks, xSpdCommand, ySpdCommand;
long xTicks, yTicks;

// x ramp profile data, xRampLow = 2500 and xRampHigh = 3250

//2250/2250, dt of 15 -- only last value (idx 4) is true
float xProfileSpd[5] = { 100, 300, 475, 562.5, 625 };  //ramp profile speeds of interest
float xTau[5] = { 0.0392, 0.113, 0.177, 0.241, 0.278 };   //time to reach speed in seconds
float xProfile[5] = { 0.22, 2.23, 5.75, 9.13, 12 }; //distance to reach speed in inches

// y ramp profile data
float yProfileSpd[6] = {0, 80, 160, 320, 480, 640 };
float yTau[6] = { 0, 0.01, 0.0187, 0.0373, 0.0669, 0.0856 };
float yProfile[6] = { 0, 0, 0.11, 0.71, 2.03, 3.46 };

bool homingEnabled = true;  //toggle to enable/disable homing
bool dontMove = false;      //toggle to enable/disable motors during testing
bool debugMode = false;

void setup() {
  Serial.begin(115200); Serial3.begin(57600);
  pinMode(xDriveEnablePin1,OUTPUT); pinMode(xDriveEnablePin2,OUTPUT);
  digitalWrite(xDriveEnablePin1,HIGH); digitalWrite(xDriveEnablePin2,HIGH);
  pinMode(yDriveEnablePin,OUTPUT); digitalWrite(yDriveEnablePin,HIGH);
  //leftServo.SetRampRate(1);
  
  // compute some useful parameters
  xRampRateTicksS2 = xRampRate*microstep*encoderPPR/60;
  xRampHighTicksS2 = xRampHigh*microstep*encoderPPR/60;
  yRampRateTicksS2 = yRampRate*microstep*encoderPPR/60;
  
  xSpeedStep = xRampRate*(float)dt/1000; xSpeedStepIns2 = xSpeedStep*2*PI*rp/60;
  ySpeedStep = yRampRate*(float)dt/1000; ySpeedStepIns2 = ySpeedStep*2*PI*rp/60;
  
  xMinTicks = 6*microstep; yMinTicks = 20*microstep;
  xMaxTicks = 690*microstep; yMaxTicks = 740*microstep;
  
  xBoundLow  = 0.5; yBoundLow = 1.5;
  xBoundHigh = (float)xMaxTicks/(encoderPPR*microstep)*rp*2*PI;
  yBoundHigh = (float)yMaxTicks/(encoderPPR*microstep)*rp*2*PI;
  
  xCenter = 3 + paddleXoffset; yCenter = 19.625 + paddleXoffset;
  //defend = true; xT1 = 20; yT1 = 30; tT1 = 0.4;
  
  // homing routine
  if(homingEnabled && !dontMove) {
    Home();
  
    // make sure servos come to a stop
    for(int i=0; i<10; i++) {
      xServo.SetSpeed(0,dt*1000,0); yServo.SetSpeed(0,dt*1000,0);
      Serial3.print(xServo.GetState()); Serial3.print(",");
      Serial3.println(yServo.GetState());
      delay(dt);
    }
    xServo.ClearEncoderCount(); yServo.ClearEncoderCount();
  }
  
  delay(200);
}

void loop() {
  // get start time of loop //
  unsigned long loopStartTime = micros();
  unsigned long loopTime = loopStartTime - prevTime;
  prevTime = micros();
  float loopTimeSecs = (float)loopTime/1000000;
  if(Serial.available() > 2) { respondToSerialCmd(); }
  
  // ramp rate in ticks/s2
  xFreq = xServo.GetPulseFrequency(), yFreq = yServo.GetPulseFrequency();
  xRampDownTicks = 0.5*xFreq*xFreq/xRampHighTicksS2 + 2*getAbsolute(xFreq)*loopTimeSecs;
  yRampDownTicks = 0.5*yFreq*yFreq/yRampRateTicksS2 + 2*getAbsolute(yFreq)*loopTimeSecs;
  xTicks = xServo.GetEncoderCount(), yTicks = yServo.GetEncoderCount();
  xSpdCommand = xServo.GetSpeedCommand(), ySpdCommand = yServo.GetSpeedCommand();
  xR = (float)xTicks/(encoderPPR*microstep)*rp*2*PI, yR = (float)yTicks/(encoderPPR*microstep)*rp*2*PI;
  
  // check if a goal was scored
  goalScored = goalScored ? goalScored : analogRead(8)> 900;
  
  // ramp to target speed - set PWM frequency
  xServo.SetSpeed(xSpd,loopTime,0); yServo.SetSpeed(ySpd,loopTime,0);
  
  firstLoop = false;  //flag to avoid weird things happening on first loop iteration
  
  // use variable delay to get consistent loop iteration time //
  //check for and correct overflow
  unsigned long loopEndTime = micros(), deltaTime = 0;
  if(loopStartTime > loopEndTime) { deltaTime = (4294967295 - loopStartTime) + loopEndTime; }
  else { deltaTime = loopEndTime - loopStartTime; }
  //if(state != 0) {
  //  Serial3.print("delay time: "); Serial3.println(dt*1000-deltaTime);
  //}
  
  // apply bounding -- avoid limits
  float xUpperLim = xMaxTicks - getAbsolute(xFreq)*loopTimeSecs;
  float xLowerLim = xMinTicks + getAbsolute(xFreq)*loopTimeSecs;
  float yUpperLim = yMaxTicks - getAbsolute(yFreq)*loopTimeSecs;
  float yLowerLim = yMinTicks + getAbsolute(yFreq)*loopTimeSecs;
  
  if(xSpd > 0 && (xTicks + xRampDownTicks) >= xUpperLim) { xSpd = 0; Serial3.println("Limiting xSpd"); }
  else if(xSpd < 0 && (xTicks - xRampDownTicks) <= xLowerLim) { xSpd = 0; Serial3.println("Limiting xSpd"); }
  
  if(ySpd > 0 && (yTicks + yRampDownTicks) >= yUpperLim) { ySpd = 0; Serial3.println("Limiting ySpd"); }
  else if(ySpd < 0 && (yTicks - yRampDownTicks) <= yLowerLim) { ySpd = 0; Serial3.println("Limiting ySpd"); }
  
  // adjust x ramp -- ramp down faster than ramp up
  float xSpeedCmd = xServo.GetSpeedCommand(), ySpeedCmd = yServo.GetSpeedCommand();
  bool decelerating = (xSpeedCmd < 0 && xSpd > xSpeedCmd) || (xSpeedCmd > 0 && xSpd < xSpeedCmd);
  if(decelerating) { xRampRate = xRampHigh; }
  else { xRampRate = xRampLow; }
  xServo.SetRampRate(xRampRate);
  
  if(getAbsolute(xSpd) > xSpd_max) { xSpd = xSpd > 0 ? xSpd_max : -1*xSpd_max; }
  if(getAbsolute(ySpd) > ySpd_max) { ySpd = ySpd > 0 ? ySpd_max : -1*ySpd_max; }
  
  // saturate
  if(deltaTime > dt*1000) {
      Serial3.print("Warning! Exceeded loop time: "); Serial3.println(deltaTime);
  }
  deltaTime = deltaTime > dt*1000 ? dt*1000 : deltaTime;
  
  //use delay instead of delayMicros() because delayMicros() maxes at ~16383
  delay((float)(dt*1000 - deltaTime)/1000);
}

void respondToSerialCmd()
{
  //Read two bytes to check serial cmd;
  unsigned char firstChar = Serial.read();
  unsigned char secondChar = Serial.read();
  unsigned int cmd = firstChar << 8 | secondChar;
  
  // Receive defend and attack points, decide which to use
  if(cmd == recenterCmd) {
    unsigned long recStart = micros();
    //Serial3.println("Recenter cmd received.");
    //if(state != 0 && state != 1) { FlushSerialInput(); return; }
    unsigned char data = Serial.read();
    // check suffix bytes
    unsigned int suff1 = Serial.read(), suff2 = Serial.read();
    Serial3.print(suff1); Serial3.println(suff2);
    if(suff1 != 0x08 || suff2 != 0x09) { FlushSerialInput(); return;}
    
    centerHomeY = data == 1 ? true : false;
    recenterRequest = true;
    SendAck(recenterCmd);
    FlushSerialInput();
    Serial3.print("Recenter cmd time: "); Serial3.println(micros() - recStart);
  }
  //Set motor SS speeds in RPM, saturated to 300 rpm (2 x 4 byte float)
  //first 4 bytes are left motor, next 4 bytes are right motor
  //Also specify whether motors should rotate continuously (1)on or (0)off (1 byte)
  else if(cmd == setSpeedCmd)
  {
    //while(Serial.available() < 8); //wait for 8 characters
    unsigned int checksum = 0;
    unsigned char spdArray[8];
    for(int i=0;i<8;i++) {
      spdArray[i]  = Serial.read();
      checksum += spdArray[i];
    }
    char contFlag = Serial.read();
    checksum += contFlag;
    
    //read checksum
    unsigned int chkLow = Serial.read();
    unsigned int chkHigh  = Serial.read();
    unsigned int checksumPacket = chkHigh << 8 | chkLow;
    //Serial3.print(checksum); Serial3.print("\t"); Serial3.println(checksumPacket);
    if(checksumPacket != checksum) { FlushSerialInput(); return; }
    
    xSpd = *((float*)&spdArray[0]); ySpd = *((float*)&spdArray[4]);
    
    //set motors in continuous rotation mode
    if(contFlag == 0) { xSpd = 0; ySpd = 0; }
    
    //saturate
    xSpd = xSpd > xSpd_max ? xSpd_max : xSpd;
    xSpd = xSpd < -1*xSpd_max ? -1*xSpd_max : xSpd;
    ySpd = ySpd > ySpd_max ? ySpd_max : ySpd;
    ySpd = ySpd < -1*ySpd_max ? -1*ySpd_max : ySpd;
    
    //Serial.write((unsigned char *)(&xSpd),4);
    //Serial.write((unsigned char *)(&ySpd),4);
    
    //Serial3.print(xSpd); Serial3.print(","); Serial3.println(ySpd);
  }
  //Get current FSM state
  else if(cmd == getStateCmd)
  {
    int servoState = xServo.GetState();
    Serial.write((unsigned char *)(&servoState),2);
  }
  else if(cmd == getSpeedCmd)
  {
    float speedCMD = yServo.GetSpeedCommand();
    Serial.write((unsigned char *)(&speedCMD),4);
  }
}

// flush serial input buffer
void FlushSerialInput()
{
  while(Serial.available()) { Serial.read(); }
}

// Move towards 0 on both axes until limit switches are tripped
void Home() {
  pinMode(x1homePin,INPUT); pinMode(yhomePin,INPUT);
  bool x1Homed = false, x2Homed = false, yHomed = false;
  int xHomeCount = 0, yHomeCount = 0;
  while( !(x1Homed && yHomed) ) {
    // latch homed vars once limit switch is hit
    xHomeCount = !digitalRead(x1homePin) ? xHomeCount + 1 : 0;
    x1Homed = x1Homed || xHomeCount > 2 ? true : false;
    //x1Homed = x1Homed ? true : !digitalRead(x1homePin);
    //x2Homed = x2Homed ? true : !digitalRead(x2homePin);
    
    yHomeCount = !digitalRead(yhomePin) ? yHomeCount + 1 : 0;
    yHomed = yHomed || yHomeCount > 2 ? true : false;
    //yHomed  = yHomed  ? true : !digitalRead(yhomePin);
    
    Serial3.print(x1Homed); Serial3.print(","); Serial3.print(x2Homed);
    Serial3.print(","); Serial3.println(yHomed);

    float spdx = x1Homed ? 0 : -1*homeSpeed;
    float spdy = yHomed  ? 0 : -1*homeSpeed;
    Serial3.print(spdx); Serial3.print(",");
    Serial3.print(spdy); Serial3.print("\t");
    
    xServo.SetSpeed(spdx,dt*1000,0); yServo.SetSpeed(spdy,dt*1000,0);
    
    //Serial3.print(xServo.GetSpeedCommand()); Serial3.print(",");
    //Serial3.println(yServo.GetSpeedCommand());
    
    delay(dt);
  }
  
  xServo.ClearEncoderCount(); yServo.ClearEncoderCount();
  delay(dt);
}

float getAbsolute(float val)
{
  val = val < 0 ? -1*val : val;
  return val;
}
