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
unsigned int recenterCmd = 're';   //recenter (move to 4,18)

// vars
float encoderPPR = 195;
int leftDir = 1;
//float rampRate = 20000;
//float xRampRate = 2250, yRampRate = 8000;   //motor acceleration in rpm/s2
float xRampRate = 2250, yRampRate = 6000;   //motor acceleration in rpm/s2
float yRampInit = 7000;  //used to reset ramp rate after changing it
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
int x1homePin = 34, yCenterPin = 19, yhomePin = 32;
int goalScoredPin = 8, playerSideGoalScoredPin = 9;
volatile long yTickCorrection = 0;  //correction factor, updated when y-axis center limit switch is hit
bool defend = false, attackStraight = false, attackAngled = false;
float xT1 = 0, yT1 = 0, tT1 = 0, xT2 = 0, yT2 = 0, tT2 = 0, xTau2 = 0, yTau2 = 0;
long dt = 10;  //iteration rate in ms
long yCenterSwitchTicks = 3851;
unsigned long prevTime = 0;

int state = 3, prev_state = -1;
int recenterState = 0, prev_recenterState = -1;
bool recenterRequest = false, goalScored = false, firstLoop = true, centerHomeY = true;
bool rxHomed = false, ryCentered = false, ryHomed = false;
float t3 = 0;

//float kp = 10, ki = 0, kd = 5;  //controller gains
//float kp = 11, ki = 0, kd = (float)4/1000;  //controller gains
//float kp = 20, ki = 30, kd = (float)7/1000;  //controller gains  -- defend or attack point 1
float kpy = 13, kiy = 0, kdy = (float)20/1000;  //controller gains  -- defend or attack point 1
float kpx = 10, kix = 2, kdx = (float)40/1000;  //controller gains  -- defend or attack point 1
float prev_xerror = 0, sum_xerror = 0;
float prev_yerror = 0, sum_yerror = 0;
bool xRampDown = false, yRampDown = false;
bool firstMoveX = true;

//motor data
float xFreq, yFreq, xRampDownTicks, yRampDownTicks, xSpdCommand, ySpdCommand;
long xTicks, yTicks;

// x ramp profile data, xRampLow = 2500 and xRampHigh = 3250
/*
//2500/3250
float xProfileSpd[4] = { 100, 300, 475, 650 };  //ramp profile speeds of interest
float xTau[4] = { 0.0392, 0.113, 0.177, 0.241 };   //time to reach speed in seconds
float xProfile[4] = { 0.22, 2.23, 5.75, 10.94 }; //distance to reach speed in inches
*/
//2250/2250
//float xProfileSpd[5] = { 100, 300, 475, 562.5, 640 };  //ramp profile speeds of interest
//float xTau[5] = { 0.0392, 0.113, 0.177, 0.241, 0.270 };   //time to reach speed in seconds
//float xProfile[5] = { 0.22, 2.23, 5.75, 9.13, 12 }; //distance to reach speed in inches

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
  
  SendAck(startDefenseCmd);
  
  // compute some useful parameters
  xRampRateTicksS2 = xRampRate*microstep*encoderPPR/60;
  xRampHighTicksS2 = xRampHigh*microstep*encoderPPR/60;
  yRampRateTicksS2 = yRampRate*microstep*encoderPPR/60;
  
  xSpeedStep = xRampRate*(float)dt/1000; xSpeedStepIns2 = xSpeedStep*2*PI*rp/60;
  ySpeedStep = yRampRate*(float)dt/1000; ySpeedStepIns2 = ySpeedStep*2*PI*rp/60;
  
  xMinTicks = 6*microstep; yMinTicks = 20*microstep;
  xMaxTicks = 690*microstep; yMaxTicks = 753*microstep;
  
  xBoundLow  = 0.5; yBoundLow = 1.25;
  xBoundHigh = (float)xMaxTicks/(encoderPPR*microstep)*rp*2*PI;
  yBoundHigh = (float)yMaxTicks/(encoderPPR*microstep)*rp*2*PI;
  
  xCenter = 3 + paddleXoffset; yCenter = 19.625 + paddleYoffset;
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
  
  if(dontMove) { state = 0; }  //avoid starting on centering state
  
  delay(200);
}

void loop() {
  // get start time of loop //
  unsigned long loopStartTime = micros();
  unsigned long loopTime = loopStartTime - prevTime;
  prevTime = micros();
  float loopTimeSecs = (float)loopTime/1000000;
  if(Serial.available() > 2) { respondToSerialCmd(); }
  
  // check if a goal was scored
  goalScored = goalScored ? goalScored : analogRead(8)> 900;
  // code to check player side goal, currently works but is unused
  //bool playerSideGoalScored = analogRead(playerSideGoalScoredPin) > 900;
  
  // ramp rate in ticks/s2
  xFreq = xServo.GetPulseFrequency(), yFreq = yServo.GetPulseFrequency();
  //xRampDownTicks = 0.5*xFreq*xFreq/xRampHighTicksS2 + 2*getAbsolute(xFreq)*loopTimeSecs;
  //yRampDownTicks = 0.5*yFreq*yFreq/yRampRateTicksS2 + 2*getAbsolute(yFreq)*loopTimeSecs;
  xRampDownTicks = 0.5*xFreq*xFreq/xRampHighTicksS2;
  yRampDownTicks = 0.5*yFreq*yFreq/yRampRateTicksS2;
  //float rampDownTicks = freq*freq/rampRateTicksS2;  //empirical evidence shows that i'm off by factor of 2
  xTicks = xServo.GetEncoderCount(), yTicks = yServo.GetEncoderCount();
  xSpdCommand = xServo.GetSpeedCommand(), ySpdCommand = yServo.GetSpeedCommand();
  xR = (float)xTicks/(encoderPPR*microstep)*rp*2*PI, yR = (float)yTicks/(encoderPPR*microstep)*rp*2*PI;
  
  //Serial3.print(xServo.GetSpeedCommand()); Serial3.print(","); Serial3.println(yServo.GetSpeedCommand());
  
  // Finite State Machine
  int next_state = state, next_recenterState = recenterState;
  switch(state) {
    case 0: {  //joystick control
      if(prev_state != 0) {
        Serial3.println("Entered State 0");
      }
      //ySpd = 20;
      //Serial3.print(yTicks); Serial3.print(","); Serial3.println(!digitalRead(yCenterPin));
      /*
      if(firstLoop) { break; }
      // ramp profile testing
      xSpd = xSpd_max; t3 += loopTimeSecs;
      if(xR > 25) { xSpd = 0; }
      else {
        Serial3.print(t3,4); Serial3.print(","); Serial3.print(state); Serial3.print("\t");
        Serial3.print(xTicks); Serial3.print(","); Serial3.print(yTicks); Serial3.print("\t");
        Serial3.print(xR - paddleXoffset); Serial3.print(","); Serial3.print(yR - paddleYoffset);
        //Serial3.print(","); Serial3.print(xdot_nom); Serial3.print(","); Serial3.print(ydot_nom);
        //Serial3.print(","); Serial3.print(xdot_mod); Serial3.print(","); Serial3.print(ydot_mod);
        Serial3.print("\t"); Serial3.print(xSpd); Serial3.print(","); Serial3.print(ySpd);
        //Serial3.print(","); Serial3.print(xRampDown); Serial3.print(","); Serial3.print(yRampDown);
        Serial3.print("\t"); Serial3.print(xSpdCommand); Serial3.print(",");
        Serial3.println(ySpdCommand);
      }
      */
      if(debugMode) {
        Serial3.print(xTicks); Serial3.print(","); Serial3.print(yTicks); Serial3.print("\t");
        Serial3.print(xR - paddleXoffset); Serial3.print(","); Serial3.println(yR - paddleYoffset);
      }
      next_state = recenterRequest || goalScored ? 3 : 0;
      next_state = defend ? 1 : next_state;
      next_state = attackAngled ? 4 : next_state;
      next_state = attackStraight ? 2 : next_state;
      break;
    }
    case 1: {  //defense or straight attack pt 1 -- use xT1, yT1, tT1
      // state entry
      if(prev_state != 1) {
        Serial3.println("Entered State 1");
        xRampDown = false; yRampDown = false;
        t3 = 0;
      }
      
      //bool target1reached = moveWithPidRampDown(xT1, yT1, tT1, loopTimeSecs, false);
      bool target1reached = moveWithOpenLoopRampDown(xT1, yT1, t3, loopTimeSecs);
      if(tT1 < 0) { tT1 = 0; }
      t3 += loopTimeSecs;
      //if(tT2 < 0) { attack = false; defend = true; }  //switch flags if out of attack time
      
      // state transitions
      // return to idle state
      next_state = target1reached ? 0 : 1;
      next_state = recenterRequest || goalScored ? 3 : next_state;
      //next_state = getAbsolute(xR-xT1) && getAbsolute(yR-yT1) < 0.4;
      // stay in this state to continue updating position until it's time to attack
      //next_state = target1reached && attack && tT2 <= xTau2 + 0.80 ? 2 : next_state;
      next_state = attackAngled ? 2 : next_state;
      next_state = attackStraight ? 2 : next_state;
      
      // state exit actions
      if(next_state != 1) {
        xSpd = 0; ySpd = 0;
        xRampDown = false; yRampDown = false;
        defend = false;
      }
      break;
    }
    case 2: {  //defend+attack pt2 -- use xT2, yT2, tT2
      //state entry
      if(prev_state != 2) {
        Serial3.println("Entered State 2");
        xRampDown = false; yRampDown = false;
      }
      if(tT2 <= 0) {
        xRampDown = true; yRampDown = true;
        xSpd = 0; ySpd = 0;
      }
    
      //wait until optimal time -- full speed is reached in xTau or yTau seconds
      bool moveX = false;
      moveX = (getAbsolute(tT2 - xTau2) < (float)dt/(2*1000) || tT2 <= xTau2) && xTau2 >= (float)dt/2000;
      //moveY = (getAbsolute(tT2 - yTau2) < (float)dt/(2*1000) || tT2 <= yTau2) && yTau2 >= (float)dt/2000;
    
      //move with no pid ramp down control -- stop point is not important
      float xTargetTicks = xT2*microstep*encoderPPR/(rp*2*PI);
      float yTargetTicks = yT2*microstep*encoderPPR/(rp*2*PI);
      
      //move x if time is right and not in ramp down
      float xLim, yLim;
      if(moveX && !xRampDown) {
        if(xT2 >= xR) {
          // determine ramp down point
          xLim = xTargetTicks - getAbsolute(xFreq)*loopTimeSecs;
          // go full speed until ramp down point is reached
          xRampDown = xTicks + xRampDownTicks >= xLim;
          xSpd = xRampDown ? 0 : xSpd_max;
        }
        else {
          // determine ramp down point
          xLim = xTargetTicks + getAbsolute(xFreq)*loopTimeSecs;
          // go full speed until ramp down point is reached
          xRampDown = xTicks - xRampDownTicks <= xLim;
          xSpd = xRampDown ? 0 : -1*xSpd_max;
        }
      }
      
      // use pid to position y
      bool yOnly = true;
      bool targetReached = moveWithPidRampDown(xT2, yT2, 0.01, loopTimeSecs, yOnly);
      
      if(debugMode) {
        Serial3.print(tT2,4); Serial3.print(","); Serial3.print(state); Serial3.print("\t");
        Serial3.print(xTicks); Serial3.print(","); Serial3.print(yTicks); Serial3.print("\t");
        Serial3.print(xR - paddleXoffset); Serial3.print(","); Serial3.print(yR - paddleYoffset);
        //Serial3.print(","); Serial3.print(xdot_nom); Serial3.print(","); Serial3.print(ydot_nom);
        //Serial3.print(","); Serial3.print(xdot_mod); Serial3.print(","); Serial3.print(ydot_mod);
        Serial3.print("\t"); Serial3.print(xSpd); Serial3.print(","); Serial3.print(ySpd);
        //Serial3.print(","); Serial3.print(xRampDown); Serial3.print(","); Serial3.print(yRampDown);
        Serial3.print("\t"); Serial3.print(xSpdCommand); Serial3.print(",");
        Serial3.println(ySpdCommand);
      }
      //Serial3.flush();
      next_state = xRampDown ? 0 : 2;
      next_state = recenterRequest || goalScored ? 3 : next_state;
      
      // state exit actions
      if(next_state != 2) {
        xSpd = 0; ySpd = 0;
        xRampDown = false; yRampDown = false;
        attackStraight = false;
      }
      break;
    }
    case 3: {  //recenter and rehome axes
      if(prev_state != 3) {
        Serial3.println("Entered State 3"); t3 = 0;
        xRampDown = false; yRampDown = false;
        rxHomed = false; ryCentered = false; ryHomed = false;
        recenterState = 0;
        defend = false; attackStraight = false; attackAngled = false;
        if(goalScored) { centerHomeY = true; }
      }
      if(firstLoop) { break; }  //don't do anything on first loop due to loopTime being a huge number
      
      float xC = xCenter, yC = yCenter;
      //bool targetReached = moveWithPidRampDown(xC, yC, 0.01, xFreq, yFreq,
      //  xRampDownTicks, yRampDownTicks, xTicks, yTicks);
      bool olTargetReached = true, pidTargetReached = false;
      if(recenterState == 0) {  //move with open loop ramp down to center
        // change target y if homing
        if(homingEnabled && centerHomeY) {  //move near center switch
          yC = 23 + paddleYoffset;
        } 
        olTargetReached = moveWithOpenLoopRampDown(xC, yC, 0.01, loopTimeSecs);
        bool transitionCondition = olTargetReached && getAbsolute(xServo.GetSpeedCommand()) <= 200 &&
          getAbsolute(xC - xR) < 15 && getAbsolute(yServo.GetSpeedCommand()) <= 300;
        next_recenterState = transitionCondition ? 1 : 0;
      }
      else if(recenterState == 1 || recenterState == 3) {  //use pid when close
        //recenter state entry
        if(prev_recenterState != 1 && prev_recenterState != 3) { xRampDown = false; yRampDown = false; }
        // change target y if homing
        if(recenterState == 1 && homingEnabled && centerHomeY) {  //move near center switch
          yC = 23 + paddleYoffset;
        }
        // move with pid control to xC, yC
        pidTargetReached = moveWithPidRampDown(xC, yC, 0.01, loopTimeSecs, false);
        // recenter state transitions
        next_recenterState = pidTargetReached && recenterState == 1 ? 2 : recenterState;
        next_recenterState = pidTargetReached && recenterState == 3 ? 4 : next_recenterState;
        
        // rstate exit
        if(pidTargetReached) { xSpd = 0; ySpd = 0; }
      }
      else if(recenterState == 2) {    //home x and centerhome y
        // home x-axis, hit center switch on y-axis
        if(homingEnabled) {
          // only home y if requested
          if(centerHomeY) {
            bool prevyCentered = ryCentered, prevxHomed = rxHomed, prevyHomed = ryHomed;
            ryCentered = ryCentered ? ryCentered : !digitalRead(yCenterPin);
            // also watch home pin in case paddle starts on wrong side of center switch
            ryHomed    = ryHomed ? ryHomed : !digitalRead(yhomePin);
            ySpd = ryCentered || ryHomed ? 0 : -3*homeSpeed;
            // Set yTickCorrection when y-axis first hits center switch
            if(ryCentered && prevyCentered == false) {
              //yTickCorrection = 3835 - yTicks;
              yServo.SetEncoderCount(3835);
              centerHomeY = false;
            }
            else if(ryHomed && prevyHomed == false) {
              yServo.ClearEncoderCount();
              centerHomeY = false;
            }
          }
          else { ySpd = 0; }
          
          // home x
          bool prevxHomed = rxHomed;
          rxHomed = rxHomed ? rxHomed : !digitalRead(x1homePin);
          Serial3.print("Homing status: "); Serial3.print(rxHomed); Serial3.print(","); Serial3.println(ryCentered);
          xSpd = rxHomed ? 0 : -2*homeSpeed;
          
          // Zero x when first homed
          if(rxHomed && prevxHomed == false) { xServo.ClearEncoderCount(); }
          
          // transitions
          next_recenterState = rxHomed && !centerHomeY ? 3 : 2;  //go to pid recentering again
          // rstate exit actions
          if(next_recenterState != 2) {
            //xServo.ClearEncoderCount(); //yServo.ClearEncoderCount();
            rxHomed = false; ryCentered = false;
          }
        }
        else { next_recenterState = 4; }
      }
      
      next_state = next_recenterState == 4 ? 0 : 3;  //idle if done recentering
      //next_state = next_recenterState == 4 && (defend || attack) ? 1 : next_state;
      next_state = recenterState == 2 && defend ? 1 : next_state;  //interrupt homing and switch to defend
      next_state = recenterState == 2 && attackAngled ? 4 : next_state;  //interrupt homing and switch to attack
      next_state = recenterState == 2 && attackStraight ? 2 : next_state;  //interrupt homing and switch to attack
      if(next_state != 3) {
        xSpd = 0; ySpd = 0;
        rxHomed = false; ryCentered = false; ryHomed = false;  //in case rstate 2 was interrupted
        next_recenterState = 0; recenterRequest = false; goalScored = false;
      }
      break;
    }
    case 4:  //angled attack to puck point
    {
      //state entry
      if(prev_state != 4) {
        Serial3.println("Entered State 4");
        xRampDown = false; yRampDown = false;
        //tT2 -= 5*(float)dt/1000; //speed up arrival time
      }
      if(tT2 <= 0) {
        xRampDown = true; yRampDown = true;
        xSpd = 0; ySpd = 0;
      }
    
      //wait until optimal time -- full speed is reached in xTau or yTau seconds
      bool moveX = false;
      moveX = (getAbsolute(tT2 - xTau2) < (float)dt/(2*1000) || tT2 <= xTau2) && xTau2 >= (float)dt/2000;
      //moveY = (getAbsolute(tT2 - yTau2) < (float)dt/(2*1000) || tT2 <= yTau2) && yTau2 >= (float)dt/2000;
      
      //move x if time is right and not in ramp down
      float xLim, yLim;
      if(moveX) {
        // change y ramp rate to arrive at target location at correct time
        float vy = yFreq/(encoderPPR*microstep)*2*PI*rp;
        int dir = yT2 > yR ? 1 : -1;
        float ay = 2*(yT2 + dir*1.6 - yR - vy*tT2)/(tT2*tT2);
        yRampRate = getAbsolute(ay*60/(2*PI*rp));
        yRampRate = yRampRate > yRampInit ? yRampInit : yRampRate;
        yRampRate = yRampRate < 1 ? 1 : yRampRate;
        yServo.SetRampRate(yRampRate);
        dir = ay > 0 ? 1 : -1;
        float yTarget = ay > 0 ? yBoundHigh : yBoundLow;  //set new yTarget to move through puck point
        ySpd = dir*ySpd_max;
        
        bool olTargetReached = moveWithOpenLoopRampDown(xT2, yTarget, 0.01, loopTimeSecs);
        if(debugMode) {
          Serial3.print(tT2); Serial3.print("\t");Serial3.print(xR - paddleXoffset);Serial3.print(",");Serial3.print(yR - paddleYoffset);
          Serial3.print("\t"); Serial3.print(vy); Serial3.print(",");Serial3.println(ay);
        }
      }
      
      if(debugMode) {
        Serial3.print(tT2,4); Serial3.print(","); Serial3.print(state); Serial3.print("\t");
        Serial3.print(xTicks); Serial3.print(","); Serial3.print(yTicks); Serial3.print("\t");
        Serial3.print(xR - paddleXoffset); Serial3.print(","); Serial3.print(yR - paddleYoffset);
        //Serial3.print(","); Serial3.print(xdot_nom); Serial3.print(","); Serial3.print(ydot_nom);
        //Serial3.print(","); Serial3.print(xdot_mod); Serial3.print(","); Serial3.print(ydot_mod);
        Serial3.print("\t"); Serial3.print(xSpd); Serial3.print(","); Serial3.print(ySpd);
        //Serial3.print(","); Serial3.print(xRampDown); Serial3.print(","); Serial3.print(yRampDown);
        Serial3.print("\t"); Serial3.print(xSpdCommand); Serial3.print(",");
        Serial3.println(ySpdCommand);
      }
      //Serial3.flush();
      next_state = xRampDown ? 0 : 4;
      next_state = recenterRequest || goalScored ? 3 : next_state;
      
      // state exit actions
      if(next_state != 4) {
        xSpd = 0; ySpd = 0;
        xRampDown = false; yRampDown = false;
        yRampRate = yRampInit; yServo.SetRampRate(yRampRate);
        attackAngled = false;
      }
      break;
    }
  }
  prev_state = state; state = next_state;
  prev_recenterState = recenterState; recenterState = next_recenterState;
  
  // subtract elapsed time from target times
  if(defend) { tT1 -= loopTimeSecs; }
  if(attackStraight || attackAngled) { tT2 -= loopTimeSecs; }
  
  // apply bounding -- avoid limits
  if(recenterState != 2) {  //don't do this when homing in recenter state
    float xUpperLim = xMaxTicks - getAbsolute(xFreq)*loopTimeSecs;
    float xLowerLim = xMinTicks + getAbsolute(xFreq)*loopTimeSecs;
    float yUpperLim = yMaxTicks - getAbsolute(yFreq)*loopTimeSecs;
    float yLowerLim = yMinTicks + getAbsolute(yFreq)*loopTimeSecs;
    
    if(xSpd > 0 && (xTicks + xRampDownTicks) >= xUpperLim) { xSpd = 0; Serial3.println("Limiting xSpd"); }
    else if(xSpd < 0 && (xTicks - xRampDownTicks) <= xLowerLim) { xSpd = 0; Serial3.println("Limiting xSpd"); }
    
    if(ySpd > 0 && (yTicks + yRampDownTicks) >= yUpperLim) { ySpd = 0; Serial3.println("Limiting ySpd"); }
    else if(ySpd < 0 && (yTicks - yRampDownTicks) <= yLowerLim) { ySpd = 0; Serial3.println("Limiting ySpd"); }
  }
  
  // adjust x ramp -- ramp down faster than ramp up
  float xSpeedCmd = xServo.GetSpeedCommand(), ySpeedCmd = yServo.GetSpeedCommand();
  bool decelerating = (xSpeedCmd < 0 && xSpd > xSpeedCmd) || (xSpeedCmd > 0 && xSpd < xSpeedCmd);
  if(decelerating) { xRampRate = xRampHigh; }
  else { xRampRate = xRampLow; }
  xServo.SetRampRate(xRampRate);
  
  
  if(getAbsolute(xSpd) > xSpd_max) { xSpd = xSpd > 0 ? xSpd_max : -1*xSpd_max; }
  if(getAbsolute(ySpd) > ySpd_max) { ySpd = ySpd > 0 ? ySpd_max : -1*ySpd_max; }
  
  // useful for testing without motors going bonkers
  if(dontMove) { xSpd = 0; ySpd = 0; }
  
  //xServo.SetSpeed(xSpd,dt*1000,0);  //dt in us
  //yServo.SetSpeed(ySpd,dt*1000,0);  //dt in us
  
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
  
  // saturate
  if(deltaTime > dt*1000) {
      Serial3.print("Warning! Exceeded loop time: "); Serial3.println(deltaTime);
  }
  deltaTime = deltaTime > dt*1000 ? dt*1000 : deltaTime;
  
  //use delay instead of delayMicros() because delayMicros() maxes at ~16383
  delay((float)(dt*1000 - deltaTime)/1000);
}

bool moveWithOpenLoopRampDown(float xT, float yT, float tT, float loopTimeSecs)
{
  //move with no pid ramp down control -- stop point is not important
  float xTargetTicks = xT*microstep*encoderPPR/(rp*2*PI);
  float yTargetTicks = yT*microstep*encoderPPR/(rp*2*PI);
  
  //move x and y motors if not in ramp down
  float xLim, yLim;
  if(!xRampDown) {
    if(xT >= xR) {
      // determine ramp down point
      xLim = xTargetTicks - getAbsolute(xFreq)*loopTimeSecs;
      // go full speed until ramp down point is reached
      xRampDown = xSpdCommand > 0 && xTicks + xRampDownTicks >= xLim;
      xSpd = xRampDown ? 0 : xSpd_max;
    }
    else {
      // determine ramp down point
      xLim = xTargetTicks + getAbsolute(xFreq)*loopTimeSecs;
      // go full speed until ramp down point is reached
      xRampDown = xSpdCommand < 0 && xTicks - xRampDownTicks <= xLim;
      xSpd = xRampDown ? 0 : -1*xSpd_max;
    }
  }
  
  if(!yRampDown) {
    if(yT >= yR) {
      yLim = yTargetTicks - getAbsolute(yFreq)*loopTimeSecs;
      yRampDown = yTicks + yRampDownTicks >= yLim;
      ySpd = yRampDown ? 0 : ySpd_max;
    }
    else {
      yLim = yTargetTicks + getAbsolute(yFreq)*loopTimeSecs;
      yRampDown = yTicks - yRampDownTicks <= yLim;
      ySpd = yRampDown ? 0 : -1*ySpd_max;
    }
  }
  if(debugMode) {
    Serial3.print(tT,4); Serial3.print(","); Serial3.print(state); Serial3.print("\t");
    Serial3.print(xTicks); Serial3.print(","); Serial3.print(yTicks); Serial3.print("\t");
    Serial3.print(xR - paddleXoffset); Serial3.print(","); Serial3.print(yR - paddleYoffset);
    Serial3.print("\t"); Serial3.print(xSpd); Serial3.print(","); Serial3.print(ySpd);
    Serial3.print("\t"); Serial3.print(xServo.GetSpeedCommand()); Serial3.print(",");
    Serial3.println(yServo.GetSpeedCommand());
  }
  return (xRampDown && yRampDown);
}

// move towards target point, use pid ramp down to stop within 0.25 inches
// return value indicates target reached
bool moveWithPidRampDown(float xT, float yT, float tT, float loopTimeSecs, bool yOnly)
{
  // actions
  float xTargetTicks = xT*microstep*encoderPPR/(rp*2*PI);
  float yTargetTicks = yT*microstep*encoderPPR/(rp*2*PI);
  
  float xLim, yLim;
  float xRampDownFactor = 1.25, yRampDownFactor = 1.5;
  
  // check if it's time to ramp down
  if(!xRampDown && !yOnly) {
    if(xT >= xR) {
      // determine ramp down point
      xLim = xTargetTicks - getAbsolute(xFreq)*loopTimeSecs;
      // switch to pid if near endpoint
      xRampDown = xTicks + xRampDownFactor*xRampDownTicks >= xLim ? true : false;
    }
    else {
      // determine ramp down point
      xLim = xTargetTicks + getAbsolute(xFreq)*loopTimeSecs;
      // switch to pid if near endpoint
      xRampDown = xTicks - xRampDownFactor*xRampDownTicks <= xLim ? true : false;
    }
    if(xRampDown) { sum_xerror = 0; }
  }
  
  if(!yRampDown) {
    if(yT >= yR) {
      yLim = yTargetTicks - getAbsolute(yFreq)*loopTimeSecs;
      yRampDown = yTicks + yRampDownFactor*yRampDownTicks >= yLim ? true : false;
    }
    else {
      yLim = yTargetTicks + getAbsolute(yFreq)*loopTimeSecs;
      yRampDown = yTicks - yRampDownFactor*yRampDownTicks <= yLim ? true : false;
    }
    if(yRampDown) { sum_yerror = 0; }
  }
  // compute nominal axis speeds for arrival at target destination in time td //
  float xdot_nom = (xT - xR)/tT, ydot_nom = (yT - yR)/tT;
  
  // go full speed when defending if >=2 timesteps of ramping will be involved anyway
  // this should avoid position overshoot when full speeding
  if(getAbsolute(xdot_nom) >= 2*xSpeedStepIns2) { xdot_nom = xT >= xR ? xSpd_max : -1*xSpd_max; }
  if(getAbsolute(ydot_nom) >= 2*ySpeedStepIns2) { ydot_nom = yT >= yR ? ySpd_max : -1*ySpd_max; }
  
  //if(tT < 0) { xdot_nom = 0; ydot_nom = 0; }
  
  // execute pid control //
  // x axis
  float xerror = xT - xR;
  float deriv_xerror = (xerror - prev_xerror)/loopTimeSecs;
  sum_xerror += (xerror + prev_xerror)*loopTimeSecs/2;
  float xdot_mod = kpx*xerror + kix*sum_xerror + kdx*deriv_xerror;
  
  //Serial3.print("pid: "); Serial3.print(xerror); Serial3.print(","); Serial3.print(xdot_nom);
  //Serial3.print(","); Serial3.println(xdot_mod);
  
  // y axis
  float yerror = yT - yR;
  float deriv_yerror = (yerror - prev_yerror)/loopTimeSecs;
  sum_yerror += (yerror + prev_yerror) * loopTimeSecs/2;
  float ydot_mod = kpy*yerror + kiy*sum_yerror + kdy*deriv_yerror;
  
  prev_xerror = xerror; prev_yerror = yerror;
  
  // set speeds //
  // convert in/s to motor rpm
  if(!yOnly) {  //ignore if only y-axis motor is being controlled
    if(xRampDown) { xSpd = getAbsolute(xerror) >= 0.25 ? 60*xdot_mod/(2*PI*rp) : 0; }
    else { xSpd = 60*xdot_nom/(2*PI*rp); }
  }
  if(yRampDown) { ySpd = getAbsolute(yerror) >= 0.25 ? 60*ydot_mod/(2*PI*rp) : 0; }
  else { ySpd = 60*ydot_nom/(2*PI*rp); }
  
  //xSpd = getAbsolute(xerror) > 0.25 ? 60*(xdot_nom + xdot_mod)/(2*PI*rp) : 0;
  //ySpd = getAbsolute(yerror) > 0.25 ? 60*(ydot_nom + ydot_mod)/(2*PI*rp) : 0;
  
  //Serial3.print(tT2,4); tT2 -= loopTimeSecs;  //for getting ramp profile
  if(!(attackStraight || attackAngled) && debugMode) {
    Serial3.print("pid: ");
    if(attackStraight || attackAngled) { Serial3.print(tT2,4); }
    else { Serial3.print(tT,4); } 
    Serial3.print(","); Serial3.print(state); Serial3.print("\t");
    Serial3.print(xTicks); Serial3.print(","); Serial3.print(yTicks); Serial3.print("\t");
    Serial3.print(xR - paddleXoffset); Serial3.print(","); Serial3.print(yR - paddleYoffset);
    //Serial3.print(","); Serial3.print(xdot_nom); Serial3.print(","); Serial3.print(ydot_nom);
    //Serial3.print(","); Serial3.print(xdot_mod); Serial3.print(","); Serial3.print(ydot_mod);
    Serial3.print("\t"); Serial3.print(xSpd); Serial3.print(","); Serial3.print(ySpd);
    //Serial3.print(","); Serial3.print(xRampDown); Serial3.print(","); Serial3.print(yRampDown);
    Serial3.print("\t"); Serial3.print(xServo.GetSpeedCommand()); Serial3.print(",");
    Serial3.println(yServo.GetSpeedCommand());
  }
  
  return getAbsolute(xerror) < 0.25 && getAbsolute(yerror) < 0.25;
}

void respondToSerialCmd()
{
  //Read two bytes to check serial cmd;
  unsigned char firstChar = Serial.read();
  unsigned char secondChar = Serial.read();
  unsigned int cmd = firstChar << 8 | secondChar;
  
  //Serial3.print("Serial cmd received: "); Serial3.print(firstChar); Serial3.print(","); Serial3.println(secondChar);
  
  if(cmd == recenterCmd) {
    unsigned long recStart = micros();
    Serial3.print("Recenter cmd received: "); Serial3.println(recStart);
    //if(state != 0 && state != 1) { FlushSerialInput(); return; }
    unsigned char data = Serial.read();
    // check suffix bytes
    unsigned int suff1 = Serial.read(), suff2 = Serial.read();
    //Serial3.print(suff1); Serial3.println(suff2);
    if(suff1 != 0x08 || suff2 != 0x09) { FlushSerialInput(); return;}
    
    centerHomeY = data == 1 ? true : false;
    recenterRequest = true; defend = false;
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
  // receive target point (x,y) and max arrival time -- 3 floats
  else if(cmd == startDefenseCmd)
  {
    unsigned long defStart = micros();
    if(debugMode) {
      Serial3.print("Defense cmd received: "); Serial3.println(defStart);
    }
    //ignore if attacking
    if(attackStraight || attackAngled) { SendAck(startDefenseCmd); FlushSerialInput(); return; }
    
    unsigned char dataArray[12];
    unsigned int checksum = 0;
    for(int i=0;i<12;i++) {
      dataArray[i]  = Serial.read();
      checksum += dataArray[i];
    }
    
    //read checksum
    unsigned int chkLow = Serial.read();
    unsigned int chkHigh  = Serial.read();
    unsigned int checksumPacket = chkHigh << 8 | chkLow;
    //Serial3.print(checksum); Serial3.print("\t"); Serial3.println(checksumPacket);
    
    //check suffix bytes
    unsigned int suff1 = Serial.read(), suff2 = Serial.read();
    //Serial3.print(suff1); Serial3.println(suff2);
    if(suff1 != 0x08 || suff2 != 0x09) { FlushSerialInput(); return;}
    
    // check checksum
    if(checksumPacket != checksum) { FlushSerialInput(); return; }
    
    float check_xT1 = *((float*)&dataArray[0]) + paddleXoffset;  //compensate for paddle offset
    float check_yT1 = *((float*)&dataArray[4]) + paddleYoffset;
    float check_tT1 = *((float*)&dataArray[8]);
    
    //Serial3.print(check_xT1); Serial3.print(","); Serial3.println(check_yT1);
    
    // stay away from limits
    //if(check_yT1 < yBoundLow || check_yT1 > yBoundHigh) { FlushSerialInput(); return; }
    
    if(!(attackStraight || attackAngled)) { defend = true; }
    xT1 = check_xT1; yT1 = check_yT1; //tT1 = check_tT1; //tT1 = check_tT1/2;
    tT1 = 0.01;
    
    SendAck(startDefenseCmd);
    FlushSerialInput();
    if(debugMode) {
      Serial3.print("Defense cmd time: "); Serial3.println(micros() - defStart);
    }
  }
  else if(cmd == startAttackCmd) {
    unsigned long attStart = micros();
    //receive desired attack point, time of arrival, possibly angle of attack
    if(debugMode) {
      Serial3.print("Arrack cmd received: "); Serial3.println(attStart);
    }  
    unsigned char dataArray[12];
    unsigned int checksum = 0;
    for(int i=0;i<12;i++) {
      dataArray[i]  = Serial.read();
      checksum += dataArray[i];
    }
    
    //read checksum
    unsigned int chkLow = Serial.read();
    unsigned int chkHigh  = Serial.read();
    unsigned int checksumPacket = chkHigh << 8 | chkLow;
    //Serial3.print(checksum); Serial3.print("\t"); Serial3.println(checksumPacket);
    
    //check suffix bytes
    unsigned int suff1 = Serial.read(), suff2 = Serial.read();
    //Serial3.print(suff1); Serial3.println(suff2);
    if(suff1 != 0x08 || suff2 != 0x09) { FlushSerialInput(); return;}
    
    // check checksum
    if(checksumPacket != checksum) { FlushSerialInput(); return; }
    
    // ignore this packet if already attacking
    //if(state == 2) { SendAck(startAttackCmd); return; }
    
    // read packet values, make sure they're in range
    float xP2 = *((float*)&dataArray[0]) + paddleXoffset;  //compensate for paddle offset
    float yP2 = *((float*)&dataArray[4]) + paddleYoffset;
    float check_tT2 = *((float*)&dataArray[8]);
    
    //Serial3.print(xP2); Serial3.print(","); Serial3.println(yP2);
    
    // select shot based on xP2 received
    float margin = 8;  //minimum required distance from y boundaries
    xTau2 = getAbsolute(xP2 - 8.75) < getAbsolute(xP2 - 15) ? xTau[2] : xTau[4];
    
    tT2 = check_tT2;
    xRampDown = false; yRampDown = false;
    
    // set xT2 to max, yT2 to puck y-value
    xT2 = xBoundHigh; yT2 = yP2;
    
    if(yT2 > yBoundLow + margin && yT2 < yBoundHigh - margin) {
      attackAngled = true; attackStraight = false;
    }
    else if(!attackAngled) { attackStraight = true; attackAngled = false; }
    defend = false; 
    //attack = true;
    //Serial3.print("Shot selection: "); Serial3.print(xProfileSpd[4]); Serial3.print(",");
    //Serial3.print(yP2);
    
    //Serial3.print("\t"); Serial3.println(tT2);
    
    SendAck(startAttackCmd);
    FlushSerialInput();
    if(debugMode) {
      Serial3.print("Attack cmd time: "); Serial3.println(micros() - attStart);
    }
  }
  /*
  else if(cmd == sayHelloCmd)
  {
    Serial3.println("Hello!");
    unsigned char spdArray[8];
    unsigned int checksum = 0;
    for(int i=0;i<8;i++) {
      spdArray[i]  = Serial.read();
      checksum += spdArray[i];
    }
    float resp1 = *((float*)&spdArray[0]);
    float resp2 = *((float*)&spdArray[4]);
    Serial3.print(resp1); Serial3.print(","); Serial3.println(resp2);
    
    Serial.write((unsigned char *)(&resp1),4);
    Serial.write((unsigned char *)(&resp2),4);
  }
  else {  //flush input buffer
    FlushSerialInput();
  }
  */
}

//send acknowledgement of specified cmd -- use
void SendAck(unsigned int ackedCmd)
{
  unsigned long ackStart = micros();
  unsigned char cmdHigh = ackedCmd >> 8;
  unsigned char cmdLow = ackedCmd;
  //Serial3.print("ack: ");
  //Serial3.print(cmdHigh); Serial3.print(","); Serial3.println(cmdLow);
  unsigned int checksum = 'a' + cmdHigh + cmdLow;
  unsigned char chkHigh = checksum >> 8;
  unsigned char chkLow = checksum;
  unsigned char msg[7] = { 'a', cmdHigh, cmdLow, chkHigh, chkLow, 0x08, 0x09 };
  
  Serial.write(msg,7);
  //Serial.write('a'); Serial.write(cmdHigh); Serial.write(cmdLow);
  //Serial.write(chkHigh); Serial.write(chkLow);
  //Serial.write(0x08); Serial.write(0x09);  //suffix
  //Serial3.print("Ack time: "); Serial3.println(micros() - ackStart);
}

// flush serial input buffer
void FlushSerialInput()
{
  while(Serial.available()) { Serial.read(); }
}

// move robot to position (yMaxTicks/2, 2000)
void Center()
{
  bool xCentered = false, yCentered = false;
  while(!xCentered || !yCentered)
  {
    float xFreq = xServo.GetPulseFrequency(), yFreq = yServo.GetPulseFrequency();
    float xRampDownTicks = 0.5*xFreq*xFreq/xRampRateTicksS2 + getAbsolute(xFreq)*(float)dt/1000;
    float yRampDownTicks = 0.5*yFreq*yFreq/yRampRateTicksS2 + getAbsolute(yFreq)*(float)dt/1000;
    //float rampDownTicks = freq*freq/rampRateTicksS2;  //empirical evidence shows that i'm off by factor of 2
    long xTicks = xServo.GetEncoderCount(), yTicks = yServo.GetEncoderCount();
    float xUpperLim = 2000 - getAbsolute(xFreq)*(float)dt/1000;
    float yUpperLim = yMaxTicks/2 - getAbsolute(yFreq)*(float)dt/1000;
    
    float spdx = 0, spdy = 0;
    
    if(!xCentered) {
      if((xTicks + xRampDownTicks) >= xUpperLim) { spdx = 0; xCentered = true; }
      else { spdx = centerSpeed;  }
    }
    
    if(!yCentered) {
      if((yTicks + yRampDownTicks) >= yUpperLim) { spdy = 0; yCentered = true; }
      else { spdy = centerSpeed;  }
    }
    xServo.SetSpeed(spdx,dt*1000,0); yServo.SetSpeed(spdy,dt*1000,0);
    delay(dt);
  }
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
    bool x2homed = !digitalRead(yCenterPin);
    //x1Homed = x1Homed ? true : !digitalRead(x1homePin);
    //x2Homed = x2Homed ? true : !digitalRead(x2homePin);
    
    yHomeCount = !digitalRead(yhomePin) ? yHomeCount + 1 : 0;
    yHomed = yHomed || yHomeCount > 2 ? true : false;
    //yHomed  = yHomed  ? true : !digitalRead(yhomePin);
    
    Serial3.print(x1Homed); Serial3.print(","); Serial3.print(x2Homed);
    Serial3.print(","); Serial3.println(yHomed);

    float spdx = x1Homed ? 0 : -1*homeSpeed;
    float spdy = yHomed  ? 0 : -1*homeSpeed;
    Serial3.print(xServo.GetEncoderCount()); Serial3.print(",");
    Serial3.print(yServo.GetEncoderCount()); Serial3.print("\t");
    
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
