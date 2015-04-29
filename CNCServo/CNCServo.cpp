#include "CNCServo.h"

//<<constructor>>
//possible timer values: 3 or 4
//for direction, use 1 for normal dirSignal or -1 to invert
//multiplier is inverse of microstep. e.g. 10x microstep would be 0.1
CNCServo::CNCServo(int _timer, float _rampRate, int _direction, float _encoderPPR, float multiplier) {
	speedCMD = 0;
	encoderCount = 0;
	prev_encoderCount = 0;
	encoderTickRate = 0;
	rampDownTicks = 0;
	fpulseACT = 0;
	geckodriveMultiplier = multiplier;
	prev_speedACT = 0;
	previousDir = LOW;
	ocraVal = 65535;
	encoderPPR = _encoderPPR;
	rampRate = _rampRate;
	timer = _timer;
	direction = _direction;
	slaveSelectPin = 8;	//tentatively
	InitTimerAndPins();
	InitEncoder();
}

//<<destructor>>
CNCServo::~CNCServo() {

}

// public methods //

// ramp motors to input speed
// this method must be called on each iteration of the main program
// time interval is timing iteration rate in microseconds
void CNCServo::SetSpeed(float speedSS, long timeInterval, float syncMod) {
	float speedACT = RampMotor(speedSS,timeInterval,syncMod);
	UpdateEncoderCount(timeInterval);
}

void CNCServo::SetRampRate(float _rampRate) {
	rampRate = _rampRate;
}

void CNCServo::SetEncoderPPR(float _encoderPPR) {
	encoderPPR = _encoderPPR;
}

long CNCServo::GetEncoderCount() {
	return encoderCount;
}

float CNCServo::GetEncoderTickRate() {
	return encoderTickRate;
}

float CNCServo::GetRampDownTicks() {
	return rampDownTicks;
}

float CNCServo::GetPulseFrequency() {
	return fpulseACT;
}

float CNCServo::GetSpeedCommand() {
	return speedCMD;
}

int CNCServo::GetOCRAVal() {
	return ocraVal;
}

int CNCServo::GetState() {
	return state;
}

void CNCServo::UpdateEncoderCount(long dt) {

	// expected and reported counts
	long expectedCount = encoderCount + direction*fpulseACT*(float)dt/(float)1000000;
	encoderCount = ReadEncoder();	//reported

	//set equal to expected value if encoderCount is massive
	//protects against spurious buffer board output
	if(encoderCount > 100000 || encoderCount < -100000) {
		encoderCount = expectedCount;
	}
	/*
	// replace with expected if difference > 0.5 revolution
	long delta = expectedCount - encoderCount;
	if(delta > 20480 || delta < -20480) { 
		encoderCount = expectedCount;
		Serial.println("Replaced encoder count.");
	}
	*/
	// Estimate encoderTickRate
	encoderTickRate = (encoderCount - prev_encoderCount) / ((float)dt/(float)1000000);
	/*
	float deltaSpeed = encoderTickRate - direction*fpulseACT;
	if(deltaSpeed > 60000 || deltaSpeed < -60000) {
		encoderTickRate = direction*fpulseACT;
		Serial.println("Replaced tick rate.");
	}
	*/
	float rampRate_ticks = encoderPPR*rampRate/(float)60;
	rampDownTicks = 0.5*encoderTickRate*encoderTickRate/rampRate_ticks;

	prev_encoderCount = encoderCount;
	//Serial.print(encoderCount);
      	//Serial.print("\t");
      	//Serial.print(encoderTickRate);
      	//if(timer==3) { Serial.print("\t"); }
      	//else if(timer==4) { Serial.println(""); }
}

void CNCServo::SetEncoderCount(long newValue) {
	newValue *= -1;		//this is necessary
	// Set encoder's data register to 0
	digitalWrite(slaveSelectPin,LOW);      // Begin SPI conversation  
	// Write to DTR
	SPI.transfer(0x98);
	// Load data
	SPI.transfer(newValue >> 24);  // Highest order byte
	SPI.transfer(newValue >> 16);  
	SPI.transfer(newValue >> 8);
	SPI.transfer(newValue);  // lowest order byte
	digitalWrite(slaveSelectPin,HIGH);     // Terminate SPI conversation 

	delayMicroseconds(100);  // provides some breathing room between SPI conversations

	// Set encoder1's current data register to center
	digitalWrite(slaveSelectPin,LOW);      // Begin SPI conversation  
	SPI.transfer(0xE0);
	digitalWrite(slaveSelectPin,HIGH);     // Terminate SPI conversation

	delayMicroseconds(100);  // provides some breathing room between SPI conversations

	encoderCount = 0;
	prev_encoderCount = 0;
	encoderTickRate = 0;
}

// private methods //

// Accelerates motors if not at target SS speed, holds constant otherwise
// speedSS in rpm, dt in microseconds
// returns the actual speed (rev/s) attained after discretization
float CNCServo::RampMotor(float speedSS, unsigned long dt, float syncMod)
{
	//finite state machine
	int next_state;
	switch(state) 
	{
		case 0: { //idle state -- pwm disabled
			//respond immediately to change in speedSS
			if(speedSS > 0) {
				next_state = 2;
				state = 2;
				RampMotor(speedSS,dt,syncMod);
				break;
			}
			else if(speedSS < 0) {
				next_state = 3;
				state = 3;
				RampMotor(speedSS,dt,syncMod);
				break;
			}
			// actions //
			speedCMD = 0;
			fpulseACT = 0;
			
			if(pwmEnabled == true) {
				pinMode(stepPin,INPUT);    //disable pwm output
				pwmEnabled = false;
			}

			// transitions //
			next_state = speedSS > 0 ? 2 : 0;  //ramp up
			next_state = speedSS < 0 ? 3 : next_state;  //ramp down
			break;
		}	
		case 1: { //steady state
			// actions //
			//respond immediately to change in speedSS
			if(speedSS > speedCMD) {
				next_state = 2;
				state = 2;
				RampMotor(speedSS,dt,syncMod);
				break;
			}
			else if(speedSS < speedCMD) {
				next_state = 3;
				state = 3;
				RampMotor(speedSS,dt,syncMod);
				break;
			}
			
			// transitions //
			next_state = speedSS > speedCMD ? 2 : 1;  //ramp up condition
			next_state = speedSS < speedCMD ? 3 : next_state;  //ramp down condition
			break;
		}	
		case 2: { //ramp up state
			// actions //
			//immediate transition if speedSS drops below speedCMD while ramping
			if(speedSS < speedCMD) {
				next_state = 3;
				state = 3;
				RampMotor(speedSS,dt,syncMod);
				break;
			}

			//accelerate current speed
			speedCMD = speedCMD + (float)dt*rampRate/(float)1000000;
			//saturate to commanded speed
			speedCMD = speedCMD > speedSS ? speedSS : speedCMD;

			//write to direction pin and update timing registers
			if(speedCMD != 0) { SetOutputs(speedCMD); }

			// transitions //
			next_state = speedCMD == speedSS ? 1 : 2;  //steady state condition
			next_state = speedCMD == speedSS && speedSS == 0 ? 0 : next_state; //idle
			break;
		}	
		case 3: {  //ramp down state
			// actions //
			//immediate transition if speedSS jumps above speedCMD while ramping
			if(speedSS > speedCMD) {
				next_state = 2;
				state = 2;
				RampMotor(speedSS,dt,syncMod);
				break;
			}
			
			//deaccelerate current speed
			speedCMD = (speedCMD - (float)dt*rampRate/(float)1000000);
			//saturate to commanded speed
			speedCMD = speedCMD < speedSS ? speedSS : speedCMD;

			//write to direction pin and update timing registers
			if(speedCMD != 0) { SetOutputs(speedCMD); }

			// transitions //
			next_state = speedCMD == speedSS ? 1 : 3;  //steady state condition
			next_state = speedCMD == speedSS && speedSS == 0 ? 0 : next_state; //idle
			break;
		}
	}
	//Serial.print(speedSS);
	//Serial.print("\t");
	//Serial.print(speedCMD);
	//Serial.print("\t");
	state = next_state;
	//return fpulseACT/(float)81920;
	return fpulseACT/encoderPPR;
}

// initialize timer for Fast PWM mode with OCRA top limit of counter
// and set step and dir pins
void CNCServo::InitTimerAndPins()
{
	//WGM (Waveform Generation Mode) pins set to 1111 for fast PWM
	if(timer == 3) {
  		TCCR3A = _BV(COM3A0) | _BV(COM3B1) | _BV(WGM31) | _BV(WGM30);
  		//TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);    //CS 001, prescaler 1
  		//TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS32) | _BV(CS30);    //CS 001, prescaler 1024
  		TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);    //CS 001, prescaler 8
  		stepPin = 2;
  		dirPin = 22;
  		slaveSelectPin = 8;	//encoder 1
  		pinMode(dirPin,OUTPUT);
	}
	else if(timer == 4) {
		TCCR4A = _BV(COM4A0) | _BV(COM4B1) | _BV(WGM41) | _BV(WGM40);
  		//TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);    //CS 001, prescaler 1
  		//TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS42) | _BV(CS40);    //CS 001, prescaler 1024
  		TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);    //CS 001, prescaler 8
  		stepPin = 7;
  		dirPin = 23;
  		slaveSelectPin = 9;	//encoder 2
  		pinMode(dirPin,OUTPUT);
	}
}

void CNCServo::InitEncoder() {
  
	// Set slave select as outputs
	pinMode(slaveSelectPin, OUTPUT);

	// Raise select pins
	// Communication begins when you drop the individual select signsl
	digitalWrite(slaveSelectPin,HIGH);

	SPI.begin();

	// Initialize encoder
	//    Clock division factor: 0
	//    Negative index input
	//    free-running count mode
	//    non-quadrature mode
	// NOTE: For more information on commands, see datasheet
	digitalWrite(slaveSelectPin,LOW);        // Begin SPI conversation
	SPI.transfer(0x88);                       // Write to MDR0
	//SPI.transfer(0x03);                       // Configure to 4 byte mode
        SPI.transfer(0x00);                      // set to non-quadrature mode
	digitalWrite(slaveSelectPin,HIGH);       // Terminate SPI conversation 
	ClearEncoderCount();
}

long CNCServo::ReadEncoder() {
	// Initialize temporary variables for SPI read
	unsigned int count_1, count_2, count_3, count_4;
	long count_value;  

	// Read encoder
	digitalWrite(slaveSelectPin,LOW);      // Begin SPI conversation
	SPI.transfer(0x60);                     // Request count
	count_1 = SPI.transfer(0x00);           // Read highest order byte
	count_2 = SPI.transfer(0x00);           
	count_3 = SPI.transfer(0x00);           
	count_4 = SPI.transfer(0x00);           // Read lowest order byte
	digitalWrite(slaveSelectPin,HIGH);     // Terminate SPI conversation 

	// Calculate encoder count
	count_value = (count_1 << 8) + count_2;
	count_value = (count_value << 8) + count_3;
	count_value = (count_value << 8) + count_4;

	count_value = -1*direction*count_value;
	return count_value;
}

void CNCServo::ClearEncoderCount() {
	// Set encoder's data register to 0
	digitalWrite(slaveSelectPin,LOW);      // Begin SPI conversation  
	// Write to DTR
	SPI.transfer(0x98);
	// Load data
	SPI.transfer(0x00);  // Highest order byte
	SPI.transfer(0x00);  
	SPI.transfer(0x00);
	SPI.transfer(0x00);  // lowest order byte
	digitalWrite(slaveSelectPin,HIGH);     // Terminate SPI conversation 

	delayMicroseconds(100);  // provides some breathing room between SPI conversations

	// Set encoder1's current data register to center
	digitalWrite(slaveSelectPin,LOW);      // Begin SPI conversation  
	SPI.transfer(0xE0);
	digitalWrite(slaveSelectPin,HIGH);     // Terminate SPI conversation

	delayMicroseconds(100);  // provides some breathing room between SPI conversations

	encoderCount = 0;
	prev_encoderCount = 0;
	encoderTickRate = 0;
}

// write direction signal and set PWM outputs -- input is speedCMD
// speedCMD in rpm
// updates fpulseACT -- actual pulse output frequency after discretization
void CNCServo::SetOutputs(float spd)
{
	//Serial.print(spd);
	//Serial.print("\t");
	// determine direction signal
	bool dirSignal;
	if(direction == 1) { dirSignal = spd > 0 ? LOW : HIGH; }
	else	{ dirSignal = spd > 0 ? HIGH : LOW; }

	// disable pwm while switching direction signal
	if(dirSignal != previousDir) {
		pinMode(stepPin,INPUT);
		pwmEnabled = false;
	}
	// write direction signal
	digitalWrite(dirPin,dirSignal);
	previousDir = dirSignal;
	
	spd = spd > 0 ? spd : -1*spd;
	spd = spd < 0.2 ? 0.2 : spd;
	//spd = spd < 0.4 ? 0.4 : spd;
	//speed -> register values//
	//compute encoder pulse frequency
	float fpulse = encoderPPR*spd/60*1/geckodriveMultiplier;  //Hz
	float prescaler = 8;	//for now only one constant prescaler is used

    	float pulsePeriod = 1/fpulse;
    	float tPulse = .00001;		//10 uS pulse width
	//compute ocra value
	ocraVal = round((float)16000000/(prescaler*fpulse))-1;  //16 MHz clock frequency
	ocraVal = ocraVal < 1 ? 1 : ocraVal;
	//ocraVal = ocraVal > 65535 ? 65535 : ocraVal;
	unsigned int ocrbVal = (float)(ocraVal+1)/(float)2 - 1;	//50% duty cycle
	
	if(ocrbVal == 0) { ocrbVal = 1; }
	if(ocrbVal > ocraVal) { ocrbVal = ocraVal - 2; }

	if(timer == 3) {
		//TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS30);    //CS 001, prescaler 1
		//TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS32) | _BV(CS30);    //CS 001, prescaler 1024
		//TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS32);    //CS 001, prescaler 256
		//TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31) | _BV(CS30);    //CS 001, prescaler 64
		TCCR3B = _BV(WGM33) | _BV(WGM32) | _BV(CS31);    //CS 001, prescaler 8
		OCR3A = ocraVal;
		OCR3B = ocrbVal;
	}
	else if(timer == 4) {
		//TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS40);    //CS 001, prescaler 1
		//TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS42);    //CS 001, prescaler 256
		//TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41) | _BV(CS40);    //CS 001, prescaler 64
		TCCR4B = _BV(WGM43) | _BV(WGM42) | _BV(CS41);    //CS 001, prescaler 8
		OCR4A = ocraVal;
		OCR4B = ocrbVal;
	}

	//fpulseACT = fpulse < 1 ? 0 : (float)16000000/(prescaler*(ocraVal+1))*geckodriveMultiplier;
	fpulseACT = fpulse < 1 ? 0 : (float)16000000/(prescaler*(ocraVal+1));
	fpulseACT = dirSignal == LOW ? fpulseACT : -1*fpulseACT;

	//Serial.print(spd);
	//Serial.print(",");
	//Serial.println(fpulseACT);
	//Serial.print(state);
	//Serial.print(",");
	//Serial.print(speedCMD);
	//Serial.print(",");
	//Serial.print(ocraVal);
	//Serial.print(",");
	//Serial.print(fpulse);
	//Serial.print(",");
	//Serial.println(fpulseACT);

	//enable pwm output
	if(pwmEnabled == false) {
		pinMode(stepPin,OUTPUT);
		pwmEnabled = true;
	}
}




