#ifndef LED13_H
#define LED13_H
 
#include <Arduino.h>
#include <SPI.h>

class CNCServo {
	public:
		CNCServo(int _timer, float _rampRate, int _direction, float _encoderPPR, float multiplier);
		~CNCServo();
		void SetSpeed(float speedSS, long timeInterval, float syncMod);
		void SetRampRate(float _rampRate);
		void SetEncoderPPR(float _encoderPPR);
		void ClearEncoderCount();
		long GetEncoderCount();
		float GetEncoderTickRate();
		float GetRampDownDistance();
		float GetRampDownTicks();
		float GetPulseFrequency();
		float GetSpeedCommand();
		int GetState();
		int GetOCRAVal();
		void UpdateEncoderCount(long dt);
		void SetEncoderCount(long newValue);
	private:
		int stepPin;
		int dirPin;
		int direction;		//1 for high dirSignal when spd>0, -1 for opposite
		bool previousDir;
		int state;		//state of rampMotor() FSM
		float rampRate;		//acceleration rate, rpm/s
		float speedCMD;		//current commmanded motor speed, rpm
		long encoderCount;	//distance traveled in encoder ticks
		long prev_encoderCount;
		float encoderTickRate;	// encoder ticks/sec
		int slaveSelectPin;	// chip select pin for encoder
		float rampDownTicks;	//predicted distance to travel during ramp (enc ticks)
		float fpulseACT;	//output pulse frequency
		float geckodriveMultiplier;	//geckodrive dip switch setting freq gain
		float prev_speedACT;	//previous actual speed in rev/s
		float encoderPPR;	//encoder pulses per shaft revoluation
		unsigned int ocraVal;		//value written to OCRA in timing register
		int timer;		//timer to use for PWM output
		bool pwmEnabled;
		float RampMotor(float speedSS, unsigned long dt, float syncMod);
		void InitTimerAndPins();
		void SetOutputs(float speedCMD);
		void InitEncoder();
		long ReadEncoder();
};

#endif
