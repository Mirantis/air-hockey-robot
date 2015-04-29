#include "stdafx.h"
#include <iostream>
#include <fstream>
//#include <WinNT.h>

using namespace std;

class ArduinoSerialInterface
{
public:
	ArduinoSerialInterface(LPCWSTR port);
	~ArduinoSerialInterface();
	bool openComPort();
	bool readComPort(int cvState);
	bool writeComPort(unsigned char *message, int length);
	bool sendDefendCommand(float xT1, float yT1, float tT1);
	bool sendAttackCommand(float xP2, float yP2, float tT2);
	bool sendRecenterCommand(bool centerHomeY);
	bool sendDefendAttackCommand(float xD, float yD, float tD, float xA, float yA, float tA, float aA);
	bool defenseCmdAcked, attackCmdAcked, defAttCmdAcked, recenterCmdAcked;
private:
	void arrayCopy(unsigned char *destArray, int destIndex, unsigned char *srcArray, int length);
	HANDLE serialPort;
	LPCWSTR portSpecifier;	//LPCWSTR: pointer to string of 2-byte chars
	unsigned int startDefenseCmd;	//command header telling arduino to defend a point
	unsigned int startAttackCmd;	//command header telling arduino to attack a point
	unsigned int defendAttackCmd;	//command header for packet containing defense and attack point
	unsigned int recenterCmd;		//command telling arduino to recenter paddle -- command to (4,18)
};