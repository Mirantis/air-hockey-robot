#include "ArduinoSerialInterface.h"

// port must be in a specific format if greater than COM9, e.g. L"\\\\.\\COM41"
ArduinoSerialInterface::ArduinoSerialInterface(LPCWSTR port) {
	portSpecifier = port;	//LPCWSTR: pointer to string of 2-byte chars
	startDefenseCmd = 'sd';	//command header telling arduino to defend a point
	startAttackCmd = 'sa';	//command header telling arduino to attack a point
	defendAttackCmd = 'da';	//command header for packet containing defense and attack point
	recenterCmd = 'rc';		//command telling arduino to recenter paddle -- command to (4,18)
	defenseCmdAcked = false; attackCmdAcked = false; defAttCmdAcked = false; recenterCmdAcked = false;
}

ArduinoSerialInterface::~ArduinoSerialInterface() {}

bool ArduinoSerialInterface::openComPort()
{
	DCB dcb;
	serialPort = CreateFile(portSpecifier,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
	
	if (!GetCommState(serialPort,&dcb))
		return(false);

	// Serial port configuration
	//dcb.BaudRate = CBR_57600;
	dcb.BaudRate = CBR_115200;
	dcb.ByteSize = 8;
	dcb.Parity = NOPARITY;
	dcb.StopBits = ONESTOPBIT;
	dcb.fDtrControl = DTR_CONTROL_DISABLE;
	if (!SetCommState(serialPort,&dcb))
		return(false);

	if(serialPort == INVALID_HANDLE_VALUE) { cout << "Serial port handle invalid!" << endl; }
}

// Read from COM PORT and output to console
bool ArduinoSerialInterface::readComPort(int cvState)
{
	DWORD dwRead;
	char buffer[4096];
	DWORD temp; 
	COMSTAT comstat;
	BOOL readResult;

	// Get Serial stats
	ClearCommError(serialPort,&temp,&comstat);

	// New bytes pending read?
	if (comstat.cbInQue > 6)
	{
		//cout << "Reading " << comstat.cbInQue << " bytes." << endl;
		ReadFile(serialPort, buffer, comstat.cbInQue, &dwRead, NULL);
		//float a = *((float*)&buffer[0]);
		//float b = *((float*)&buffer[4]);
		//cout << a << "," << b << endl;
		if(buffer[0] == 'a') {	//ack
			unsigned int ackedCmd = buffer[1] << 8 | buffer[2];
			unsigned int packetChecksum = buffer[3] << 8 | buffer[4];
			unsigned char sync1 = buffer[5], sync2 = buffer[6];
			
			//verify checksum
			unsigned int checksum = 'a' + buffer[1] + buffer[2];
			if(checksum != packetChecksum) { return false; }
			
			//verify sync bytes
			if(sync1 != 0x08 || sync2 != 0x09) { return false; }

			//identify command being acked
			if(ackedCmd == startDefenseCmd && cvState == 2)		{ defenseCmdAcked = true; }
			else if(ackedCmd == startAttackCmd && cvState == 2)	{ attackCmdAcked = true; }
			else if(ackedCmd == defendAttackCmd && cvState == 2)	{ defAttCmdAcked = true; }
			else if(ackedCmd == recenterCmd && cvState == 3)	{ recenterCmdAcked = true; }
		}
	}
	return true;
}

bool ArduinoSerialInterface::writeComPort(unsigned char *message, int length)
{
	DWORD byteswritten;

	bool retVal = WriteFile(serialPort,message,length,&byteswritten,NULL);
	return retVal;
}

// copy contents of srcArray into destArray at specified index
void ArduinoSerialInterface::arrayCopy(unsigned char *destArray, int destIndex, unsigned char *srcArray, int length)
{
	for(int i=0; i<length; i++)
		destArray[destIndex + i] = srcArray[i];
}

// params target x, target y, target time
bool ArduinoSerialInterface::sendDefendCommand(float xT1, float yT1, float tT1)
{
	// build command
	unsigned char message[18];
	message[0] = 's'; message[1] = 'd';

	unsigned char* xArray = (unsigned char*)(&xT1);
	unsigned char* yArray = (unsigned char*)(&yT1);
	unsigned char* tArray = (unsigned char*)(&tT1);
	arrayCopy(message,2,xArray,4); arrayCopy(message,6,yArray,4); arrayCopy(message,10,tArray,4);

	// compute checksum, add to array
	unsigned int checksum = 0;
	for(int i=2;i<18-4; i++) { checksum += message[i]; }
	unsigned char* chkArray = (unsigned char*)(&checksum);
	arrayCopy(message,14,chkArray,2);

	// add suffix
	message[16] = 0x08; message[17] = 0x09;
	defenseCmdAcked = false;

	return writeComPort(message,18);
}

// inputs: x, y puck at attack point, time puck is at point, angle of attack
bool ArduinoSerialInterface::sendAttackCommand(float xP2, float yP2, float tT2)
{
	// build command
	unsigned char message[18];
	message[0] = 's'; message[1] = 'a';

	unsigned char* xArray = (unsigned char*)(&xP2);
	unsigned char* yArray = (unsigned char*)(&yP2);
	unsigned char* tArray = (unsigned char*)(&tT2);
	arrayCopy(message,2,xArray,4); arrayCopy(message,6,yArray,4); arrayCopy(message,10,tArray,4);

	// compute checksum, add to array
	unsigned int checksum = 0;
	for(int i=2;i<18-4; i++) { checksum += message[i]; }
	unsigned char* chkArray = (unsigned char*)(&checksum);
	arrayCopy(message,14,chkArray,2);

	// add suffix
	message[16] = 0x08; message[17] = 0x09;
	attackCmdAcked = false;

	return writeComPort(message,18);
}

bool ArduinoSerialInterface::sendRecenterCommand(float xRobotT, float yRobotT, bool centerHomeY)
{
	unsigned char message[15];
	message[0] = 'r'; message[1] = 'c';
	
	unsigned char* xrArray = (unsigned char*)(&xRobotT);
	unsigned char* yrArray = (unsigned char*)(&yRobotT);

	arrayCopy(message,2,xrArray,4); arrayCopy(message,6,yrArray,4);
	message[10] = centerHomeY;

	unsigned int checksum = 0;
	for(int i=2;i<15-4;i++) { checksum += message[i];  }
	unsigned char* chkArray = (unsigned char*)(&checksum);
	arrayCopy(message,11,chkArray,2);
	message[13] = 0x08; message[14] = 0x09;

	return writeComPort(message,15);
}

// inputs: defense point and time, attack point and time and angle
bool ArduinoSerialInterface::sendDefendAttackCommand(float xD, float yD, float tD, float xA, float yA, float tA, float aA)
{
	// build command
	unsigned char message[34];
	message[0] = 'd'; message[1] = 'a';

	unsigned char* xdArray = (unsigned char*)(&xD);
	unsigned char* ydArray = (unsigned char*)(&yD);
	unsigned char* tdArray = (unsigned char*)(&tD);
	unsigned char* xaArray = (unsigned char*)(&xA);
	unsigned char* yaArray = (unsigned char*)(&yA);
	unsigned char* taArray = (unsigned char*)(&tA);
	unsigned char* aaArray = (unsigned char*)(&aA);
	arrayCopy(message,2,xdArray,4); arrayCopy(message,6,ydArray,4); arrayCopy(message,10,tdArray,4);
	arrayCopy(message,14,xaArray,4); arrayCopy(message,18,yaArray,4); arrayCopy(message,22,taArray,4);
	arrayCopy(message,26,aaArray,4);

	// compute checksum, add to array
	unsigned int checksum = 0;
	for(int i=2;i<34-4;i++) { checksum += message[i]; }
	unsigned char* chkArray = (unsigned char*)(&checksum);
	arrayCopy(message,30,chkArray,2);

	// add suffix
	message[32] = 0x08; message[33] = 0x09;
	defAttCmdAcked = false;

	return writeComPort(message,34);
}