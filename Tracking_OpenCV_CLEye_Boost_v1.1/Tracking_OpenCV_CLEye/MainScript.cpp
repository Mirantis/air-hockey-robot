
#include "stdafx.h"
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "boost/thread/thread.hpp"
#include <iostream>
#include <fstream>
#include <time.h>
#include "PuckTracker.h";
#include "ArduinoSerialInterface.h"

using namespace cv;
using namespace std;

#pragma region Defines
PVOID latency = NULL;
#define OUTPUT_WINDOW	"Output Window"

// PS3 Camera pixels
#define CAM_PIX_WIDTH 320
#define CAM_PIX_HEIGHT 240
#define numberOfCritXvalues 5
#define PI 3.14159265358979323846
#define YDEF_HIGH 29
#define YDEF_LOW 10
#define TDEF_MIN 0.02
#define YATT_HIGH 33
#define YATT_LOW 3.45
#define TATT_MIN 0.33
#pragma endregion

#pragma region Variables
// Variables //
int RminH=5, RmaxH=20, RminS=110, RmaxS=200, RminV=90, RmaxV=200;

// Thread Lock //
boost::mutex mtx;

// OpenCV and Camera Capture Vars //
Mat imgCapture, imgLines;
bool displayEnabled = true;		//enable to show camera output -- slows down compute cycle time significantly
bool loggingEnabled = false;	//enable to log data to asdf.txt
bool cameraUpdated = false;

// testing
double globalArea = 0, globalRoundness = 0;
int param1 = 1, param2 = 1, blurSize = 9;

// fps check
long oldFrameTimestamp = 0, frameTimestamp = 0;

#pragma endregion

#pragma region "Camera Capture"
// Camera capture class -- runs on its own thread and polls the camera for image updates
class CLEyeCapture
{
	GUID _cameraGUID;
	CLEyeCameraInstance _cam;
	CLEyeCameraColorMode _mode;
	CLEyeCameraResolution _resolution;
	int _fps;
	HANDLE _hThread;
	bool _running;
	double measuredCnt;
	bool _isColor;
	
public:
	double tMin, tMax, tAvg, curr;
public:
	CLEyeCapture(CLEyeCameraResolution resolution, CLEyeCameraColorMode mode, int fps) :
	  _resolution(resolution), 
		  _mode(mode), 
		  _fps(fps), 
		  _running(false)
	  {
		  _cameraGUID = CLEyeGetCameraUUID(0);
		  if(_mode == CLEYE_COLOR_PROCESSED || _mode == CLEYE_COLOR_RAW)
			  _isColor = true;
		  else
			  _isColor = false;

	  }
		bool Start()
		{
			_running = true;
			// Start CLEyeCapture image capture thread
			_hThread = CreateThread(NULL, 0, &CLEyeCapture::CaptureThread, this, 0, 0);
			if(_hThread == NULL)
			{
				//MessageBox(NULL,"Could not create capture thread","CLEyeCapture", MB_ICONEXCLAMATION);
				cout << "Could not create capture thread" << endl;
				return false;
			}
			return true;
		}
		void Stop()
		{
			if(!_running)	return;
			_running = false;
			WaitForSingleObject(_hThread, 2000);
		}
		void Run()
		{//cout << "here0" << endl;
			int w, h;
			
			// Create camera instances
			_cam = CLEyeCreateCamera(_cameraGUID, _mode, _resolution, _fps);
			if(_cam == NULL)	return;

			// Get camera frame dimensions
			CLEyeCameraGetFrameDimensions(_cam, w, h);

			IplImage *pCapImage;

			// Create the OpenCV images
			pCapImage = cvCreateImage(cvSize(w, h), IPL_DEPTH_8U, _isColor ? 4 : 1);

			// Set some camera parameters
			CLEyeSetCameraParameter(_cam, CLEYE_GAIN, 0);
			CLEyeSetCameraParameter(_cam, CLEYE_EXPOSURE, 511);

			// Start capturing
			CLEyeCameraStart(_cam);
			Sleep(100);

			tMin = 1000; 
			tMax = 0;
			tAvg = 0;
			curr = 0;
			measuredCnt = 0;
			printf("\n");

			//cout << "here1" << endl;

			// Capture camera images
			PBYTE tempBuffer;
			cvGetImageRawData(pCapImage, &tempBuffer);
			CLEyeCameraGetFrame(_cam, tempBuffer);
			Mat imgTemp(pCapImage);
			
			mtx.lock();
			imgLines = Mat::zeros( imgTemp.size(), CV_8UC4 );
			imgCapture = Mat::zeros( imgTemp.size(), CV_8UC4 );
			mtx.unlock();

			//cout << "here2" << endl;

			// image capturing loop
			while(_running)
			{
				//oldFrameTimestamp = frameTimestamp;
				//oldFrameTimestamp = getTickCount();
				//cout << "here3" << endl;

				PBYTE pCapBuffer;

				// Capture camera images
				cvGetImageRawData(pCapImage, &pCapBuffer);
				CLEyeCameraGetFrame(_cam, pCapBuffer);

				//IplImage* imgHSV = cvCreateImage(cvGetSize(pCapImage), IPL_DEPTH_8U, 3); 
				//cvCvtColor(pCapImage, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
				//IplImage* imgThresholded = cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
				//cvInRangeS(imgHSV, cvScalar(minH,minS,minV), cvScalar(maxH,maxS,maxV), imgThresholded);
				
				Mat imgCap(pCapImage);		//creates 8UC4 Mat

				mtx.try_lock();
				imgCap.copyTo(imgCapture);	//copy to shared variable
				cameraUpdated = true;		//indicate that image was updated
				mtx.unlock();

				//frameTimestamp = getTickCount();
				//double t = ((double)(frameTimestamp - oldFrameTimestamp))/getTickFrequency() * 1000;
				//cout << "Frame interval: " << t << endl;
			}
			Sleep(1000);
			// Stop camera capture
			CLEyeCameraStop(_cam);
			// Destroy camera object
			CLEyeDestroyCamera(_cam);
			// Destroy the allocated OpenCV image
			cvReleaseImage(&pCapImage);
			_cam = NULL;
	  }
	  static DWORD WINAPI CaptureThread(LPVOID instance)
	  {
		  // seed the RNG with current tick count and thread id
		  srand(GetTickCount() + GetCurrentThreadId());
		  // forward thread to Capture function
		  CLEyeCapture *pThis = (CLEyeCapture *)instance;
		  pThis->Run();
		  return 0;
	  }
};
#pragma endregion

// used for filtered values, check if value is in range before adding to filter
float checkBounds(float checkVal, float filterVal, float lowerBound, float upperBound, int numFiltPoints)
{
	bool outOfRange = checkVal < lowerBound || checkVal > upperBound;
	if(outOfRange && numFiltPoints > 1) {
		return filterVal/(numFiltPoints-1);
	}
	else {
		return checkVal;
	}
}

int _tmain(int argc, _TCHAR* argv[])
{
	// Launch camera polling thread //
	CLEyeCapture *cam = NULL;

	// Query for number of connected cameras
	int numCams = CLEyeGetCameraCount();
	if(numCams == 0) {
		printf("No PS3Eye cameras detected\n");
		Sleep(1000);
		return -1;
	}
	
	// Open serial port
	ArduinoSerialInterface asInterface = *new ArduinoSerialInterface(L"\\\\.\\COM4");
	bool opened = asInterface.openComPort();
	if(!opened) {
		cout << "Failed to open serial port!" << endl;
		Sleep(1000);
		return -1;
	}
	
	// Initialize camera output windows
	if(displayEnabled) {
		cvNamedWindow(OUTPUT_WINDOW, CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Contours",CV_WINDOW_AUTOSIZE);
	}

	// start capture
	cam = new CLEyeCapture(CLEYE_QVGA, (CLEyeCameraColorMode)CLEYE_COLOR_PROCESSED, 100);
	cam->Start();

	// logging
	ofstream myfile;
	if(loggingEnabled) { myfile.open("asdf.txt"); }
	Sleep(2000);	//wait for camera to initialize -- this could end badly

	// puck HSV bounds
	PuckTracker::HSVBounds pbounds;
	pbounds.minH=21; pbounds.maxH=72;
	pbounds.minS=47; pbounds.maxS=106;
	pbounds.minV=163; pbounds.maxV=255;

	// robot HSV bounds
	PuckTracker::HSVBounds rbounds;
	rbounds.minH=0; rbounds.maxH=28;
	rbounds.minS=123; rbounds.maxS=211;
	rbounds.minV=160; rbounds.maxV=255;

	// Initialize puckTracker class -- maintains puck state variables
	float rPuck = 1.60, rPaddle = 1.99;	//paddle radius, inches
	PuckTracker puckTracker(rPuck, pbounds);
	PuckTracker robotTracker(rPuck, rbounds);

	int colorCycler = 0;
	
	// set up local state variables to point to pucktracker vars
	//float xPC, yPC, xPT, yPT, vPx, vPy;
	float &xPC = puckTracker.xPC, &yPC = puckTracker.yPC, &xPT = puckTracker.xPT, &yPT = puckTracker.yPT;
	float &vPx = puckTracker.vPx, &vPy = puckTracker.vPy;

	// x,y,t are target defense position (defense) or target puck position (attack). angleTarget is angle of attack
	float xDef = 0, yDef = 0, tDef = 0, xAtt = 0, yAtt = 0, tAtt = 0, angleAtt = 0;
	float critXvalues[] = { 23, 15, 12, 10, 3};		//x locations at which trajectory is evaluated for AI decision-making
	float critYvalues[numberOfCritXvalues], critTvalues[numberOfCritXvalues], critVyvalues[numberOfCritXvalues];
	float tAttThreshold = 0.33, tDefThreshold = 0.02;	//minimum required value of tAtt to send attack command
	bool arduinoReady = false, attInRange = false, defInRange = false, recenterLock = false, centerHomeY = false;
	bool filtPointsReqSet = false, ricochetDetected = false;
	int filtPoints = 0, filtPointsReq = 4;
	long prevCommandTime = 0, prevState1Time = 0;
	int vPxHighCounter = 0, vPxNegCounter = 0, retransmitCounter = 0;
	float loopTime = 0, timeInState1 = 0, state1Timeout = 1;	//seconds
	// object tracking //
	// towards: puck moving towards robot, away: puck moving away from robot, decision: latest x-value at which attack/defense must be picked
	float xThreshold_towards = 50, xThreshold_away = 30, xThreshold_decision = 35;
	float xPTfilt = 0, yPTfilt = 0, vPxfilt = 0, vPyfilt = 0;

	int state = 0, prev_state = -1;
	/*
	enum state {
		IDLE = 0;
		PUCK_TWD_ROBOT = 1;
		PUCK_TWD_PLAYER = 2;
	};
	*/
	while(true) {
		// Copy image from camera thread locally
		Mat imgOriginal;
		mtx.lock();
		if(!cameraUpdated) { mtx.unlock(); continue; }	//go to next loop iteration if camera hasn't updated the image
		imgCapture.copyTo(imgOriginal);
		cameraUpdated = false;
		mtx.unlock();

		// check com port for updates
		asInterface.readComPort(state);

		//extract puck position from image, maintain state variables
		int trackStatus = puckTracker.UpdatePuckState(imgOriginal);
		int robotStatus = robotTracker.UpdatePuckState(imgOriginal);
		bool positionUpdated = trackStatus & 1 == 1, velocityUpdated = trackStatus & 2 == 2;
		//if(!(trackStatus & 1 == 1)) { continue; }	//go to next iteration if position wasn't updated

		//if(state == 1) { cout << xPT << "," << yPT << "\t" << vPx << "\t" << vPy << endl; }
		//cout << positionUpdated << "," << velocityUpdated << "\t" << puckTracker.xPC << "," << puckTracker.yPC;
		//cout << "\t" << puckTracker.xPT << "," << puckTracker.yPT << endl;

		//predict trajectory
		Mat imgTrajectory = Mat::zeros( imgOriginal.size(), CV_8UC4 );

		vPxHighCounter = vPx > 1 ? vPxHighCounter + 1 : 0;
		vPxNegCounter = vPx < -1 ? vPxNegCounter + 1 : 0;

		#pragma region "State Machine"
		int next_state = state;
		switch(state) {
			case 0: {	//idle -- default state
				//entry
				if(prev_state != 0) { cout << "Entered state 0" << endl; }

				// state actions -- check if arduino is ready //
				//if(!arduinoReady) { arduinoReady = __; }
				arduinoReady = true;

				// state transitions -- wait for puck to come towards robot //
				next_state = arduinoReady && vPxNegCounter > 1 && xPT <= xThreshold_towards ? 1 : 0;	//transition to receive puck
				//next_state = xPT >= xThreshold_away && vPxHighCounter > 2 && !recenterLock ? 3 : next_state;	//recenter if puck moving away
				next_state = vPxHighCounter > 0 && !recenterLock ? 3 : next_state;	//recenter if puck moving away

				// exit actions
				if(next_state == 1) {
					//cout << "State 0: " << xPT << "," << yPT << "\t" << vPx << "\t" << vPy << endl;
					puckTracker.PredictPuckTrajectory(&imgTrajectory, displayEnabled, numberOfCritXvalues, critXvalues,
						critYvalues, critTvalues, critVyvalues);
					//cout << "State 0: " << xPT << "," << yPT << "\t" << vPx << "\t" << vPy << endl;
				}
				break;
			}
			case 1: {	//puck moves player -> robot: decide between attack or defense
				// entry -- clear filter values
				if(prev_state != 1) {
					cout << "Entered state 1" << endl;
					cout << xPT << "," << yPT << "," << vPx << "," << vPy << "\t";
					cout << puckTracker.prev_xPT << "," << puckTracker.prev_yPT << endl;
					xPTfilt = 0; yPTfilt = 0; vPxfilt = 0; vPyfilt = 0; filtPoints = 0;
					xDef = 0; yDef = 0; tDef = 0; xAtt = 0; yAtt = 0; tAtt = 0;
					recenterLock = false; timeInState1 = 0;
				}
				bool decide = false, noMoves = false;
				if(velocityUpdated) {	//only proceed if we got a position/velocity update
					// adjust required number of filter points based on x-velocity
					if(!filtPointsReqSet) {
						if(vPx < -350) { filtPointsReq = 2; }
						else if(vPx < -200) { filtPointsReq = 3; }
						else { filtPointsReq = 4; }
						filtPointsReqSet = true;
					}
					filtPoints++;

					// state actions -- predict puck trajectory, decide on attack/defense when conditions met
					puckTracker.PredictPuckTrajectory(&imgTrajectory, displayEnabled, numberOfCritXvalues, critXvalues,
						critYvalues, critTvalues, critVyvalues);

					// check time at critXvalues[1] -- attack point
					xAtt = critXvalues[1]; yAtt += critYvalues[1]; tAtt += critTvalues[1];
				
					// check y, t values before adding to filter
					//yAtt += checkBounds(critYvalues[1], yAtt, YATT_HIGH, YATT_LOW, filtPoints);
					//tAtt += checkBounds(critTvalues[1], tAtt, 0, 100, filtPoints);
					
					// determine command point for defense
					int idx = numberOfCritXvalues - 1;
					xDef = critXvalues[idx]; yDef += critYvalues[idx]; tDef += critTvalues[idx];
				
					//yDef += checkBounds(critYvalues[idx], yDef, yBoundLow, yBoundHigh, filtPoints);
					//tDef += checkBounds(critTvalues[idx], tDef, 0, 100, filtPoints);
	
					// compute attack and defend points + times, randomize attack angle
					decide = filtPoints >= filtPointsReq;// || (xPT <= xThreshold_decision && filtPoints > 1);

					//cout << xDef << "," << yDef << "," << tDef << "\t" << xAtt << "," << yAtt << "," << tAtt << endl;
					cout << xDef << "," << critYvalues[idx] << "," << critTvalues[idx] << "\t";
					cout << xAtt << "," << critYvalues[1] << "," << critTvalues[1] << "\t";
					cout << vPx << "," << vPy << endl;

					noMoves = false;
					if(decide) {
						//send once here for fastest response, continuously send after state 2 transition
						yDef /= filtPoints; tDef /= filtPoints;
						yAtt /= filtPoints; tAtt /= filtPoints;
					
						//if puck is coming at an angle, offset defense point by paddle radius
						//if(critVyvalues[idx]/vPx < -0.33) { yDef += rPaddle; }
						//else if(critVyvalues[idx]/vPx > 0.33) { yDef -= rPaddle; }
						//if(critVyvalues[idx]/vPx < -0.33) { yDef -= rPaddle; }
						//else if(critVyvalues[idx]/vPx > 0.33) { yDef += rPaddle; }

						// make sure commands are within bounds
						defInRange = yDef >= YDEF_LOW && yDef <= YDEF_HIGH && tDef > TDEF_MIN;
						attInRange = yAtt > YATT_LOW && yAtt < YATT_HIGH && tAtt > TATT_MIN;

						if(attInRange) { asInterface.sendAttackCommand(xAtt,yAtt,tAtt); }
						else if(defInRange) { asInterface.sendDefendCommand(xDef,yDef,tDef); }
						else { noMoves = true; }	//both out of range, give up
						prevCommandTime = getTickCount();
					}
				}
				timeInState1 += loopTime;
				cout << "Time: " << timeInState1 << endl;

				// state transitions
				next_state = timeInState1 > state1Timeout ? 0 : 1;	//timeout after 5 seconds to avoid getting stuck waiting for more points
				next_state = decide ? 2 : next_state;	//retransmit attack or defense command
				next_state = decide && vPxfilt > 1 || noMoves ? 0 : next_state;
				//next_state = xPT >= xThreshold_away && vPx > 1 ? 3 : next_state;
				next_state = vPxHighCounter > 0 && !recenterLock ? 3 : next_state;

				if(next_state != 1) { filtPointsReqSet = false; }
				break;
			}
			case 2: {	//retransmitting defense/attack routine until acked
				// entry
				if(prev_state != 2) {
					cout << "Entered state 2" << endl;
					cout << xPT << "," << yPT << "," << vPx << "," << vPy << "\t" << xDef << "," << yDef << "," << tDef;
					cout << "\t" << xAtt << "," << yAtt << "," << tAtt << "," << filtPoints << endl;
					//if(tTarget1 <= 0) { next_state = 0; return; }
					retransmitCounter = 3;
				}

				// subtract elapsed time from target time
				long currentTime = getTickCount();
				float timeDiff = ((float)(currentTime - prevCommandTime))/getTickFrequency();	//seconds
				timeDiff = timeDiff < 0 ? 0.015 : timeDiff;	//assume 15 ms interval if overflow occurs
				tDef -= timeDiff; tDef = tDef < 0 ? 0 : tDef;
				tAtt -= timeDiff; tAtt = tAtt < 0 ? 0 : tAtt;

				// state actions -- send arduino packet with defend + attack points
				/*
				if(tDef > 0.00001 && !defAttCmdAcked) {
					prevCommandTime = getTickCount();
					sendDefendAttackCommand(xDef,yDef,tDef,xAtt,yAtt,tAtt,angleAtt);
				}
				*/
				
				//retransmit until command is acknowledged
				if(attInRange && tAtt >= tAttThreshold && !asInterface.attackCmdAcked && retransmitCounter > 1) {	//send attack command if there's sufficient time
					asInterface.sendAttackCommand(xAtt,yAtt,tAtt);
					asInterface.attackCmdAcked = false;
				}
				else if(defInRange && tDef >= tDefThreshold && !asInterface.defenseCmdAcked && retransmitCounter > 1) {	//send defense command otherwise
					asInterface.sendDefendCommand(xDef,yDef,tDef);
					asInterface.defenseCmdAcked = false;
				}
				prevCommandTime = getTickCount();
				retransmitCounter = retransmitCounter > 1 ? 0 : retransmitCounter++;	//avoid swamping arduino

				// transitions
				next_state = tDef < tDefThreshold || asInterface.attackCmdAcked || asInterface.defenseCmdAcked ? 0 : 2;
				//next_state = xPT >= xThreshold_away && vPx > 1 ? 3 : next_state;
				next_state = vPxHighCounter > 0 && !recenterLock ? 3 : next_state;
				if(next_state != 2) { asInterface.defAttCmdAcked = false; asInterface.attackCmdAcked = false; asInterface.defenseCmdAcked = false; }
				break;
			}
			case 3: { //puck moves robot -> player: recenter if time is available
				if(prev_state != 3) {
					cout << endl << "Entered state 3" << endl << endl;
					asInterface.recenterCmdAcked = false;
					centerHomeY = vPx > 1 && vPx < 175;	//decide whether shot is slow enough to centerhome y-axis
					retransmitCounter = 3;
				}
				
				if(recenterLock) { next_state = 0; break; }

				if(!asInterface.recenterCmdAcked) {
					cout << "Retransmitting..." << endl;
					asInterface.sendRecenterCommand(centerHomeY);
				}
				
				next_state = asInterface.recenterCmdAcked ? 0 : 3;
				if(next_state != 3) { recenterLock = true; }
				break;
			}
			case 4: {	//goal scored
				//testing
				//predictPuckTrajectory(&imgTrajectory, critYvalues, critTvalues);

				break;
			}
			default: {

			
				break;
			}
		}
		prev_state = state;
		state = next_state;
		#pragma endregion

		//cout << state << "\t" << xPT << "," << yPT << "\t" << xPC << "," << yPC << endl;
		if(loggingEnabled) { myfile << xPT << "," << yPT << "\t" << vPx << "\t" << globalArea << "," << globalRoundness << endl; }

		//bump output to display window, check for key press
		//cvWaitKey() waits for ~10 ms even if a value less than 10 is given -- blame Windows
		if(displayEnabled) {
			// draw puck tracking lines
			//if(initCounts > initCountsNeeded && prev_xPC >= 0 && prev_yPC>=0 && xPC>=0 && yPC>=0) {
				/*
				if(colorCycler == 0) {
					line(imgLines, Point(xPC, yPC), Point(prev_xPC, prev_yPC), Scalar(0,0,255), 2);
				}
				else if(colorCycler == 1) {
					line(imgLines, Point(xPC, yPC), Point(prev_xPC, prev_yPC), Scalar(255,0,0), 2);
				}
				else if(colorCycler == 2) {
					line(imgLines, Point(xPC, yPC), Point(prev_xPC, prev_yPC), Scalar(0,255,0), 2);
				}
				colorCycler++; if(colorCycler > 2) { colorCycler = 0; }
				*/
			mtx.try_lock(); 
			line(imgLines, Point(xPC, yPC), Point(puckTracker.prev_xPC, puckTracker.prev_yPC), Scalar(0,0,255), 2); 
			cout << robotTracker.xPC << "," << robotTracker.yPC << "\t" << robotTracker.prev_xPC << "," << robotTracker.prev_yPC;
			line(imgLines, Point(robotTracker.xPC, robotTracker.yPC), Point(robotTracker.prev_xPC, robotTracker.prev_yPC), Scalar(0,255,0), 2); 
			mtx.unlock();
				//cout << "lines added" << endl;
			//}

			imgOriginal = imgOriginal + imgLines + imgTrajectory;
			imshow(OUTPUT_WINDOW, imgOriginal); //show the original image
			//imshow("Contours",imgThresholded);
		
			int keyPress = cvWaitKey(1);
			//cout << "Pressed: " << keyPress << endl;
			if (keyPress == 27) { //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
				cout << "esc key is pressed by user" << endl;
				break;
			}
			else if(keyPress == 32) { //empty out lines
				imgLines = Mat::zeros( imgLines.size(), CV_8UC4 );
			}
		}

		frameTimestamp = getTickCount();
		loopTime = ((float)(frameTimestamp - oldFrameTimestamp))/getTickFrequency();
		//cout << "Frame interval: " << t << endl;

		oldFrameTimestamp = getTickCount();
	}
	//myfile.close();
	cam -> Stop();
	cvDestroyWindow(OUTPUT_WINDOW);
	return 0;
}
