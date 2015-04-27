//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This file is part of CL-EyeMulticam SDK
//
// C++ CLEyeLatency Sample Application
//
// For updates and file downloads go to: http://codelaboratories.com
//
// Copyright 2008-2012 (c) Code Laboratories, Inc. All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "stdafx.h"
#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "boost/thread/thread.hpp"
#include <iostream>
#include <fstream>
#include <time.h>

using namespace cv;
using namespace std;

#pragma region Defines
PVOID latency = NULL;
#define OUTPUT_WINDOW	"Output Window"

// PS3 Camera pixels
// NOTE: We are using the camera at 320x240 but we are outputing a 640x480 pixel position
/*
#define CAM_PIX_WIDTH 640
#define CAM_PIX_HEIGHT 480
#define CAM_PIX_TO_IN_X 0.1
#define CAM_PIX_TO_IN_Y 0.0978
*/
#define CAM_PIX_WIDTH 320
#define CAM_PIX_HEIGHT 240
//#define CAM_PIX_TO_IN_X 0.2032
#define CAM_PIX_TO_IN_X 0.1942
#define CAM_PIX_TO_IN_Y 0.1942
#define XC0 323			//pixel value at which x=0
#define YC0 220			//pixel value at which y=0

#define numberOfCritXvalues 5
#define PI 3.14159265358979323846

#pragma endregion

#pragma region Variables
// Variables //

// Thread Lock //
boost::mutex mtx;

// Camera variables
int cam_center_x;
int cam_center_y;
float cam_pix_to_in;
float cam_rotation;		//Camera rotation relative to table frame in radians
//float deltaXct = -24.152;		//distance between camera and table center along x axis in inches -- (xCameraOrigin - xTableOrigin)
//float deltaYct = -23.0244;	//distance between camera and table center along y axis in inches -- (yCameraOrigin - yTableOrigin)
float deltaXct = 0;		//distance between camera and table center along x axis in inches -- (xCameraOrigin - xTableOrigin)
float deltaYct = 0;	//distance between camera and table center along y axis in inches -- (yCameraOrigin - yTableOrigin)
//float deltaXct = 0.2406;		//distance between camera and table center along x axis in inches -- (xCameraOrigin - xTableOrigin)
//float deltaYct = -3.8376;	//distance between camera and table center along y axis in inches -- (yCameraOrigin - yTableOrigin)

/*
// Thresholding Parameters -- red puck
int minH=0; int maxH=29;
int minS=139; int maxS=255;
int minV=0; int maxV=255;
int RminH=5; int RmaxH=20;
int RminS=110; int RmaxS=200;
int RminV=90; int RmaxV=200;
*/

// Thresholding Parameters -- red puck
int minH=28; int maxH=72;
int minS=44; int maxS=96;
int minV=159; int maxV=202;

// OpenCV Objects
Mat imgLines;
Mat imgCapture;
bool displayEnabled = false;	//enable to show camera output -- slows down compute cycle time significantly
bool loggingEnabled = false;				//enable to log data to asdf.txt

// object tracking
float xPC, yPC, xPT, yPT;							//puck coords in camera frame (C) and table frame (T)
float prev_xPC = -1, prev_yPC = -1, prev_xPT = -1, prev_yPT = -1, prev2_xPT = -1, prev2_yPT = -1;		//puck coords from previous timestep
int lastX = -1, lastY = -1;
int status;
int objectSize;
int initCountsNeeded = 10, initCounts = 0;
//float xBoundLow = -22.25, xBoundHigh = 39.75;				//camera vision x boundaries
//float yBoundLow = -18.145, yBoundHigh = 18.145;				//table y boundaries
float xBoundLow = 0 + 1.60, xBoundHigh = 62 - 1.60;				//table x boundaries, high is camera vision bound
float yBoundLow = 0 + 1.60, yBoundHigh = 39.25 - 1.60; //yBoundHigh = 36.29 - 1.60;				//table y boundaries
// towards: puck moving towards robot, away: puck moving away from robot, decision: latest x-value at which attack/defense must be picked
float xThreshold_towards = 50, xThreshold_away = 30, xThreshold_decision = 35;
float criticalXvalues[] = { 23, 15, 12, 10, 3};		//x locations at which trajectory is evaluated for AI decision-making
float vPx = 0, vPy = 0, prev_vPx = 0, prev_vPy = 0;
float accelPx = 0, accelPy = 0;
long oldPositionTimestamp = -1;
int state = 0, prev_state = -1;
float xR = 0, yR = 0;	//robot coords, received from arduino
float rPuck = 1.60, rPaddle = 1.99;	//paddle radius, inches
bool velocityUpdated = false, ricochetDetected = false;
int updateVelocityCounter = 0;

// serial port
HANDLE serialPort;
// COM port must be specified this way for serial port to be opened properly
LPCWSTR portSpecifier = L"\\\\.\\COM41";	//LPCWSTR: pointer to string of 2-byte chars
unsigned int startDefenseCmd = 'sd';	//command header telling arduino to defend a point
unsigned int startAttackCmd = 'sa';	//command header telling arduino to attack a point
unsigned int defendAttackCmd = 'da';	//command header for packet containing defense and attack point
unsigned int recenterCmd = 'rc';		//command telling arduino to recenter paddle -- command to (4,18)
bool defenseCmdAcked = false, attackCmdAcked = false, defAttCmdAcked = false, recenterCmdAcked = false;

// testing
double globalArea = 0, globalRoundness = 0;
int param1 = 1, param2 = 1, blurSize = 9;

// fps check
long oldFrameTimestamp = 0, frameTimestamp = 0;

#pragma endregion

#pragma region Puck Tracking And Trajectory

#pragma region Subsection: Reference Frame Conversions
// converts given coordinates (xC, yC) in camera frame (pixels) to coordinates (xT, yT) in table frame (inches)
// assumes small camera rotations -> cosTheta = 1, sinTheta = theta
void convertFromCameraToTableFrame(float xC, float yC, float* xT, float* yT)
{
	//xC = CAM_PIX_WIDTH - xC; yC = CAM_PIX_HEIGHT - yC;
	//*xT = (1*xC - cam_rotation*yC)*CAM_PIX_TO_IN_X + deltaXct;
	//*yT = (cam_rotation*xC + 1*yC)*CAM_PIX_TO_IN_Y + deltaYct;
	*xT = (XC0 - xC)*CAM_PIX_TO_IN_X;
	*yT = (YC0 - yC)*CAM_PIX_TO_IN_Y;
	//*xT = xBoundHigh - *xT; *yT = yBoundHigh - *yT;
}

// converts from table coordinates in inches to camera coordinates in pixels
void convertFromTableToCameraFrame(float xT, float yT, float* xC, float* yC)
{
	//xT = xBoundHigh - xT; yT = yBoundHigh - yT;
	//*xC = (1*xT - cam_rotation*yT - deltaXct)/CAM_PIX_TO_IN_X;
	//*yC = (cam_rotation*xT + 1*yT - deltaYct)/CAM_PIX_TO_IN_Y;
	*xC = XC0 - xT/CAM_PIX_TO_IN_X;
	*yC = YC0 - yT/CAM_PIX_TO_IN_Y;
}
#pragma endregion

// Image segmentation, object extraction based on size and form (roundness)
void trackObjectPuck(Mat imgThresholded)
{
	#pragma region Code
	vector< vector<Point> > contours;

	// Position initialization
	xPC = prev_xPC; yPC = prev_yPC;
	vPx = prev_vPx; vPy = prev_vPy;
	status = 0;
	
	/* //HoughCircles() sucks
	vector< Vec3f > circles;
	//HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1, 1, 150, 10, 0, 0 );
	HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT, 1, 1, param1, param2, 0, 0 );
	
	for( size_t i = 0; i < circles.size(); i++ )
    {
         Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
         int radius = cvRound(circles[i][2]);
		 cout << "circles " << center << "," << radius << endl;
         // draw the circle center
         circle( imgThresholded, center, 3, Scalar(0,255,0), 1, 8, 0 );
         // draw the circle outline
         circle( imgThresholded, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }
    namedWindow( "circles", 1 );
    imshow( "circles", imgThresholded );
    return;
	*/

	//finding all contours in the image (segmentation)
	//cvFindContours(imgThresh, storage, &contours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	findContours(imgThresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	//if (contours.size() > 0)
	//	status=1;

	//cout << "Contours size: " << contours.size() << endl;

	velocityUpdated = false;
	for(int i=0; i<contours.size(); i++)
	{
		vector<Point> contour = contours[i];
		//cout << "Contours: " << contour << endl;

		// get max x pixel value in contour, since only right edge of puck is detected
		int maxX = 0;
		for(int j=0;j<contour.size();j++) {
			maxX = contour[j].x > maxX ? contour[j].x : maxX;
		}
		//cout << "max: " << maxX << endl;

		double area = contourArea(contour);
		globalArea = area;
		//cout << "Area: " << area << endl;

		//if ((area>400)&&(area<800))  // Min and Max size of object
		//if ((area>75)&&(area<200))  // Min and Max size of object
		if ((area>8)&&(area<250))  // Min and Max size of object
		{
			status = 2;
			//Detecting roundness   roundness = perimeter2/(2*pi*area)
			double perimeter = arcLength(contour, 1);
			double roundness = (perimeter*perimeter)/(6.28*area);
			globalRoundness = roundness;
			//cout << "Roundness: " << roundness << endl;
			//printf("%lf",roundness);
			if (roundness < 2.9)
			{
				status = 3;

				//Calculate the moments of the thresholded image
				Moments oMoments = moments(contour);
				double moment01 = oMoments.m01;
				double moment10 = oMoments.m10;
				area = oMoments.m00;
				
				// Calculate object center
				// We are using 320x240 pix but we are going to output the 640x480 equivalent (*2)
				///posX = floor(moment10*2/(double)area+0.5); // round
				///posY = floor(moment01*2/(double)area+0.5);
				//xPC = floor(moment10/(double)area+0.5); // round
				xPC = (maxX - rPuck/CAM_PIX_TO_IN_X);
				yPC = floor(moment01/(double)area+0.5);
				objectSize = area;

				//cout << "Round2" << endl;
				//cout << xPC << "," << yPC << endl;

				//convert puck coordinates from camera frame to table frame
				convertFromCameraToTableFrame(xPC, yPC, &xPT, &yPT);

				
				// compute velocity once every 2 position updates
				if(xPT != prev_xPT) { updateVelocityCounter++; }
				if(updateVelocityCounter > 0) {
					long positionTimestamp = getTickCount();
					double dt = ((double)(positionTimestamp - oldPositionTimestamp))/getTickFrequency();
					if(oldPositionTimestamp == -1) { dt = 0.03; }
					//calculate puck velocity components in table frame (in/s)
					vPx = (xPT - prev2_xPT)/dt; vPy = (yPT - prev2_yPT)/dt;
					oldPositionTimestamp = getTickCount();
					//vPx = ((xPT - prev_xPT)/dt); vPx = (prev_vPx + 2*vPx)/3;
					//vPy = ((yPT - prev_yPT)/dt); vPy = (prev_vPy + 2*vPy)/3;
					//if(vPx*prev_vPx > 0) { vPx = (2*vPx + prev_vPx + accelPx*dt)/3; }
					//if(vPy*prev_vPy > 0) { vPy = (2*vPy + prev_vPy + accelPy*dt)/3; }

					accelPx = (vPx - prev_vPx)/dt; accelPy = (vPy - prev_vPy)/dt;
					prev2_xPT = xPT; prev2_yPT = yPT; velocityUpdated = true;
					updateVelocityCounter = 0;
				}
				//cout << "vpx: " << vPx << "\t" << xPT << "," << prev_xPT << endl;
			}
		}
	}
	//if(updateVelocityCounter == 0) { oldPositionTimestamp = getTickCount(); }
	if(status < 3) { vPx = -0.5; vPy = -0.5; }
	//printf("\n");
}
#pragma endregion

void predictPuckTrajectory(Mat *imgTrajectory, float *critYvalues, float *critTvalues, float *critVyvalues)
#pragma region Code
{
	bool initFlag = initCounts < initCountsNeeded;
	//print table coords
	//cout << "Table coords: " << xPT << "," << yPT << endl;

	if(initFlag) { return; }
	//return;

	//-- use rP(t) vector to check +-y boundaries for ricochet, predict full trajectory to x=0 --//
	vector<float> tTrajectory, xTrajectory, yTrajectory;
	vector<float> vxTrajectory, vyTrajectory;
	tTrajectory.push_back(0); xTrajectory.push_back(xPT); yTrajectory.push_back(yPT);
	vxTrajectory.push_back(vPx); vyTrajectory.push_back(vPy);
	float t = 0;	//time
	
	//cout << "here2" << endl;
	float xLock = xPT, yLock = yPT;	//save these values for reassignment later
	//verify that puck is coming towards robot
	if(vPx < -1)
	{
		//cout << "here3" << endl;
		//check if puck has a y-component velocity -- if so, check for ricochets
		if(abs(vPy) > 1)
		{
			bool done = false;
			int loopCounts = 0;
			while(!done) {		//continue until trajectory reaches end of table
				//cout << "here4 " << vPx << "," << vPy << endl;
				// pull out latest values
				t = tTrajectory.back();
				xPT = xTrajectory.back(); yPT = yTrajectory.back();
				vPx = vxTrajectory.back(); vPy = vyTrajectory.back();

				// Determine x-value and time at which next ricochet occurs
				// yPT(t) = yPT(t0) + vPy*t
				float tRicochet = 0, xRicochet = 0, yRicochet = 0;
				// first order prediction
				if(vPy > 0) { tRicochet = (yBoundHigh - yPT) / vPy; yRicochet = yBoundHigh; }
				else		{ tRicochet = (yBoundLow - yPT)  / vPy; yRicochet = yBoundLow;  }
				xRicochet = xPT + vPx*tRicochet;
				/*
				// second order prediction -- uses quadratic equation
				if(vPy > 0) {
					tRicochet = (-1*vPy + sqrt( pow(vPy,2) - 2*accelPy*(yPT-yBoundHigh) ))/accelPy;
					if(tRicochet < 0) { tRicochet = (-1*vPy - sqrt( pow(vPy,2) - 2*accelPy*(yPT-yBoundHigh) ))/accelPy; }
				}
				else {
					tRicochet = (-1*vPy + sqrt( pow(vPy,2) - 2*accelPy*(yPT-yBoundLow) ))/accelPy;
					if(tRicochet < 0) { tRicochet = (-1*vPy - sqrt( pow(vPy,2) - 2*accelPy*(yPT-yBoundLow) ))/accelPy; }
				}
				xRicochet = xPT + vPx*tRicochet + 0.5*accelPx*tRicochet*tRicochet;
				*/
				// If next ricochet occurs before reaching end of table (x=xBoundLow), add to trajectory vectors
				if(xRicochet >= xBoundLow) {
					vPy *= 0.90;	//assume some energy is lost in ricochet
					tTrajectory.push_back(t + tRicochet); xTrajectory.push_back(xRicochet); yTrajectory.push_back(yRicochet);
					vxTrajectory.push_back(vPx); vyTrajectory.push_back(-1*vPy);
					cout << "Added ricochet: " << t + tRicochet << "\t" << xRicochet << "," << yRicochet;
					cout << "\t" << vPx << "," << vPy << endl;
					ricochetDetected = true;
				}
				else {
					done = true;
				}
				loopCounts++;
				if(loopCounts > 10) { done = true; }	//infinite loop protection
			}
		}

		// add final state to trajectory vectors (at x = xBoundLow)
		/*
		float tAtxl = (-1*vPx + sqrt( pow(vPx,2) - 2*accelPx*(xPT-xBoundLow) ))/accelPx;	//second order
		if(tAtxl < 0) { tAtxl = (-1*vPx - sqrt( pow(vPx,2) - 2*accelPx*(xPT-xBoundLow) ))/accelPx; }
		float yAtxl = yPT + vPy*tAtxl + accelPy*tAtxl*tAtxl;
		*/
		float tAtxl = (xBoundLow - xPT) / vPx, yAtxl = yPT + vPy*tAtxl;	//first order
		tTrajectory.push_back(tAtxl); xTrajectory.push_back(xBoundLow); yTrajectory.push_back(yAtxl);
		vxTrajectory.push_back(vPx); vyTrajectory.push_back(vPy);
		
		//-- Calculate y and t at critical x values -- use these values to decide on attack/defense strategy --//
		int critXindex = 0;
		//float critYvalues[numberOfCritXvalues], critTvalues[numberOfCritXvalues];

		for(int i=0; i<tTrajectory.size() - 1; i++)
		{
			float t = tTrajectory[i], x = xTrajectory[i], y = yTrajectory[i];
			float vx = vxTrajectory[i], vy = vyTrajectory[i];
			float nextX = xTrajectory[i+1], nextY = yTrajectory[i+1];
			
			if(displayEnabled) {
				// add to imgTrajectory Mat for display
				float currentxPC, currentyPC, newxPC, newyPC;				//current and next trajectory points in camera frame
				convertFromTableToCameraFrame(x, y, &currentxPC, &currentyPC);		//current point in trajectory
				convertFromTableToCameraFrame(nextX, nextY, &newxPC, &newyPC);		//next point(ricochet)
				line(*imgTrajectory, Point(newxPC, newyPC), Point(currentxPC, currentyPC), Scalar(0,255,0), 2);
			}

			// pick up critXvalues that lie on this trajectory segment: critXvalues array is already sorted by x value descending
			while(critXindex < numberOfCritXvalues && nextX <= criticalXvalues[critXindex]) {
				float critX = criticalXvalues[critXindex];
				float tOnThisPath = (critX - x) / vx;
				critTvalues[critXindex] = t + tOnThisPath;
				critYvalues[critXindex] = y + vy * tOnThisPath;
				critVyvalues[critXindex] = vy;
			
				critXindex++;
				if(critXindex >= numberOfCritXvalues) { break; }
			}
		}
	}
	//reassign xPT, yPT to current values so they can be used elsewhere
	xPT = xLock; yPT = yLock;
	vPx = vxTrajectory[0]; vPy = vyTrajectory[0];
}
#pragma endregion

#pragma endregion

#pragma region Camera Capture
// Camera capture class
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

#pragma region Serial Comm

bool openComPort()
#pragma region Code
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
#pragma endregion

// Read from COM PORT and output to console
bool readComPort()
#pragma region Code
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
			if(ackedCmd == startDefenseCmd && state == 2)		{ defenseCmdAcked = true; }
			else if(ackedCmd == startAttackCmd && state == 2)	{ attackCmdAcked = true; }
			else if(ackedCmd == defendAttackCmd && state == 2)	{ defAttCmdAcked = true; }
			else if(ackedCmd == recenterCmd && state == 3)	{ recenterCmdAcked = true; }
		}
	}
	return true;
}
#pragma endregion

bool writeComPort(unsigned char *message, int length)
#pragma region Code
{
	DWORD byteswritten;

	bool retVal = WriteFile(serialPort,message,length,&byteswritten,NULL);
	return retVal;
}
#pragma endregion

// copy contents of srcArray into destArray at specified index
void arrayCopy(unsigned char *destArray, int destIndex, unsigned char *srcArray, int length)
#pragma region Code
{
	for(int i=0; i<length; i++)
		destArray[destIndex + i] = srcArray[i];
}
#pragma endregion

// params target x, target y, target time
bool sendDefendCommand(float xT1, float yT1, float tT1)
#pragma region Code
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
#pragma endregion

// inputs: x, y puck at attack point, time puck is at point, angle of attack
bool sendAttackCommand(float xP2, float yP2, float tT2)
#pragma region Code
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
#pragma endregion

bool sendRecenterCommand(bool centerHomeY)
#pragma region Code
{
	unsigned char message[5];
	message[0] = 'r'; message[1] = 'c';
	message[2] = centerHomeY;
	message[3] = 0x08; message[4] = 0x09;

	return writeComPort(message,5);
}
#pragma endregion

// inputs: defense point and time, attack point and time and angle
bool sendDefendAttackCommand(float xD, float yD, float tD, float xA, float yA, float tA, float aA)
#pragma region Code
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
#pragma endregion

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
	CLEyeCapture *cam = NULL;
	// Query for number of connected cameras
	int numCams = CLEyeGetCameraCount();
	if(numCams == 0)
	{
		printf("No PS3Eye cameras detected\n");
		return -1;
	}
	
	bool opened = openComPort();
	if(!opened) {
		cout << "Failed to open com port!" << endl;
		Sleep(1000);
		return -1;
	}
	
	if(displayEnabled) {
		cvNamedWindow(OUTPUT_WINDOW, CV_WINDOW_AUTOSIZE);
		cvNamedWindow("Contours",CV_WINDOW_AUTOSIZE);
	}

	cam = new CLEyeCapture(CLEYE_QVGA, (CLEyeCameraColorMode)CLEYE_COLOR_PROCESSED, 100);
	
	// start capture
	cam->Start();
	//boost::thread t(CaptureThread);
	ofstream myfile;
	
	if(loggingEnabled) { myfile.open("asdf.txt"); }
	Sleep(2000);	//wait for camera to initialize -- this could end badly

	//calibration
	/*
	namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &minH, 255); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &maxH, 255);
	//createTrackbar("param1", "Control", &param1, 50); //Hue (0 - 179)
	//createTrackbar("param2", "Control", &param2, 50);

	//createTrackbar("blur", "Control", &blurSize, 50);
	createTrackbar("LowS", "Control", &minS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &maxS, 255);

	createTrackbar("LowV", "Control", &minV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &maxV, 255);
	*/

	int colorCycler = 0;
	float xPTfilt = 0, yPTfilt = 0, vPxfilt = 0, vPyfilt = 0;
	xPT = 100, yPT = 100;
	// x,y,t are target defense position (defense) or target puck position (attack). angleTarget is angle of attack
	float xDef = 0, yDef = 0, tDef = 0, xAtt = 0, yAtt = 0, tAtt = 0, angleAtt = 0;
	float critYvalues[numberOfCritXvalues], critTvalues[numberOfCritXvalues], critVyvalues[numberOfCritXvalues];
	float tAttThreshold = 0.33, tDefThreshold = 0.02;	//minimum required value of tAtt to send attack command
	bool arduinoReady = false, attInRange = false, defInRange = false, recenterLock = false, centerHomeY = false;
	bool filtPointsReqSet = false;
	int filtPoints = 0, filtPointsReq = 4;
	long prevCommandTime = 0, prevState1Time = 0;
	int vPxHighCounter = 0, vPxNegCounter = 0, retransmitCounter = 0;
	float loopTime = 0, timeInState1 = 0, state1Timeout = 1;	//seconds
	while(true) {
		readComPort();
		//long ts = getTickCount();

		Mat imgOriginal;
		mtx.lock(); imgCapture.copyTo(imgOriginal); mtx.unlock();

		//long new_ts = getTickCount();
		//double tm = ((double)(new_ts - ts))/getTickFrequency() * 1000;
		//cout << "Ts1: " << tm << endl;

		// process image //

		// 40 is COLOR_BGR2HSV
		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, 40); //Convert the captured frame from BGR to HSV
		
		Mat imgThresholded;
		inRange(imgHSV, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), imgThresholded); //Threshold the image
		
		//GaussianBlur( imgThresholded, imgThresholded, Size(9, 9), 3, 3 );
		//medianBlur(imgThresholded, imgThresholded, blurSize);

		//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		
		//imshow(OUTPUT_WINDOW, imgThresholded); //show the thresholded image

		//track the position of the puck and the robot
		trackObjectPuck(imgThresholded);
		//if(state == 1) { cout << xPT << "," << yPT << "\t" << vPx << "\t" << vPy << endl; }

		//predict trajectory
		Mat imgTrajectory = Mat::zeros( imgOriginal.size(), CV_8UC4 );

		vPxHighCounter = vPx > 1 ? vPxHighCounter + 1 : 0;
		vPxNegCounter = vPx < -1 ? vPxNegCounter + 1 : 0;

		#pragma region "State Machine"
		int next_state = state;
		switch(state) {
			case 0: {	//startup/homing -- default state
				//entry
				if(prev_state != 0) {
					cout << "Entered state 0" << endl;
				}

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
					predictPuckTrajectory(&imgTrajectory, critYvalues, critTvalues, critVyvalues);
					//cout << "State 0: " << xPT << "," << yPT << "\t" << vPx << "\t" << vPy << endl;
				}
				break;
			}
			case 1: { //puck moves player -> robot
				// entry -- clear filter values
				if(prev_state != 1) {
					cout << "Entered state 1" << endl;
					cout << xPT << "," << yPT << "," << vPx << "," << vPy << "\t";
					cout << prev_xPT << "," << prev_yPT << endl;
					xPTfilt = 0; yPTfilt = 0; vPxfilt = 0; vPyfilt = 0; filtPoints = 0;
					xDef = 0; yDef = 0; tDef = 0; xAtt = 0; yAtt = 0; tAtt = 0;
					recenterLock = false; ricochetDetected = false; timeInState1 = 0;
				}
				if(!velocityUpdated) {	//don't do anything if we didn't get a position/velocity update
					timeInState1 += loopTime;
					if(timeInState1 > state1Timeout) { next_state = 0; }	//timeout to avoid getting stuck here
					break;
				}
				if(abs(vPx) < 0.01 && abs(vPy) < 0.01) { vPx = prev_vPx; vPy = prev_vPy; }
				if(abs(vPx - -0.5) < 0.01 && abs(vPy - -0.5) < 0.01) { vPx = prev_vPx; vPy = prev_vPy; }
				// adjust required number of filter points based on x-velocity
				if(!filtPointsReqSet) {
					if(vPx < -350) { filtPointsReq = 2; }
					else if(vPx < -200) { filtPointsReq = 3; }
					else { filtPointsReq = 4; }
					filtPointsReqSet = true;
				}
				filtPoints++;

				// set equal to filter values if no velocities were updated this iteration
				/*
				if(abs(vPx - -0.5) < 0.01 && abs(vPy - -0.5) < 0.01) { vPx = vPxfilt; vPy = vPyfilt; }
				float dir = vPy > 0 ? 1 : -1;
				
				// update velocity filter
				vPxfilt += vPx; vPyfilt += abs(vPy);	//abs because it can change directions
				vPx = vPxfilt/filtPoints; vPy = dir*vPyfilt/filtPoints;
				*/
				// state actions -- predict puck trajectory, decide on attack/defense when conditions met
				predictPuckTrajectory(&imgTrajectory, critYvalues, critTvalues, critVyvalues);

				// check time at critXvalues[1] -- attack point
				xAtt = criticalXvalues[1]; //yAtt += critYvalues[1]; tAtt += critTvalues[1];
				
				// check y, t values before adding to filter
				yAtt += checkBounds(critYvalues[1], yAtt, yBoundLow, yBoundHigh, filtPoints);
				tAtt += checkBounds(critTvalues[1], tAtt, 0, 100, filtPoints);
					
				// determine command point for defense
				/*
				int index_minVelocity = 0;		//crit X,Y,T that robot can reach with lowest speed
				float minVelocity = -1;			//lowest robot velocity required to reach a crit point
				for(int i=0;i<numberOfCritXvalues;i++) {
					float critX = criticalXvalues[i], critY = critYvalues[i], critT = critTvalues[i];
					if(critT < 0) { critT = 0.00001; }
					float distXsqrd = (critX - xR)*(critX - xR), distYsqrd = (critY - yR)*(critY - yR);
					float distRP = sqrt(distXsqrd + distYsqrd);
					float reqVelocity = distRP / critT;

					if(reqVelocity < minVelocity || minVelocity == -1) {
						minVelocity = reqVelocity;
						index_minVelocity = i;
					}

					cout << "1: " << critT << "," << critX << "," << critY << endl;
				}

				// set target defense position and time of arrival
				xDef = criticalXvalues[index_minVelocity]; yDef = critYvalues[index_minVelocity];
				tDef = critTvalues[index_minVelocity];
				*/
				int idx = numberOfCritXvalues - 1;
				xDef = criticalXvalues[idx]; yDef += critYvalues[idx]; tDef += critTvalues[idx];
				
				//yDef += checkBounds(critYvalues[idx], yDef, yBoundLow, yBoundHigh, filtPoints);
				//tDef += checkBounds(critTvalues[idx], tDef, 0, 100, filtPoints);
	
				// compute attack and defend points + times, randomize attack angle
				bool decide = filtPoints >= filtPointsReq;// || (xPT <= xThreshold_decision && filtPoints > 1);

				//cout << xDef << "," << yDef << "," << tDef << "\t" << xAtt << "," << yAtt << "," << tAtt << endl;
				cout << xDef << "," << critYvalues[idx] << "," << critTvalues[idx] << "\t";
				cout << xAtt << "," << critYvalues[1] << "," << critTvalues[1] << "\t";
				cout << vPx << "," << vPy << endl;

				bool noMoves = false;
				if(decide) {
					//send once here for fastest response, continuously send after state 2 transition
					yDef /= filtPoints; tDef /= filtPoints;
					yAtt /= filtPoints; tAtt /= filtPoints;
					
					//if puck is coming at an angle, offset defense point by paddle radius
					//if(critVyvalues[idx]/vPx < -0.33) { yDef += rPaddle; }
					//else if(critVyvalues[idx]/vPx > 0.33) { yDef -= rPaddle; }
					//if(critVyvalues[idx]/vPx < -0.33) { yDef -= rPaddle; }
					//else if(critVyvalues[idx]/vPx > 0.33) { yDef += rPaddle; }

					// check bounds here to free up compute time on arduino
					//defInRange = yDef > 3.45 && yDef < 33 && tDef > 0.02;
					defInRange = yDef >= 10 && yDef <= 29 && tDef > tDefThreshold;
					// shrink defense range when ricochet expected
					//if(ricochetDetected) { defInRange = yDef >= 15 && yDef <=24 && tDef > tDefThreshold; }
					attInRange = yAtt > 3.45 && yAtt < 33 && tAtt > tAttThreshold;
					//sendDefendAttackCommand(xDef,yDef,tDef,xAtt,yAtt,tAtt,angleAtt);

					if(attInRange) { sendAttackCommand(xAtt,yAtt,tAtt); }
					else if(defInRange) { sendDefendCommand(xDef,yDef,tDef); }
					else { noMoves = true; }	//both out of range, give up
					prevCommandTime = getTickCount();
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
			case 2: {	//executing defense/attack routine
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
				if(attInRange && tAtt >= tAttThreshold && !attackCmdAcked && retransmitCounter > 1) {	//send attack command if there's sufficient time
					sendAttackCommand(xAtt,yAtt,tAtt);
					attackCmdAcked = false;
				}
				else if(defInRange && tDef >= tDefThreshold && !defenseCmdAcked && retransmitCounter > 1) {	//send defense command otherwise
					sendDefendCommand(xDef,yDef,tDef);
					defenseCmdAcked = false;
				}
				prevCommandTime = getTickCount();
				retransmitCounter = retransmitCounter > 1 ? 0 : retransmitCounter++;	//avoid swamping arduino

				// transitions
				next_state = tDef < tDefThreshold || attackCmdAcked || defenseCmdAcked ? 0 : 2;
				//next_state = xPT >= xThreshold_away && vPx > 1 ? 3 : next_state;
				next_state = vPxHighCounter > 0 && !recenterLock ? 3 : next_state;
				if(next_state != 2) { defAttCmdAcked = false; attackCmdAcked = false; defenseCmdAcked = false; }
				break;
			}
			case 3: { //puck moves robot -> player
				if(prev_state != 3) {
					cout << endl << "Entered state 3" << endl << endl;
					recenterCmdAcked = false;
					centerHomeY = vPx > 1 && vPx < 175;	//decide whether shot is slow enough to centerhome y-axis
					retransmitCounter = 3;
				}
				
				if(recenterLock) { next_state = 0; break; }

				if(!recenterCmdAcked) {
					cout << "Retransmitting..." << endl;
					sendRecenterCommand(centerHomeY);
				}
				//retransmitCounter = retransmitCounter > 1 ? 0 : retransmitCounter++;
				/*
				//recenter
				if(!defenseCmdAcked) {
					//cout << "Recentering..." << endl;
					sendDefendCommand(criticalXvalues[numberOfCritXvalues-1],yBoundHigh/2,0.25);
					defenseCmdAcked = false;
					xR = criticalXvalues[numberOfCritXvalues-1]; yR = yBoundHigh/2;
				}
				next_state = defenseCmdAcked ? 0 : 3;
				next_state = vPx < 0 && xPT <= xThreshold_towards ? 1 : next_state;
				if(next_state != 3) {
					attack = false; defend = false;
					defenseCmdAcked = false;
				}
				*/
				next_state = recenterCmdAcked ? 0 : 3;
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
			if(initCounts > initCountsNeeded && prev_xPC >= 0 && prev_yPC>=0 && xPC>=0 && yPC>=0) {
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
				line(imgLines, Point(xPC, yPC), Point(prev_xPC, prev_yPC), Scalar(0,0,255), 2);
				//cout << "lines added" << endl;
			}

			imgOriginal = imgOriginal + imgLines + imgTrajectory;
			imshow(OUTPUT_WINDOW, imgOriginal); //show the original image
			imshow("Contours",imgThresholded);
		
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

		//update previous vars
		prev_xPC = xPC; prev_yPC = yPC;
		prev_xPT = xPT; prev_yPT = yPT;
		prev_vPx = vPx; prev_vPy = vPy;
		initCounts++;

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
