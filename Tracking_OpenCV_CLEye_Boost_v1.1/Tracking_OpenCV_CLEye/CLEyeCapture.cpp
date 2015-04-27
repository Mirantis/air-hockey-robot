
#include "stdafx.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "boost/thread/thread.hpp"

using namespace std;

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
	boost::mutex mtx;
	
public:
	double tMin, tMax, tAvg, curr;
public:
	CLEyeCapture(CLEyeCameraResolution resolution, CLEyeCameraColorMode mode, int fps, boost::mutex boost::ref(_mtx)) :
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

		  mtx = _mtx;

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