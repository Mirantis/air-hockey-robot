#include <vector>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

// This class tracks the puck and predicts its trajectory
#pragma region Defines
#define CAM_PIX_TO_IN_X 0.1942
#define CAM_PIX_TO_IN_Y 0.195
#define XC0 314				//pixel x value when puck is at table (0,0) corner
#define YC0 213				//pixel y value when puck is at table (0,0) corner
#define DOWNSAMPLE 1		//update velocity once per this many cycles
#define AREA_MAX 250		//image thresholding bound
#define AREA_MIN 8			//image thresholding bound
#define ROUNDNESS_MAX 2.9	//image thresholding bound
#define YBOUND_HIGH	39.25	//upper limit of table y minus puck radius
#define YBOUND_LOW 0		//lower limit of table y plus puck 
#define XBOUND_LOW 0
#define XBOUND_HIGH 62
#pragma endregion

class PuckTracker
{
public:
	// Thresholding Parameters
	struct HSVBounds { int minH, maxH, minS, maxS, minV, maxV; };
	HSVBounds bounds;

	// Constructors
	PuckTracker(float _rPuck, HSVBounds _bounds);
	PuckTracker();
	~PuckTracker();

	// State Variables
	float xPC, yPC, prev_xPC, prev_yPC;		//camera frame
	float xPT, yPT, prev_xPT, prev_yPT;		//table frame
	float vPx, vPy, prev_vPx, prev_vPy;
	float accelPx, accelPy;
	float rPuck;

	float vPxfilt, vPyfilt;
	float tNextRicochet;

	bool xRicochetOccurred, yRicochetOccurred;

	// Methods
	int UpdatePuckState(Mat imgOriginal);
	int PredictPuckTrajectory(Mat *imgTrajectory, bool displayEnabled, int numCritXvalues, float *critXvalues,
		float *critYvalues, float *critTvalues, float *critVyvalues);
	
private:
	void UpdateVelocityFilter(double dt);
	void convertFromCameraToTableFrame(float xC, float yC, float* xT, float* yT);
	void convertFromTableToCameraFrame(float xT, float yT, float* xC, float* yC);
	float xBoundLow, xBoundHigh, yBoundLow, yBoundHigh;
	int updateVelocityCounter;
	long oldVelocityTimestamp;
};