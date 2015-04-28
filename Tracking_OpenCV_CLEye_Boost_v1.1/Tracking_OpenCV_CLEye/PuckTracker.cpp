#include "PuckTracker.h";

PuckTracker::PuckTracker() {}
PuckTracker::~PuckTracker() {}

PuckTracker::PuckTracker(float _rPuck, HSVBounds _bounds)
{
	// initialize vars
	xPC = 0; yPC = 0; prev_xPC = 0; prev_yPC = 0;
	xPT = 0; yPT = 0; prev_xPT = 0; prev_yPT = 0;
	accelPx = 0; accelPy = 0;
	vPxfilt = 0; vPyfilt = 0;
	tNextRicochet = -1;
	updateVelocityCounter = 0; oldVelocityTimestamp = -1;

	rPuck = _rPuck;

	xBoundLow = XBOUND_LOW + rPuck; xBoundHigh = XBOUND_HIGH;
	yBoundLow = YBOUND_LOW + rPuck; yBoundHigh = YBOUND_HIGH - rPuck;

	// Thresholding Parameters
	bounds = _bounds;
}

#pragma region Reference Frame Conversions
// converts given coordinates (xC, yC) in camera frame (pixels) to coordinates (xT, yT) in table frame (inches)
// assumes small camera rotations -> cosTheta = 1, sinTheta = theta
void PuckTracker::convertFromCameraToTableFrame(float xC, float yC, float* xT, float* yT)
{
	*xT = (XC0 - xC)*CAM_PIX_TO_IN_X + rPuck;
	*yT = (YC0 - yC)*CAM_PIX_TO_IN_Y + rPuck;
}

// converts from table coordinates in inches to camera coordinates in pixels
void PuckTracker::convertFromTableToCameraFrame(float xT, float yT, float* xC, float* yC)
{
	*xC = XC0 - (xT-rPuck)/CAM_PIX_TO_IN_X;
	*yC = YC0 - (yT-rPuck)/CAM_PIX_TO_IN_Y;
}
#pragma endregion

// Image segmentation, object extraction based on size and form (roundness)
// return int value is set of binary flags: 1 - position updated, 2 - velocity updated
int PuckTracker::UpdatePuckState(Mat imgOriginal)
{
	int returnFlags = 0;

	// process image //
	// 40 is COLOR_BGR2HSV
	Mat imgHSV;
	cvtColor(imgOriginal, imgHSV, 40); //Convert the captured frame from BGR to HSV
		
	Mat imgThresholded;
	inRange(imgHSV, Scalar(bounds.minH, bounds.minS, bounds.minV), Scalar(bounds.maxH, bounds.maxS, bounds.maxV), imgThresholded); //Threshold the image

	//morphological opening (removes small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

	//morphological closing (removes small holes from the foreground)
	dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

	vector< vector<Point> > contours;

	// Position initialization
	xPC = prev_xPC; yPC = prev_yPC;
	vPx = prev_vPx; vPy = prev_vPy;

	//finding all contours in the image (segmentation)
	findContours(imgThresholded, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

	bool positionUpdated = false, velocityUpdated = false;
	for(int i=0; i<contours.size(); i++)
	{
		vector<Point> contour = contours[i];
		// get max x pixel value in contour, since only right edge of puck is detected
		int maxX = 0;
		for(int j=0;j<contour.size();j++) { maxX = contour[j].x > maxX ? contour[j].x : maxX; }
		double area = contourArea(contour);
		if (area > AREA_MIN && area < AREA_MAX)  // Min and Max size of object
		{
			//Detecting roundness   roundness = perimeter2/(2*pi*area)
			double perimeter = arcLength(contour, 1);
			double roundness = (perimeter*perimeter)/(6.28*area);

			if (roundness < ROUNDNESS_MAX)
			{
				//Calculate the moments of the thresholded image
				Moments oMoments = moments(contour);
				double moment01 = oMoments.m01, moment10 = oMoments.m10;
				area = oMoments.m00;
				
				// Calculate object center
				xPC = ceil(maxX - rPuck/CAM_PIX_TO_IN_X);
				yPC = floor(moment01/(double)area+0.5);

				//convert puck coordinates from camera frame to table frame
				convertFromCameraToTableFrame(xPC, yPC, &xPT, &yPT);

				// compute velocity once every 2 position updates
				updateVelocityCounter += 1;
				if(updateVelocityCounter >= DOWNSAMPLE) {
					// calculate time interval since last velocity update
					long velocityTimestamp = getTickCount();
					double dt = ((double)(velocityTimestamp - oldVelocityTimestamp))/getTickFrequency();
					if(oldVelocityTimestamp == -1) { dt = 0.03; }

					//calculate puck velocity components in table frame (in/s)
					vPx = (xPT - prev_xPT)/dt; vPy = (yPT - prev_yPT)/dt;
					oldVelocityTimestamp = getTickCount();

					// various filtering attempts
					//vPx = ((xPT - prev_xPT)/dt); vPx = (prev_vPx + 2*vPx)/3;
					//vPy = ((yPT - prev_yPT)/dt); vPy = (prev_vPy + 2*vPy)/3;
					//if(vPx*prev_vPx > 0) { vPx = (2*vPx + prev_vPx + accelPx*dt)/3; }
					//if(vPy*prev_vPy > 0) { vPy = (2*vPy + prev_vPy + accelPy*dt)/3; }

					//accelPx = (vPx - prev_vPx)/dt; accelPy = (vPy - prev_vPy)/dt;
					UpdateVelocityFilter(dt);

					prev_xPT = xPT; prev_yPT = yPT; velocityUpdated = true;
					prev_xPC = xPC; prev_yPC = yPC;
					prev_vPx = vPx; prev_vPy = vPy;
					updateVelocityCounter = 0;
					returnFlags |= 2;
				}
				returnFlags |= 1;
			}
		}
	}
	return returnFlags;
}

void PuckTracker::UpdateVelocityFilter(double dt)
{
	if(prev_vPx*vPx > 1) { vPx = (prev_vPx + 2*vPx)/3; }	//check if velocity changed direction
	else { xRicochetOccurred = true; }
	if(prev_vPy*vPy > 1) { vPy = (prev_vPy + 2*vPy)/3; }
	else { yRicochetOccurred = true; }
}

// return int is binary flag set: 1 - ricochet detected
// this method is hard-coded for getting puck trajectory when approaching robot
int PuckTracker::PredictPuckTrajectory(Mat *imgTrajectory, bool displayEnabled, int numCritXvalues, float *critXvalues, float *critYvalues,
	float *critTvalues, float *critVyvalues)
{
	int returnFlags = 0;
	//-- use rP(t) vector to check +-y boundaries for ricochet, predict full trajectory to x=0 --//
	vector<float> tTrajectory, xTrajectory, yTrajectory;
	vector<float> vxTrajectory, vyTrajectory;
	tTrajectory.push_back(0); xTrajectory.push_back(xPT); yTrajectory.push_back(yPT);
	vxTrajectory.push_back(vPx); vyTrajectory.push_back(vPy);
	float t = 0;	//time
	
	float xLock = xPT, yLock = yPT;	//save these values for reassignment later
	//verify that puck is coming towards robot
	if(vPx < -1)
	{
		//check if puck has a y-component velocity -- if so, check for ricochets
		if(abs(vPy) > 1)
		{
			bool done = false;
			int loopCounts = 0;
			while(!done) {		//continue until trajectory reaches end of table
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
					
				// If next ricochet occurs before reaching end of table (x=xBoundLow), add to trajectory vectors
				if(xRicochet >= xBoundLow) {
					vPy *= 0.90;	//assume some energy is lost in ricochet
					tTrajectory.push_back(t + tRicochet); xTrajectory.push_back(xRicochet); yTrajectory.push_back(yRicochet);
					vxTrajectory.push_back(vPx); vyTrajectory.push_back(-1*vPy);
						
					cout << "Added ricochet: " << t + tRicochet << "\t" << xRicochet << "," << yRicochet;
					cout << "\t" << vPx << "," << vPy << endl;
					returnFlags |= 1;

					// Update next ricochet time to allow velocity filter to prepare
					if(abs(tNextRicochet - -1) < 0.001) { tNextRicochet = t + tRicochet; }
				}
				else {
					done = true;
				}
				loopCounts++; if(loopCounts > 10) { done = true; }	//infinite loop protection
			}
		}

		float tAtxl = (xBoundLow - xPT) / vPx, yAtxl = yPT + vPy*tAtxl;	//first order
		tTrajectory.push_back(tAtxl); xTrajectory.push_back(xBoundLow); yTrajectory.push_back(yAtxl);
		vxTrajectory.push_back(vPx); vyTrajectory.push_back(vPy);
		
		//-- Calculate y and t at critical x values -- use these values to decide on attack/defense strategy --//
		int critXindex = 0;
		//float critYvalues[numberOfCritXvalues], critTvalues[numberOfCritXvalues];

		// need to clear out values of critY, critT when xTrajectory[0] exceeds critX, i.e. puck has already passed critX value

		// fill critY, critT, critVy with values according to predicted trajectory
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
			while(critXindex < numCritXvalues && nextX <= critXvalues[critXindex]) {
				float critX = critXvalues[critXindex];
				float tOnThisPath = (critX - x) / vx;
				float v = sqrt(pow(vx,2) + pow(vy,2));
				critTvalues[critXindex] = t + tOnThisPath - abs(rPuck/v);
				critYvalues[critXindex] = y + vy * tOnThisPath;
				critVyvalues[critXindex] = vy;
			
				critXindex++;
				if(critXindex >= numCritXvalues) { break; }
			}
		}
	}
	//reassign xPT, yPT to current values so they can be used elsewhere
	xPT = xLock; yPT = yLock;
	vPx = vxTrajectory[0]; vPy = vyTrajectory[0];

	return returnFlags;
}