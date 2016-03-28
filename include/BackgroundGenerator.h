#pragma once

#include "opencv2\opencv.hpp"
#include <math.h>
#include "FrameGrabber.h"
#include "SOM.hpp"

//#include "../MarkerDetectorAruco/MarkerDetectorAruco.h"

class BackgroundGenerator
{

private:
	cv::Mat H_; //rectification homography
	cv::Mat backgroundImage_;
	//MarkerDetectorAruco detector_;
	vector<cv::Point2f> rectifiedPoints_;
	vector<cv::Point2f> markerCenter_;
	float sheetWmt_;
	float sheetHmt_;
	float pixelXmeter_;
	int lastHue_;
	int consecutiveHue_;
	
	cv::Mat ST_, W_, st2_,std2_;
	void sortCorners(std::vector<cv::Point>& corners, cv::Point center);
	void findHomographyUsingCorners(FrameGrabber* grabber, int numFrames);
	void initFilters();
	cv::Mat getMedianImage(vector<cv::Mat> stack);
	cv::Mat getMean(vector<cv::Mat> stack, cv::Mat &std);
	cv::Mat metricallyTrimmedMean(vector<cv::Mat> stack, cv::Mat median, float alpha = 0.3);
	cv::Mat computeMAD(vector<cv::Mat> stack, cv::Mat median);
	cv::Mat mask_std, mask;
	cv::Mat diamond2, diamond3;
	cv::Mat Hue_, Huestd_;
	SOM som_;
	cv::Mat rect3x3;
	

public:
	BackgroundGenerator(void) {};
	BackgroundGenerator(float markerSize, float sheetWmt, float sheetHmt, float pixelXmeter, string calibrationFilename) : 
		sheetWmt_(sheetWmt), sheetHmt_(sheetHmt), pixelXmeter_(pixelXmeter)
	{
		//detector_ = MarkerDetectorAruco(calibrationFilename,  markerSize);
		rectifiedPoints_.push_back(cv::Point2f(0,0));
		rectifiedPoints_.push_back(cv::Point2f(0,sheetHmt*pixelXmeter));
		rectifiedPoints_.push_back(cv::Point2f(sheetWmt*pixelXmeter,sheetHmt*pixelXmeter));
		rectifiedPoints_.push_back(cv::Point2f(sheetWmt*pixelXmeter,0));
		initFilters();
		lastHue_ = -1;
		consecutiveHue_ = 0;
		fcnt = 0;

	};

	BackgroundGenerator(float sheetWidth, float sheetHeight) :
		sheetWmt_(sheetWidth), sheetHmt_(sheetHeight), pixelXmeter_(1.)
	{
		rectifiedPoints_.push_back(cv::Point2f(0, 0));
		rectifiedPoints_.push_back(cv::Point2f(0, sheetHmt_));
		rectifiedPoints_.push_back(cv::Point2f(sheetWmt_, sheetHmt_));
		rectifiedPoints_.push_back(cv::Point2f(sheetWmt_, 0));
		initFilters();
		lastHue_ = -1;
		consecutiveHue_ = 0;
		fcnt = 0;

	};

	inline cv::Mat getHomography(){ return H_; };
	inline cv::Mat getBackgroundImage(){ return backgroundImage_; };
	void computeBackground(int numFrames, FrameGrabber* grabber);
	void computeBackgroundUsingCorners(int numFrames, FrameGrabber* grabber);
	void computeBackground_TM09(int numFrames, FrameGrabber* grabber, float alpha = 0.3);
	void computeBackgroundHue(int numFrames, FrameGrabber* grabber);
	void computeBackground_SOM(int numFrames, FrameGrabber* grabber);
	long int fcnt;
	cv::Mat getForegroundMask_TM09(cv::Mat frame, float k);
	cv::Mat getForegroundMaskHue(cv::Mat frame, float tol);
	cv::Mat getForegroundMask_SOM(cv::Mat frame);
	
	~BackgroundGenerator(void);
};

