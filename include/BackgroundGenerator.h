#pragma once

#include "opencv2\opencv.hpp"
#include <math.h>
#include "FrameGrabber.h"
#include "SOM.hpp"

class BackgroundGenerator
{

private:
	cv::Mat H_; //rectification homography
	cv::Mat backgroundImage_;
	vector<cv::Point2f> rectifiedPoints_;
	vector<cv::Point2f> markerCenter_;
	float sheetWmt_;
	float sheetHmt_;
	float pixelXmeter_;
	
	void sortCorners(std::vector<cv::Point>& corners, cv::Point center);
	void findHomographyUsingCorners(FrameGrabber* grabber, int numFrames);
	cv::Mat mask_std, mask;
	SOM som_;
	cv::Mat rect3x3;
	

public:
	BackgroundGenerator(void) {};

	BackgroundGenerator(float sheetWidth, float sheetHeight) :
		sheetWmt_(sheetWidth), sheetHmt_(sheetHeight), pixelXmeter_(1.)
	{
		rectifiedPoints_.push_back(cv::Point2f(0, 0));
		rectifiedPoints_.push_back(cv::Point2f(0, sheetHmt_));
		rectifiedPoints_.push_back(cv::Point2f(sheetWmt_, sheetHmt_));
		rectifiedPoints_.push_back(cv::Point2f(sheetWmt_, 0));
		fcnt = 0;

	};

	inline cv::Mat getHomography(){ return H_; };
	inline cv::Mat getBackgroundImage(){ return backgroundImage_; };
	void computeBackground_SOM(int numFrames, FrameGrabber* grabber);
	long int fcnt;
	cv::Mat getForegroundMask_SOM(cv::Mat frame);
	
	~BackgroundGenerator(void);
};

