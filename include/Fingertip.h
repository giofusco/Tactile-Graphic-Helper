#pragma once
//#include <opencv2\world.hpp>
#include <opencv2\imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <time.h> 

using namespace std;
class Fingertip
{
public:
	Fingertip(void);
	Fingertip(cv::Point pos);
	Fingertip(cv::Point pos, int valThreshold );
	~Fingertip(void);
	inline vector<cv::Point> getTrace() { return trace_;}
	int framesSinceLastSeen;
	bool addPoint(cv::Point p, float maxDist);
	bool isValid(){ 
		return ((framesSinceLastSeen < validityThreshold_) ? true : false);}
	inline bool isStable(){ return (trace_.size()>3? true : false);}
	inline void drawTip(cv::Mat image) { cv::circle(image, getPosition(5), 0, 
										 cv::Scalar(this->color_(0), this->color_(1), this->color_(2)), 2), 
										 cv::circle(image, getPosition(5), 7, this->color_, 2); }
	cv::Point getPosition(int numSample);
	double getStationaryTime(){ return double(difftime(time(NULL), timer_)); }
	inline void resetStationaryTime() { time(&timer_); }; //reset timer to current time
private:
	static long int id_;
	cv::Scalar color_;
	cv::Point lastPosition_;
	cv::Point firstStationary_;
	vector<cv::Point> trace_;
	int validityThreshold_;
	void addPoint(cv::Point pos);
	time_t timer_; //it contains the first instant in which the fingertip appeared stationary

};


