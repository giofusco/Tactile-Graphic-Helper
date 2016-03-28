#include "Fingertip.h"
#include <opencv2/opencv.hpp>


long int Fingertip::id_ = 0;

Fingertip::Fingertip(void)
{
	uchar r = rand() % 255, g = rand() % 255, b = rand() % 255;
	framesSinceLastSeen = 0;
	validityThreshold_ = 5;
	this->color_  = CV_RGB(r, g, b);
//	std::cerr << "Color: " << this->color_ << std::endl;
	lastPosition_ = cv::Point(-1,-1);
	id_++;
	time(&timer_);	// first time seen we assume it is stationary
}

Fingertip::Fingertip(cv::Point pos) : Fingertip(){
	addPoint(pos);
	firstStationary_ = pos;
}

Fingertip::Fingertip(cv::Point pos, int valThreshold ) : Fingertip(pos){
	validityThreshold_ = valThreshold;
	//addPoint(pos);
}

Fingertip::~Fingertip(void)
{
}


void Fingertip::addPoint(cv::Point pos){
	lastPosition_ = pos;
	trace_.push_back(pos);
	framesSinceLastSeen = 0;

}

cv::Point Fingertip::getPosition(int numSample){
	if (trace_.size()>=numSample){
		vector<cv::Point>::reverse_iterator rit;
		int cnt=0;
		cv::Point meanP = cv::Point(0,0);
		for (rit = trace_.rbegin(); cnt <numSample; ++rit){
			//std::cerr << "*" ;
			meanP.x = meanP.x + rit->x;
			meanP.y = meanP.y + rit->y;
			cnt++;
		}
		//std::cerr << std::endl;
		return cv::Point(meanP.x/numSample, meanP.y/numSample);
	}
	else return lastPosition_;
}

bool Fingertip::addPoint(cv::Point pos, float maxDist){
	float d = abs(pos.x - lastPosition_.x) + abs(pos.y - lastPosition_.y);
	
	//std::cerr << "Tip: " << lastPosition_ << ", curr: " << pos << std::endl;
	if (lastPosition_ != cv::Point(-1,-1)){
		float d_first = abs(pos.x - firstStationary_.x) + abs(pos.y - firstStationary_.y);
		if (d < maxDist){
			addPoint(pos);
			if (d_first > 5.){
				time(&timer_); //reset stationary time
				firstStationary_ = pos;
			}
			return true;
		}
		else return false;
	}
	else{
		time(&timer_);	// first time seen we assume it is stationary
		addPoint(pos);
		firstStationary_ = pos;
		return true;
	}
}