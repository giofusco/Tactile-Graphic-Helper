#pragma once
//#include "../MarkerDetectorAruco/MarkerDetectorAruco.h"

class Cursor
{
public:
	Cursor(void){
		tipPoint_ = cv::Point3f(0,0,0.015);
		tipPoint_.x = 0.025;
		tipPoint_.y = -0.005;
		lastPosition_ = cv::Point2f(0,0);
	}

	Cursor(int markerID, float markerSize, std::string calibrationFilename) : markerID_(markerID) {
		//detector_ = MarkerDetectorAruco(calibrationFilename,  markerSize);
		Cursor();
	}

	cv::Point2f detectCursor(cv::Mat frame, bool verbose);
	
	inline void adjustCursor(float xinc, float yinc, float zinc = 0) {
		tipPoint_.x += xinc;
		tipPoint_.y += yinc;
		tipPoint_.z += zinc;
	};

	inline void setHomography(cv::Mat H){
		H_ = H ;
	};

	~Cursor(void);
private:
	int markerID_;
	//MarkerDetectorAruco detector_;
	cv::Point3f tipPoint_;
	cv::Point2f lastPosition_;
	cv::Mat H_;
};

