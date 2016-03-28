#pragma once
#include <opencv2\core\core.hpp>

#include "robustLine.h"
#include "Fingertip.h";

using namespace std;

class FingerTracker
{
public:
	FingerTracker(void);
	~FingerTracker(void);
	void track(cv::Mat hand, float adjustment); //input: binary map and a factor to adjust the pad position
	void showTips(cv::Mat background);
	vector<cv::Point> getLastSeen(int numFrames);
	vector<Fingertip> getLastSeenTip(int numFrames);
private:
	cv::Mat diamond2, diamond3;
	vector<Fingertip> tips;
	float getWeight(cv::Point p, int r, cv::Mat img);
	
	static int isRidge(float v1, float v2, float v3) {
		if (v1 <= v2 && v2 >= v3) return 1;
		return 0;
	};
};

