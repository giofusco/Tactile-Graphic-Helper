#pragma once
#include <opencv2/core/core.hpp>
#include "opencv2\highgui\highgui.hpp"
#include <iostream>
#include <string>
#include <stdio.h>
#include <map>
#include "FeatureInfo.h"

using namespace std;


class TGModel
{

public:
	TGModel(void);
	TGModel(string tgdir, string tgfilename);
	~TGModel(void);

	void resizeAnnotations(cv::Size sz);
	string whatsAt(cv::Point pt, bool moreDetails); // returns the element in the point pt
	string isThereA(string query); // returns yes or no if the query element is in the TG
	string whatsThere(); //returns a list of all the elements in the TG
	inline cv::Size getSize() { return imageSize_; }

private:
	string tgTitle_;
	string tgType_;
	cv::Mat annotations_;
	map<int, FeatureInfo> features;
	void loadAnnotations(string filename, int rows, int cols);
	cv::Mat T_;
	vector<vector<cv::Point> > annotContours_;
	string num2string(int n); //takes a number 0-9 and returns the string (it should take care of any number eventually)
	cv::Size imageSize_;
};

