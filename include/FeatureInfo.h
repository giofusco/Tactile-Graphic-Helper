#pragma once
#include <string>
#include <stdio.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include "opencv2\highgui\highgui.hpp"

using namespace std;

class FeatureInfo
{
public:
	FeatureInfo(void);
	FeatureInfo(cv::FileNode fn);
	~FeatureInfo(void);

	int id;
	string title;
	string type;
	string style;
	string thickness;
	string purpose;
	string from;
	string to;
	int qty;
	
};

