#pragma once
#include "opencv2\opencv.hpp"
#include "Lattice.hpp"
class SOM
{
public:
	SOM(){};
	SOM(int M, int N, int scale, int numFeat, float eps1, float eps2, float c1, float c2, float sigma, long int K);
	SOM(cv::Mat I, int M, int N, int scale, int numFeat, float eps1, float eps2, float c1, float c2, float sigma, long int K);
	~SOM();
	cv::Mat detect(cv::Mat D, int t); ///< input image and time 
private:
	int M_;
	int N_;
	int n_;
	int numFeats_;
	float eps1_;
	float eps2_;
	float c1_;
	float c2_;
	float sigma_;
	cv::Mat W_;
	float alpha1_;
	float alpha2_;
	Lattice lattice_;
	long int K_;

};

