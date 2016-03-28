#pragma once

#include "opencv2\opencv.hpp"

class Lattice
{
public:
	Lattice();
	Lattice(int M, int N, int scale, int numFeat);
	Lattice(cv::Mat I, int  M, int N, int scale, int numFeat);

	~Lattice();
	void initializeMap(cv::Mat I);

	float findBestMatch(cv::Vec3f p, cv::Point2i& match);
	bool isShadow(cv::Vec3f p, cv::Point2i& pt, float beta, float gamma, float th, float ts);
	void update(cv::Point2i pt, cv::Vec3f c, float alpha, cv::Mat& W);
	void update(cv::Point2i pt, cv::Vec3f c, float alpha, float sigma);
	inline void finalizeUpdate(){ prevMap_ = map_; }


private:
	int scale_;
	int numFeat_;
	cv::Mat map_;
	cv::Mat prevMap_;
};

