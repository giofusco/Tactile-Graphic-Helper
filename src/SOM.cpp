#include "SOM.hpp"



SOM::SOM(int M, int N, int scale, int numFeat, float eps1, float eps2, float c1, float c2, float sigma, long int K)
{
	M_ = M;
	N_ = N;
	n_ = scale;
	numFeats_ = numFeat;
	eps1_ = eps1;
	eps2_ = eps2;
	c1_ = c1;
	c2_ = c2;
	W_ = cv::getGaussianKernel(n_, sigma);
	cv::mulTransposed(W_, W_, false);
	double min = 0, max = 0;
	cv::minMaxLoc(W_,&min, &max);
	alpha1_ = c1_ / max;
	alpha2_ = c2_ / max;
	K_ = K;
	//alpha1_ = 1.;
	//alpha2_ = .8;
}

SOM::SOM(cv::Mat I, int M, int N, int scale, int numFeat, float eps1, float eps2, float c1, float c2, float sigma, long int K) : SOM(M, N, scale, numFeat, eps1, eps2, c1, c2, sigma, K)
{
	lattice_ = Lattice(I, M, N, scale, numFeat);
}

SOM::~SOM()
{
}

cv::Mat SOM::detect(cv::Mat D, int t){
	
	cv::Mat B = cv::Mat::zeros(D.size(),CV_8UC1);
	cv::Mat_<uchar> _B = B; //faster access to Mat

	float alpha_t = alpha1_ - t * ( (alpha1_ - alpha2_) / K_);
	cv::Point2i pt;
	cv::Mat dist = cv::Mat::zeros(D.size(), CV_32FC1);
	for (int x = 0; x < D.rows; ++x)
	{
		cv::Vec3f* pixel = D.ptr<cv::Vec3f>(x); // point to first pixel in row
	

		for (int y = 0; y < D.cols; ++y)
		{
			cv::Vec3f v = pixel[y]; //current pixel
			pt = cv::Point2i(x, y);
			float d = lattice_.findBestMatch(v, pt); //pt is an input/output variable
			dist.at<float>(x, y) = d;
			if (t <= K_ && d <= eps1_){ //calibration
				lattice_.update(pt,v, alpha_t, W_);
			}
			else if (t>K_ && d <= eps2_){	//online
				lattice_.update(pt, v, alpha2_, W_);
			}
			else if (!lattice_.isShadow(v, cv::Point2i(x, y), 1., .7, 10, 0.1))
				_B(x,y) = 255;	//foreground
			//else
			//	_B(x, y) = 120; //mark shadows 
		}
	}
	lattice_.finalizeUpdate();
	B = _B;
	return B;
}
