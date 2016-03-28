#include "Lattice.hpp"


Lattice::Lattice()
{
}

Lattice::Lattice(int M, int N, int scale, int numFeat){
	scale_ = scale;
	numFeat_ = numFeat;
	map_ = cv::Mat::zeros(cv::Size(N*scale, M*scale), CV_32FC3);
}

Lattice::Lattice(cv::Mat I, int  M, int N, int scale, int numFeat) : Lattice(M, N, scale, numFeat) {
	initializeMap(I);
}

void Lattice::initializeMap(cv::Mat I){

	assert(I.channels() == numFeat_ && I.depth() == CV_32F);

	cv::Mat_<cv::Vec3f> _map = map_; //faster access to Mat
	float h, s, v;
	for (int x = 0; x < I.rows; ++x)
	{
		cv::Vec3f* pixel = I.ptr<cv::Vec3f>(x); // point to first pixel in row
		for (int y = 0; y < I.cols; ++y)
		{
			h = pixel[y][0];
			s = pixel[y][1];
			v = pixel[y][2];

			//set the neighbours indexes: scale x scale
			for (int i = x*scale_; i <= scale_*(x + 1) - 1; i++){
				for (int j = y*scale_; j <= scale_*(y + 1) - 1; j++){
					assert(i >= 0 && j >= 0 && i < map_.size().height && j < map_.size().width); //boundary check
					_map(i, j)[0] = h;
					_map(i, j)[1] = s;
					_map(i, j)[2] = v;
					//	}
				}
			}

		}
	}
	map_ = _map;
	map_.copyTo(prevMap_);
}
 
float Lattice::findBestMatch(cv::Vec3f c, cv::Point2i& pt){ //pt is input/output
	float minDist = 1e10;

	int x = pt.x;
	int y = pt.y;

	//set the neighbours indexes: scale x scale
	for (int i = x*scale_; i <= scale_*(x + 1) - 1; i++){
		cv::Vec3f* pixel = map_.ptr<cv::Vec3f>(i); // point to first pixel in row
		for (int j = y*scale_; j <= scale_*(y + 1) - 1; j++){
			cv::Vec3f v = pixel[j]; //current pixel
			float d = sqrt(pow(c[2] * c[1] * std::cos(c[0]) - v[2] * v[1] * std::cos(v[0]), 2) +
				+pow(c[2] * c[1] * std::sin(c[0]) - v[2] * v[1] * std::sin(v[0]), 2) +
				+pow(c[2] - v[2], 2));
			if (d <= minDist){
				minDist = d;
				pt = cv::Point2i(i, j);
			}
		}
	}

	return minDist;
}


bool Lattice::isShadow(cv::Vec3f p, cv::Point2i& pt, float beta, float gamma, float th, float ts){
	int x = pt.x;
	int y = pt.y;
	bool shadow = false;
	//set the neighbours indexes: scale x scale
	for (int i = x*scale_; i <= scale_*(x + 1) - 1; i++){
		cv::Vec3f* pixel = map_.ptr<cv::Vec3f>(i); // point to first pixel in row
		for (int j = y*scale_; j <= scale_*(y + 1) - 1; j++){
			cv::Vec3f c = pixel[j]; //current pixel
			float t1 = p[2] / c[2]; 
			float t2 = p[1] - c[1];
			float t3 = abs(p[0] * 57.2957795131 - c[0] * 57.2957795131);
			if ((t1 >= gamma && t1 <= beta) && (t2 <= ts) && (t3 <= th))
				return true;
			
		}
	}
	return shadow;
}
//float Lattice::findBestMatch(cv::Vec3f c, cv::Point2i& pt){ 
//	float minDist = 1e10;
//	float degtorad = 0.01745329251;
//	for (int x = 0; x < map_.rows; ++x)
//	{
//		cv::Vec3f* pixel = map_.ptr<cv::Vec3f>(x); // point to first pixel in row
//		for (int y = 0; y < map_.cols; ++y)
//		{
//			cv::Vec3f v = pixel[y]; //current pixel
//			float d = sqrt(pow(c[2] * c[1] * std::cos(c[0] * degtorad) - v[2] * v[1] * std::cos(v[0] * degtorad), 2) + 
//				+pow(c[2] * c[1] * std::sin(c[0] * degtorad) - v[2] * v[1] * std::sin(v[0] * degtorad), 2) +
//				+pow(c[2] - v[2], 2));
//			if (d < minDist){
//				minDist = d;
//				pt = cv::Point2i(x, y);
//			}
//		}
//	}
//	return minDist;
//}

void Lattice::update(cv::Point2i pt, cv::Vec3f c, float alpha, cv::Mat& W){
	int x_base = pt.x - floor(numFeat_ / 2);
	int x_top = pt.x + floor(numFeat_ / 2);
	int y_base = pt.y - floor(numFeat_ / 2);
	int y_top = pt.y + floor(numFeat_ / 2);

	cv::Mat_<cv::Vec3f> _map = map_; //faster access to Mat

	int i = 0, j = 0;
	for (int x = x_base; x <= x_top; ++x)
	{
		if (x >= 0 && x < map_.size().height){ //boundary check on x
			cv::Vec3f* pixel = prevMap_.ptr<cv::Vec3f>(x); // point to first pixel in row
			double* w = W.ptr<double>(i); // current row of W
			for (int y = y_base; y <= y_top; ++y)
			{
				if (y >= 0 && y < map_.size().width){ //boundary check on y
					//cv::Vec3f tmp1 =  (1. - (alpha*w[j])) * pixel[y];
					//cv::Vec3f tmp2 = alpha*w[j] * c;
					cv::Vec3f newValue = (1. - (alpha*w[j])) * pixel[y] + alpha*w[j] * c;
					_map(x, y) = newValue;
				}
				j++;
			}
		}
		i++;
		j = 0;
	}
	map_ = _map;
}

Lattice::~Lattice()
{
}

void Lattice::update(cv::Point2i pt, cv::Vec3f c, float alpha, float sigma){
	int x_base = pt.x - floor(numFeat_ / 2);
	int x_top = pt.x + floor(numFeat_ / 2);
	int y_base = pt.y - floor(numFeat_ / 2);
	int y_top = pt.y + floor(numFeat_ / 2);

	cv::Mat_<cv::Vec3f> _map = map_; //faster access to Mat

	int i = -1, j = -1;
	for (int x = x_base; x <= x_top; ++x)
	{
		if (x >= 0 && x < map_.size().height){ //boundary check on x
			cv::Vec3f* pixel = prevMap_.ptr<cv::Vec3f>(x); // point to first pixel in row
			for (int y = y_base; y <= y_top; ++y)
			{
				if (y >= 0 && y < map_.size().width){ //boundary check on y
					//cv::Vec3f tmp1 =  (1. - (alpha*w[j])) * pixel[y];
					//cv::Vec3f tmp2 = alpha*w[j] * c;
					float w = expf(sqrt((i*i) + (j*j)) / (2 * sigma*sigma));
					cv::Vec3f newValue = (1. - (alpha*w)) * pixel[y] + alpha*w * c;
					_map(x, y) = newValue;
				}
				j++;
			}
		}
		i++;
		j = 0;
	}
	map_ = _map;
}