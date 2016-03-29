#include "FingerTracker.h"
#define _USE_MATH_DEFINES // for C++
#include <cmath>

FingerTracker::FingerTracker(void)
{
	uchar diam2[5][5] = { { 0,0,1,0,0}, { 0,1,1,1,0} ,{ 1,1,1,1,1}, {0,1,1,1,0}, {0,0,1,0,0} }; 
	uchar diam3[7][7] = { {0, 0,0,1,0,0,0}, {0, 0,1,1,1,0,0} ,{0, 1,1,1,1,1,0}, {1, 1,1,1,1,1,1}, 
	{0,1,1,1,1,1,0}, {0, 0,1,1,1,0,0},{0, 0,0,1,0,0,0} }; 
	diamond2.create(5,5,CV_8UC1);
	diamond3.create(7,7,CV_8UC1);
	for (int r = 0; r < diamond2.rows;r++){
		for(int c =0; c < diamond2.cols; c++){
			diamond2.at<uchar>(r,c) = diam2[r][c];
		}
	}
	for (int r = 0; r < diamond3.rows;r++){
		for(int c =0; c < diamond3.cols; c++){
			diamond3.at<uchar>(r,c) = diam3[r][c];
		}
	}
}


FingerTracker::~FingerTracker(void)
{
}



float FingerTracker::getWeight(cv::Point p, int r, cv::Mat img) {
	float w = 0;

	for (int y = max(p.y - r, 0); y < min(p.y + r, img.rows); y++) {
		for (int x = max(p.x - r, 0); x < min(p.x + r, img.cols); x++) {
			if (RobustLine::distance(p, x, y) > r) continue;
			w += img.at<float>(y, x);
		}
	}
	return w;
}


void FingerTracker::track(cv::Mat hand, float adjustment){
	static int counter = 0;
	cv::Mat hand_filtered = hand.clone();

	/// Detect edges using Threshold
	cv::imshow("The Hand Filtered", hand_filtered);
	cv::moveWindow("The Hand Filtered", 10,10);
	cv::Mat hand_dist;
	distanceTransform(hand_filtered, hand_dist, CV_DIST_L2, 3);
	blur(hand_dist, hand_dist, cv::Size(3, 3));
	//imwrite(string("dt_"+to_string(counter)+".png"),hand_dist);
	counter++;
	cv::Mat dd;
	//normalize(hand_dist, dd, 0.0, 1.0, NORM_MINMAX);
	cv::imshow("dt of one hand", hand_dist);
	//cvMoveWindow("dt of one hand", 100, 100);


	float ridgeMin = 0.05; 
	float ridgeMax = 20.;
	cv::Mat bgrImg, dTrnsfrm;

	vector<cv::Point> vecPnt;
	vecPnt.clear();
	bgrImg = cv::Mat::zeros(hand.size(),CV_8UC3);
	cv::Mat tip = cv::Mat::zeros(hand.size(),CV_8UC3);
	cv::Mat_<float> &dt = (cv::Mat_<float> &)hand_dist;
	for (int y = 1; y < dt.rows - 1; y++)
		for (int x = 1; x < dt.cols - 1; x++) {
			if (dt(y, x) < ridgeMin || dt(y, x) > ridgeMax) continue;
			int cnt = isRidge(dt(y, x - 1), dt(y, x), dt(y, x + 1));
			cnt += isRidge(dt(y - 1, x), dt(y, x), dt(y + 1, x));
			cnt += isRidge(dt(y - 1, x - 1), dt(y, x), dt(y + 1, x + 1));
			cnt += isRidge(dt(y + 1, x - 1), dt(y, x), dt(y - 1, x + 1));
			//            if (cnt == 2 || cnt == 3){
			if (cnt > 1) {
				vecPnt.push_back(cv::Point(x, y));
				//circle(tip, Point(x, y), 1, Scalar(255));
			}
		}


		vector<cv::Point> tmp = vecPnt; // save ridge point
		vector<RobustLine> vLine;
		RobustLine::initAllLines(vLine, vecPnt);
		RobustLine::drawData(vLine, bgrImg);
		RobustLine::getFitteLines(vLine);
		RobustLine::tryGroupAll(vLine);
		RobustLine::drawAllLines(vLine, bgrImg);
		cv::imshow("All lines", bgrImg);
		//vFinger.clear();
		vector<cv::Point> candidateTips;
		cv::Point fingertip;
		//cv::Mat tip;
		//backgroundImage_.copyTo(tip);
		for (int i = 0; i < vLine.size(); i++) {
			
			if (vLine[i].length() > 10){
				cv::Point p1, p2;
				p1 = vLine[i].p1;
				p2 = vLine[i].p2;
				float dt1 = getWeight(p1, 20, dt);
				float dt2 = getWeight(p2, 20, dt);
				//    cout<<"Finger::setTip(), dt1, dt2 = "<<dt1<<", "<<dt2<<endl;
				cv::Point p(p1);
				cv::Point tail(p2);
				int dx = p1.x - p2.x, dy = p1.y - p2.y;

				if (dt2 < dt1) {
					p = p2;
					tail = p1;
					dx = -dx;
					dy = -dy;
				}
				float deltx, delty;
				if (abs(dx) > abs(dy)) {
					deltx = dx < 0 ? -1 : 1;
					delty = deltx * dy / dx; 
				} else {
					delty = dy < 0 ? -1 : 1;
					deltx = delty * dx / dy;
				}
				float x = p.x, y = p.y;
				int cnt = max(abs(dx), abs(dy));

				if (y < dt.rows && x < dt.cols){
					float tol = dt.at<float>(y, x)*0.25;
					for (int ii = 0; ii < cnt; ii++, x += deltx, y += delty){
						if (y < dt.rows && x < dt.cols && x>=0 && y>=0){
							//cerr << "[] x: " << x << ", " << y <<endl;
							if (dt.at<float>(y, x) <= tol) {
				//				cerr << "[] x: " << x << ", " << y << ", " << dt.at<float>(y, x) << ", " << tol << endl;
								break;
							}
						}
					}
				}
				 
				fingertip.x = x;
				fingertip.y = y;
				cv::Point diff = fingertip - tail;
				int dir;
				if (diff.x < 0 || diff.y <0)
					dir = 1; //vector pointing towards the origin (top,left corner of the image)
				else dir = -1; //vector pointing away from the origin

				float angle = atan2f(dy,dx);
				//	cerr << "FingerTip: " << fingertip << ", Tail: " << tail << " m = " << float(-dy)/float(dx) << " Angle: " << angle*180/3.14 << endl;
				cv::Point rectTip;
				rectTip.x = fingertip.x - adjustment * dir * cos(angle);
				rectTip.y = fingertip.y - adjustment * dir * sin(angle);
				if (rectTip.x > 5 && rectTip.x < hand.size().width - 5 &&  rectTip.y > 5 && rectTip.y < hand.size().height - 5)
					candidateTips.push_back(rectTip);

				circle(tip,fingertip,10, cv::Scalar(255,0,0),1);
				circle(tip,rectTip,5, cv::Scalar(255,255,0),1);
				line(tip,vLine[i].p1,vLine[i].p2,cv::Scalar(255,0,0),1);

			} //else cerr << vLine[i].length() << endl;
		}
		vecPnt = tmp; // put back ridge points
		//cv::imshow("Raw output", tip);

		//cerr << "CANDIDATE TIPS SIZE: " << candidateTips.size() << endl;
		//merge fingertips with previous observations
		if (tips.size() == 0){ //if no tips have been seen ever, push everything
			//cerr << "Creating new tips" << endl;
			for (int t = 0; t< candidateTips.size(); t++){
				//Fingertip ftip = Fingertip(candidateTips[t],5);
				//cerr << ".";
				tips.push_back( Fingertip(candidateTips[t],5));
			}
		}
		else{
			vector<bool> updatedTips(tips.size());
			for (int ct = 0; ct< candidateTips.size(); ct++){
				cv::Point atip = candidateTips[ct];
				bool ins = false;
				for (int t=0; t< tips.size(); t++){
					ins = tips[t].addPoint(atip,35.);
					if (ins) 
						break;
				}
				if (!ins){ //no candidate tips found
					tips.push_back( Fingertip(candidateTips[ct],5));
				}

			}
			//check which tips we haven't seen, and update the counter of missed frames;
			for (int u=0;u<updatedTips.size();u++){
				if (!updatedTips[u])
					tips[u].framesSinceLastSeen += 1;
			}
		}

		////check if it is time to prune some tips
		vector<Fingertip>::iterator it;
		for (it=tips.begin(); it!=tips.end();){
			if (!it->isValid()){
				it = tips.erase(it);
			}
			else 
				++it;
		}
}

vector<cv::Point> FingerTracker::getLastSeen(int numFrames){
	vector<Fingertip>::iterator it;
	vector<cv::Point> lastTips;
	for (it = tips.begin(); it!= tips.end(); ++it){
		if (it->framesSinceLastSeen <= 1 && it->isStable())
			lastTips.push_back(it->getPosition(numFrames));
	}
	return lastTips;
}

vector<Fingertip> FingerTracker::getLastSeenTip(int numFrames){
	vector<Fingertip>::iterator it;
	vector<Fingertip> lastTips;
	for (it = tips.begin(); it != tips.end(); ++it){
		if (it->framesSinceLastSeen <= 1 && it->isStable())
			lastTips.push_back(*it);
	}
	return lastTips;
}

void FingerTracker::showTips(cv::Mat image){

	for (int t=0;t<tips.size();t++)
		if (tips[t].isValid() && tips[t].isStable()){
			tips[t].drawTip(image);
		} 

		cv::imshow("Filtered Tips", image);
		//moveWindow("Filtered Tips", 10,10);
}