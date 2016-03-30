#include "BackgroundGenerator.h"


BackgroundGenerator::~BackgroundGenerator(void)
{
}


cv::Mat BackgroundGenerator::getForegroundMask_SOM(cv::Mat frame){
	cv::Mat tmp, hsvFrame;
	warpPerspective(frame, backgroundImage_, H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
//	backgroundImage_.convertTo(tmp, CV_32FC3);
	frame.convertTo(tmp, CV_32FC3);
	tmp *= 1. / 255;
	cv::cvtColor(tmp, hsvFrame, CV_BGR2HSV);
	std::vector<cv::Mat> channels;
	//std::vector<cv::Mat> lab_channels;
	cv::split(hsvFrame, channels);
	channels[0] *= 0.01745329251; //convert hue to radians
	cv::merge(channels, hsvFrame);
	cv::medianBlur(hsvFrame, hsvFrame, 3);
	fcnt++;
	cv::Mat B = som_.detect(hsvFrame, fcnt);
	//cv::medianBlur(B, B, 3);
	erode(B, B, rect3x3);
	warpPerspective(B, B, H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
	dilate(B, B, rect3x3);
	erode(B, B, rect3x3);
	cv::medianBlur(B, B, 3);
	return B;
}


void BackgroundGenerator::computeBackground_SOM(int numFrames, FrameGrabber* grabber){
	findHomographyUsingCorners(grabber, 10);
	cv::Mat frame;
	
	//## modified 3/11/16
	
	//grabber->getCurrentFrame().copyTo(frame);
	//warpPerspective(frame, backgroundImage_, H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify

	grabber->getCurrentFrame().copyTo(backgroundImage_);

	cv::Mat tmp, hsvFrame;
	backgroundImage_.convertTo(tmp, CV_32FC3);
	tmp *= 1. / 255;
	cv::cvtColor(tmp, hsvFrame, CV_BGR2HSV);
	std::vector<cv::Mat> channels;
	//std::vector<cv::Mat> lab_channels;
	cv::split(hsvFrame, channels);
	channels[0] *= 0.01745329251; //convert hue to radians
	cv::merge(channels, hsvFrame);


	som_ = SOM(hsvFrame, hsvFrame.size().height, hsvFrame.size().width, 3, 3, .15, .1, 1., .001, .5, numFrames); // init SOM
	while (fcnt < numFrames){

		grabber->getCurrentFrame().copyTo(backgroundImage_);
		//grabber->getCurrentFrame().copyTo(frame);
		//warpPerspective(frame, backgroundImage_, H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
		backgroundImage_.convertTo(tmp, CV_32FC3);
		tmp *= 1. / 255;
		cv::cvtColor(tmp, hsvFrame, CV_BGR2HSV);
		std::vector<cv::Mat> channels;
		//std::vector<cv::Mat> lab_channels;
		cv::split(hsvFrame, channels);
		channels[0] *= 0.01745329251; //convert hue to radians
		cv::merge(channels, hsvFrame);
		cv::medianBlur(hsvFrame, hsvFrame, 3);
		fcnt++;
		cv::Mat B = som_.detect(hsvFrame, fcnt);
	}
} 


void BackgroundGenerator::sortCorners(std::vector<cv::Point>& corners, cv::Point center)
{
	std::vector<cv::Point2f> top, bot;

	for (int i = 0; i < corners.size(); i++)
	{
		if (corners[i].y < center.y)
			top.push_back(corners[i]);
		else
			bot.push_back(corners[i]);
	}

	cv::Point tl = top[0].x > top[1].x ? top[1] : top[0];
	cv::Point tr = top[0].x > top[1].x ? top[0] : top[1];
	cv::Point bl = bot[0].x > bot[1].x ? bot[1] : bot[0];
	cv::Point br = bot[0].x > bot[1].x ? bot[0] : bot[1];

	corners.clear();
	corners.push_back(tl);
	corners.push_back(tr);
	corners.push_back(br);
	corners.push_back(bl);
}


//sets homography matrix H_ looking at the paper corners
void BackgroundGenerator::findHomographyUsingCorners(FrameGrabber* grabber, int numFrames){ 
	cv::Mat frame;
	cv::Mat edges, gframe;
	markerCenter_.clear();
	numFrames = 1;
	//cout << "Creating background model and rectification matrix... ";
	//waitKey(3000); //wait 3 seconds (is it necessary?)
	try{
		int fcnt = 0;
		int skippedFrames = 0;
		vector< vector <cv::Point2f> > corners;
		for (int i=0;i<4;i++)
			corners.push_back(vector<cv::Point2f>());

		cv::Mat avg;
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy2;
		while (fcnt < numFrames && skippedFrames < 2*numFrames){
		//bool done = false;
		//while (!done){
			grabber->getCurrentFrame().copyTo(frame);
			//uframe = frame.clone();
			
			cvtColor(frame,gframe,CV_BGR2GRAY);
			//blur(gframe,gframe, cv::Size(3,3));

			cv::Mat foo;
			double otsu_thresh_val = cv::threshold(
				gframe, foo, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU
				);
			double high_thresh_val = otsu_thresh_val,
				lower_thresh_val = otsu_thresh_val * 0.5;
			cv::Canny(gframe, edges, lower_thresh_val, high_thresh_val);

			//Canny(gframe,edges, 100, 100, 3, true);
			imshow("Canny", edges);

			

			cv::findContours ( edges , contours, hierarchy2,CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE );
			cv::drawContours(frame,contours,-1,cv::Scalar(255,0,0),1);
			cv::imshow("Edges", frame);
			vector<cv::Point>  approxCurve;
			//find longest countour
			int maxLen=0, maxid=-1;
			for ( unsigned int i=0;i<contours.size();i++ ){
				if (contours[i].size() > maxLen){
					approxPolyDP (  contours[i]  ,approxCurve , double (contours[i].size() ) *0.05 , true );
					//approxPolyDP (  contours[i]  ,approxCurve , 1e-5, true );
					if ( approxCurve.size() == 4 && cv::isContourConvex(cv::Mat(approxCurve)) ){
						maxLen =  contours[i].size();
						maxid = i;
					}
				}

			}

			if (maxid > -1){
				
				drawContours(frame,contours,maxid, cv::Scalar(255,0,0),1);
				imshow("Longest Contour", frame);
				
				cv::approxPolyDP (  contours[maxid]  ,approxCurve , double (contours[maxid].size() ) *0.05 , true );
				
				for ( unsigned int i=0;i<approxCurve.size();i++ )
					cv::line ( frame ,approxCurve[i],approxCurve[ ( i+1 ) %approxCurve.size() ], cv::Scalar(255,255,0),2 );

				// Get mass center
				cv::Point center(0,0);
				for (int j = 0; j < approxCurve.size(); j++)
					center += approxCurve[j];

				center *= (1. / approxCurve.size());
				sortCorners(approxCurve, center);
				for (int cc=0;cc<4;cc++)
					corners[cc].push_back(approxCurve[cc]);
				fcnt++;
				//	cerr << "FRAMES: " << numFrames << endl;
			}
			else skippedFrames++;
		}
		if (fcnt > 0){
			vector<cv::Point2f> tmpC;
			for (int c=0;c<4;c++){
				float meanx = 0;
				float meany = 0;
				for (int p=0; p<fcnt;p++){
					meanx += corners[c][p].x;
					meany += corners[c][p].y;
				}
				tmpC.push_back(cv::Point2f(meanx / fcnt, meany / fcnt));
			}

			markerCenter_.push_back(tmpC[2]);
			markerCenter_.push_back(tmpC[1]);
			markerCenter_.push_back(tmpC[0]);

			markerCenter_.push_back(tmpC[3]);



			circle(frame,markerCenter_[0],3, cv::Scalar(0,0,255),2);
			circle(frame,markerCenter_[1],3, cv::Scalar(0,0,255),2);
			circle(frame,markerCenter_[2],3, cv::Scalar(0,0,255),2);
			circle(frame,markerCenter_[3],3, cv::Scalar(0,0,255),2);

			imshow("Points", frame);

			H_ = findHomography(markerCenter_,rectifiedPoints_,0); //compute homography once


		}
		else throw(-2);
	}
	catch(std::exception& e)
	{
		std::cerr << "Error: " << e.what() << "\n";
		//return -1;
	}

	catch (int e){
		if (e==-1)
			std::cerr << "Error opening the video stream.";
	}
}


cv::Mat BackgroundGenerator::getMean(vector<cv::Mat> stack, cv::Mat &std){
	cv::Size dim = stack[0].size();
	cv::Mat Mean;
	Mean.create(dim, CV_8UC1);
	//cerr << "Size stack: " << stack.size() << endl;
	// Probably highly inefficient 
	for (int r=0;r< stack[0].rows;r++){
		for(int c=0;c< stack[0].cols;c++){
			vector<uchar> tmp;
			for (int im=0;im<stack.size();im++){
				//tmp.push_back(float(static_cast<float>(stack[im].at<float>(r,c))));
				tmp.push_back((stack[im].at<uchar>(r,c)));
				//cerr << stack[im].at<float>(r,c) << endl;

			}
			
			cv::Mat m,s;
			meanStdDev(tmp,m,s);
			cerr << "M: " << m.at<float>(0,0) << endl;
			cerr << "S: " << s << endl;
			float mm = m.at<float>(0,0);
			float ss = s.at<float>(0,0);
			Mean.at<uchar>(r,c) = uchar(mm);
			std.at<uchar>(r,c) = (uchar)(ss); 
		//	cerr <<  Mean.at<float>(r,c) << endl;
		}
	}
	//imshow("Mean", Mean);
	return Mean;
}


cv::Mat BackgroundGenerator::getMedianImage(vector<cv::Mat> stack){
	cv::Size dim = stack[0].size();
	cv::Mat median;
	median.create(dim,CV_32FC1);

	// Probably highly inefficient 
	for (int r=0;r< stack[0].rows;r++){
		for(int c=0;c< stack[0].cols;c++){
			vector<float> tmp;
			for (int im=0;im<stack.size();im++){
				//tmp.push_back(float(static_cast<float>(stack[im].at<float>(r,c))));
				tmp.push_back((stack[im].at<float>(r,c)));
				//cerr << stack[im].at<float>(r,c) << endl;

			}
			sort(tmp.begin(), tmp.end());

			float tmpmedian;
			if (tmp.size()  % 2 == 0)
				tmpmedian = (tmp[tmp.size() / 2 - 1] + tmp[tmp.size() / 2]) / 2;
			else 
				tmpmedian = tmp[tmp.size() / 2];
			median.at<float>(r,c)  = tmpmedian;

			//cerr << tmpmedian << endl;
		}
	}
	//imshow("Median", median);
	return median;
}



cv::Mat BackgroundGenerator::metricallyTrimmedMean(vector<cv::Mat> stack, cv::Mat median, float alpha){

	vector<cv::Mat> q;
	float T = floor( (1-alpha)*stack.size()+.5);

	for (int i=0; i<stack.size();i++){
		q.push_back(cv::abs(median - stack[i]));
		//imwrite(String(to_string(i)+".bmp"),q[i]);
	}

	cv::Mat y;
	y.create(stack[0].size(),CV_32FC1);
	for (int r=0;r< stack[0].rows;r++){
		for(int c=0;c< stack[0].cols;c++){
			vector<float> tmp;
			for (int im=0;im<stack.size();im++){
				//tmp.push_back(float(static_cast<float>(stack[im].at<float>(r,c))));
				tmp.push_back((q[im].at<float>(r,c)));
				//cerr << stack[im].at<float>(r,c) << endl;

			}
			sort(tmp.begin(), tmp.end());

			float avg = 0;
			for(int m=0;m<T;m++)
				avg += tmp[m];

			y.at<float>(r,c) = avg/T;
		}
	}
	return y;
}

cv::Mat BackgroundGenerator::computeMAD(vector<cv::Mat> stack, cv::Mat median){
	vector<cv::Mat> stack2;
	for (int i=0; i<stack.size();i++){
		stack2.push_back(cv::abs(median - stack[i]));
	}
	cv::Mat madd = getMedianImage(stack2);
	for (int r=0; r<madd.rows;r++){
		for (int c=0; c<madd.cols;c++){
			if (madd.at<float>(r,c) == 0)
				madd.at<float>(r,c) = 0.5;
		}
	}

	return madd;

}

void BackgroundGenerator::computeBackground_TM09(int numFrames, FrameGrabber* grabber, float alpha ){
	findHomographyUsingCorners(grabber, 10);
	vector<cv::Mat> stack;  //stack of frames used to generate the background
	cv::Mat frame;
	for (int fcnt=0; fcnt < numFrames; fcnt++){
		grabber->getCurrentFrame().copyTo(frame);
		cv::cvtColor(frame,frame,CV_BGR2GRAY);
		warpPerspective(frame,frame,H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
		//stack.push_back(frame);
		cv::Mat tmp;
		frame.convertTo(tmp,CV_32FC1,1.);
		stack.push_back(tmp);
	}
	
	grabber->getCurrentFrame().copyTo(frame);
	warpPerspective(frame,backgroundImage_,H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
	//cerr << "BG Size: " << backgroundImage_.size() << endl;
	//imshow("BG", backgroundImage_);

	W_ = getMedianImage(stack);

	//cerr << "out of median" << endl;
	cv::Mat med = metricallyTrimmedMean(stack, W_, alpha);
	//cerr << "out of mean" << endl;
	cv::Mat madd = computeMAD(stack, med);
	//imshow("MADD", madd);
	//imwrite("madd.png",madd);
	//Mat std_dev = 1.4826*madd;
	ST_ = 1.4826*madd;
	////find mode of std_dev


	cv::Mat tmpW = W_ + 0.0001;
	cv::Mat tmpST, tmpST2;
	float smask = 16; //sum of the coefficient of the mask

	divide(ST_,tmpW,tmpST,CV_32FC1);
	multiply(tmpST,tmpST,tmpST2,CV_32FC1);
	filter2D(tmpST2,std2_,CV_32FC1, mask_std);

	tmpST = ST_ + 1.01;
	tmpST /= smask;

	filter2D(tmpST,st2_,CV_32FC1, mask);
}

void BackgroundGenerator::initFilters(){

	rect3x3 = cv::getStructuringElement(cv::MORPH_RECT,
		cv::Size(3, 3),
		cv::Point(1, 1));

	float mask_std_tmp[5][5] = { {0, 0, 1, 0, 0}, { 0, 1, 1, 1, 0 }, {1, 1, 144, 1, 1,}, {0, 1, 1, 1, 0}, {0, 0, 1, 0, 0} };
	float mask_tmp[3][3] = { {1, 2, 1}, {4,2,2}, {1,2,1} };
	mask_std.create(5,5,CV_32FC1);
	mask.create(3,3,CV_32FC1);
	float div = 13*13;
	for (int r = 0; r < mask_std.rows;r++){
		for(int c =0; c < mask_std.cols; c++){
			mask_std.at<float>(r,c) = mask_std_tmp[r][c] / div;
		}
	}
	for (int r = 0; r < mask.rows;r++){
		for(int c =0; c < mask.cols; c++){
			mask.at<float>(r,c) = mask_tmp[r][c] / div;
		}
	}
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


cv::Mat BackgroundGenerator::getForegroundMaskHue(cv::Mat frame, float tol){
	cv::Mat fore;
	cv::Mat rect_hsv;
	warpPerspective(frame,frame,H_,cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
	cvtColor(frame,rect_hsv,CV_BGR2HSV);
	cv::Mat rchannel[3];
	split(rect_hsv, rchannel);
	//rchannel[0].convertTo(rchannel[0],CV_32FC1);
	absdiff(rchannel[0],Hue_,fore);
	cv::Mat foremask;
	inRange(fore,tol,360,foremask);
	imshow("ForeMask", fore);
	cv::Mat fore_hue;
	fore.copyTo(fore_hue,foremask);

	cv::Mat hist;
	float range[] = { 0,  360} ;
	const float* histRange = { range };
	int histsize = 361;
	calcHist(&fore_hue,1,0,cv::Mat(),hist,1,&histsize, &histRange, true, false);
	
	//skip hue = 0 (background?)
	int max = 0;
	int maxp = -1;
	for (int hh = 1; hh < hist.total(); hh++){
		if (hist.at<int>(0,hh) > max){
			max = hist.at<int>(0,hh) ;
			maxp = hh;
		}

	}

	//cerr << "Backgr hue: " << bgmaxp << " Handpeak: " <<  maxp << endl;
	cv::Mat hand = cv::Mat::zeros(frame.size(),CV_8UC1);
	//cerr << "Foreground Peak Hue: " << maxp << endl;
	
	//check if it is random foreground
	if ( abs(maxp-lastHue_) <= 5)
		consecutiveHue_++;
	else consecutiveHue_ = 0;
	lastHue_ = maxp;
	if (maxp >= 0 && consecutiveHue_ > 10)
		inRange(fore_hue,maxp-11,maxp+11,hand);

	medianBlur(hand,hand,7);


	//imshow("Hand", hand);
	return hand;
}

//cv::Mat BackgroundGenerator::getForegroundMask_TM09(cv::Mat frame, float k){
//	
//	warpPerspective(frame,frame,H_,Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
//		
//	//imshow("Rectified image", frame);
//
//
//	cv::Mat rect_hsv; 
//	
//	cvtColor(frame,rect_hsv,CV_BGR2HSV);
//	cv::Mat rchannel[3];
//
//	split(rect_hsv, rchannel);
//	medianBlur(rchannel[0],rchannel[0],7);
//	imshow("Hue Orig", rchannel[0]);
//	moveWindow("Hue Orig", 300 , 10);
//
//	float range[] = { 0,  180} ;
//	const float* histRange = { range };
//	int histsize = 181;
//	cv::Mat histBG;
//	
//	calcHist(&rchannel[0],1,0,cv::Mat(),histBG,1,&histsize, &histRange, true, false);
//	//calcHist(&bgchannel[0],1,0,cv::Mat(),bghist,1,&histsize, &histRange, true, false);
//	//skip hue = 0 (background?)
//	long int maxBG = 0;
//	int maxpBG = -1;
//	for (int hh = 1; hh < histBG.total(); hh++){
//		//cerr << hist.at<float>(hh,0) << " " ;
//		if (histBG.at<int>(hh,0) > maxBG){
//			maxBG = histBG.at<int>(hh,0) ;
//			maxpBG = hh;
//		}
//
//	}
//	//cerr << "BACKGROUND Peak Hue: " << maxpBG << endl;
//
//
//	cv::Mat frame_copy;
//	frame.copyTo(frame_copy);
//	cvtColor(frame,frame,CV_BGR2GRAY);
//	frame.convertTo(frame,CV_32FC1,1.);
//
//	cv::Mat bw, d1;
//
//	filter2D(abs(frame-W_),bw,CV_32FC1,mask);
//
//
//	k = 3;
//	d1.create(bw.size(),CV_8UC1);
//	for (int r=0;r<bw.rows;r++){
//		for (int c=0;c<bw.cols;c++){
//			if (fabs(bw.at<float>(r,c))>float(k)*st2_.at<float>(r,c))
//				d1.at<uchar>(r,c) = 255;
//			else d1.at<uchar>(r,c) = 0;
//		}
//	}
//	morphologyEx(d1,d1,CV_MOP_OPEN,diamond2);
//	morphologyEx(d1,d1,CV_MOP_CLOSE,diamond3);
//
//	cv::Mat R, media;
//	divide(frame+0.00001,W_+0.00001,R);
//	medianBlur(R, media,5);
//	cv::Mat shadows = cv::Mat::zeros(frame.size(),CV_8UC1);
//	float kshad = 3.;
//	for (int r=0;r<R.rows;r++){
//		for (int c=0;c<R.cols;c++){
//			if ( ( (R.at<float>(r,c)-media.at<float>(r,c))*(R.at<float>(r,c)-media.at<float>(r,c))) < kshad*kshad*std2_.at<float>(r,c))
//				shadows.at<uchar>(r,c) = 255;
//		}
//	}
//	cv::Mat shad_filt;
//	medianBlur(shadows,shad_filt,5);
//	cv::Mat y; 
//	bitwise_and(shadows,shad_filt,y);
//	bitwise_and(y,d1,y);
//	float Tlow = .7, Thigh = 1.;
//	cv::Mat ys = cv::Mat::zeros(y.size(),CV_8UC1);
//	cv::Mat yh = cv::Mat::zeros(y.size(),CV_8UC1);
//	cv::Mat foreground = cv::Mat::zeros(y.size(),CV_8UC1);
//	for (int r=0;r<ys.rows;r++){
//		for (int c=0;c<ys.cols;c++){
//			if (media.at<float>(r,c)>Tlow && media.at<float>(r,c) < 1 && y.at<uchar>(r,c)==255){
//				ys.at<uchar>(r,c) = 255;
//			}
//			else if (d1.at<uchar>(r,c) ==255){
//				foreground.at<uchar>(r,c) = 255;
//			}
//			if (media.at<float>(r,c)<Thigh && media.at<float>(r,c) > 1 && y.at<uchar>(r,c)==255)
//				yh.at<uchar>(r,c) = 255;
//		}
//	}
//	cv::Mat holes;
//	foreground.copyTo(holes);
//	floodFill(holes,Point(0,0),255);
//	for (int r=0;r<holes.rows;r++){
//		for (int c=0;c<holes.cols;c++){
//			if (255 - holes.at<uchar>(r,c) == 255)
//				foreground.at<uchar>(r,c) = 255;
//			if (ys.at<uchar>(r,c) == 255 && foreground.at<uchar>(r,c))
//				ys.at<uchar>(r,c) = 0;
//		}
//	}
//
//	cv::Mat yall;
//	bitwise_or(ys,yh,yall);
//
//	cv::Mat d3;
//	bitwise_and(d1,255-yall,d3);
//	morphologyEx(d3,d3,CV_MOP_OPEN,diamond2);
//	morphologyEx(d3,d3,CV_MOP_CLOSE,diamond3);
//	cv::Mat masked;
//	frame_copy.copyTo(masked,d3);
//
//	cv::Mat masked_hsv;
//	cvtColor(masked,masked_hsv,CV_BGR2HSV);
//	cv::Mat channel[3];
//	split(masked_hsv, channel);
//
//
//	MatND hist; //, bghist;
//	cv::Mat bghsv;
//	cvtColor(backgroundImage_,bghsv,CV_BGR2HSV);
//	cv::Mat bgchannel[3];
//	split(bghsv, bgchannel);
//
//	//float range[] = { 0,  180} ;
//	//const float* histRange = { range };
//	//int histsize = 181;
//	
//	calcHist(&channel[0],1,0,cv::Mat(),hist,1,&histsize, &histRange, true, false);
//	//calcHist(&bgchannel[0],1,0,cv::Mat(),bghist,1,&histsize, &histRange, true, false);
//	//skip hue = 0 (background?)
//	long int max = 0;
//	int maxp = -1;
//	for (int hh = 1; hh < hist.total(); hh++){
//		//cerr << hist.at<float>(hh,0) << " " ;
//		if (hist.at<int>(hh,0) > max){
//			max = hist.at<int>(hh,0) ;
//			maxp = hh;
//		}
//
//	}
//	//cerr << endl;
//	//imshow("BG HUE", bgchannel[0]);
//	/*int bgmax = 0;
//	int bgmaxp = -1;
//	for (int hh = 1; hh < bghist.total(); hh++){
//		if (bghist.at<int>(0,hh) > bgmax){
//			bgmax = bghist.at<int>(0,hh) ;
//			bgmaxp = hh;
//		}
//
//	}*/
//
//	//cerr << "Backgr hue: " << bgmaxp << " Handpeak: " <<  maxp << endl;
//	cv::Mat hand = cv::Mat::zeros(frame.size(),CV_8UC1);
//	//cerr << "Foreground Peak Hue: " << maxp << endl;
//	//cerr << "Diff in HUE: " << (maxp-maxpBG) << endl;
//	int hdiff = maxpBG - maxp;
//	int huplim = 11;
//	int hlowlim = -11;
//	if (hdiff > 0 && hdiff <20 )
//		huplim = int(hdiff * .25);
//	else if (hdiff < 0 && hdiff > -20 ) 
//		hlowlim = int(hdiff * .25);
//
//	//cerr << "low: " << hlowlim << ", up: " << huplim << endl;
//
//	//check if it is random foreground
//	if ( abs(maxp-lastHue_) <= 5 && hdiff > 5) /// IT WAS 5
//		consecutiveHue_++;
//	else consecutiveHue_ = 0;
//	lastHue_ = maxp;
//	if (maxp >= 0 && consecutiveHue_ > 10){
//		if (maxp+hlowlim < 1)
//			inRange(rchannel[0],1,maxp+huplim,hand);
//		else 
//			inRange(rchannel[0],maxp+hlowlim,maxp+huplim,hand);
//		//if (maxp - 11 < 5)
//			//inRange(rchannel[0],maxp,maxp+11,hand);
//		//else 
//			//inRange(rchannel[0],maxp+hlowlim,maxp+huplim,hand);
//	}
//
//	medianBlur(hand,hand,7);
//
//	//imshow("Hue Fore", channel[0]);
//	imshow("Hand", hand);
//	//int c = waitKey(10); //menu input
//	//if (char(c) == 'w'){
//	//	cerr << "Dumping HUE to file." << endl;
//	//	cv::FileStorage file("myhue.xml", cv::FileStorage::WRITE);
//	//	file << "hue" << channel[0];
//	//}
//	return hand;
//}

cv::Mat BackgroundGenerator::getForegroundMask_TM09(cv::Mat frame, float k){

	warpPerspective(frame, frame, H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify

	//imshow("Rectified image", frame);
	
	cv::Mat frame_copy;
	frame.copyTo(frame_copy);
	cvtColor(frame, frame, CV_BGR2GRAY);
	frame.convertTo(frame, CV_32FC1, 1.);

	cv::Mat bw, d1;

	filter2D(abs(frame - W_), bw, CV_32FC1, mask);


	k = 3;
	d1.create(bw.size(), CV_8UC1);
	for (int r = 0; r<bw.rows; r++){
		for (int c = 0; c<bw.cols; c++){
			if (fabs(bw.at<float>(r, c))>float(k)*st2_.at<float>(r, c))
				d1.at<uchar>(r, c) = 255;
			else d1.at<uchar>(r, c) = 0;
		} 
	}
	morphologyEx(d1, d1, CV_MOP_OPEN, diamond2);
	morphologyEx(d1, d1, CV_MOP_CLOSE, diamond3);

	cv::Mat R, media;
	divide(frame + 0.00001, W_ + 0.00001, R);
	medianBlur(R, media, 5);
	cv::Mat shadows = cv::Mat::zeros(frame.size(), CV_8UC1);
	float kshad = 3.;
	for (int r = 0; r<R.rows; r++){
		for (int c = 0; c<R.cols; c++){
			if (((R.at<float>(r, c) - media.at<float>(r, c))*(R.at<float>(r, c) - media.at<float>(r, c))) < kshad*kshad*std2_.at<float>(r, c))
				shadows.at<uchar>(r, c) = 255;
		}
	}
	cv::Mat shad_filt;
	medianBlur(shadows, shad_filt, 5);
	cv::Mat y;
	bitwise_and(shadows, shad_filt, y);
	bitwise_and(y, d1, y);
	float Tlow = .7, Thigh = 1.;
	cv::Mat ys = cv::Mat::zeros(y.size(), CV_8UC1);
	cv::Mat yh = cv::Mat::zeros(y.size(), CV_8UC1);
	cv::Mat foreground = cv::Mat::zeros(y.size(), CV_8UC1);
	for (int r = 0; r<ys.rows; r++){
		for (int c = 0; c<ys.cols; c++){
			if (media.at<float>(r, c)>Tlow && media.at<float>(r, c) < 1 && y.at<uchar>(r, c) == 255){
				ys.at<uchar>(r, c) = 255;
			}
			else if (d1.at<uchar>(r, c) == 255){
				foreground.at<uchar>(r, c) = 255;
			}
			if (media.at<float>(r, c)<Thigh && media.at<float>(r, c) > 1 && y.at<uchar>(r, c) == 255)
				yh.at<uchar>(r, c) = 255;
		}
	}
	cv::Mat holes;
	foreground.copyTo(holes);
	floodFill(holes, cv::Point(0, 0), 255);
	for (int r = 0; r<holes.rows; r++){
		for (int c = 0; c<holes.cols; c++){
			if (255 - holes.at<uchar>(r, c) == 255)
				foreground.at<uchar>(r, c) = 255;
			if (ys.at<uchar>(r, c) == 255 && foreground.at<uchar>(r, c))
				ys.at<uchar>(r, c) = 0;
		}
	}

	cv::Mat yall;
	bitwise_or(ys, yh, yall);

	cv::Mat d3;
	bitwise_and(d1, 255 - yall, d3);
	morphologyEx(d3, d3, CV_MOP_OPEN, diamond2);
	morphologyEx(d3, d3, CV_MOP_CLOSE, diamond3);
	cv::Mat hand;

	//frame_copy.copyTo(hand, d3);
	//cvtColor(hand, hand, CV_BGR2GRAY);
	
	cv::MatND hist, bghist;
	cv::Mat bghsv;
	cvtColor(frame_copy,bghsv,CV_BGR2HSV_FULL);
	cv::Mat channel[3];
	split(bghsv, channel);
	
	imshow("Hue Map", channel[0]);


	float range[] = { 0,  255} ;
	const float* histRange = { range };
	int histsize = 256;
	medianBlur(channel[0],channel[0],7);
	calcHist(&channel[0],1,0,d3,hist,1,&histsize, &histRange, true, false);
	//calcHist(&bgchannel[0],1,0,cv::Mat(),bghist,1,&histsize, &histRange, true, false);
	//skip hue = 0 (background?)
	long int max = 0;
	int maxp = -1;
	for (int hh = 1; hh < hist.total(); hh++){
		cerr << hist.at<float>(hh,0) << " " ;
		if (hist.at<int>(hh,0) > max){
			max = hist.at<int>(hh,0) ;
			maxp = hh;	//this is the peak of the hue in the foreground
		}
	
	}
	cerr << "maxHue: " << maxp << endl;
	frame_copy.copyTo(hand, foreground);
	cvtColor(hand, hand, CV_BGR2GRAY);




	calcHist(&channel[0], 1, 0, 255-d3, bghist, 1, &histsize, &histRange, true, false);
	//calcHist(&bgchannel[0],1,0,cv::Mat(),bghist,1,&histsize, &histRange, true, false);
	//skip hue = 0 (background?)
	long int maxbg = 0;
	int maxpbg = -1;
	for (int hh = 1; hh < hist.total(); hh++){
		cerr << bghist.at<float>(hh, 0) << " ";
		if (bghist.at<int>(hh, 0) > max){
			maxbg = bghist.at<int>(hh, 0);
			maxpbg = hh;	//this is the peak of the hue in the background
		}

	}



	cv::Mat hueMask = cv::Mat::zeros(y.size(), CV_8UC1);
	if (maxp > 0){
		//split the hue range in two, checking for wrapping at hues > 255
		vector<vector<int>> ranges;
		int left_limit = maxp - 2*(abs(maxpbg-maxp))/3;
		int right_limit = maxp + 2*(abs(maxpbg - maxp)) / 3;
		if (left_limit < 0){
			ranges.push_back({ 0, maxp });
			ranges.push_back({ 256 + left_limit, 255 });
		}
		else ranges.push_back({ left_limit, maxp });
		if (right_limit > 255){
			ranges.push_back({ maxp, 255 });
			ranges.push_back({ 0, right_limit-256 });
		}
		else ranges.push_back({ maxp, right_limit });
		
		
	
		vector<vector<int>>::iterator it;
		for (it = ranges.begin(); it != ranges.end(); ++it){
			cv::Mat tmp_mask;
			inRange(channel[0], it->at(0), it->at(1), tmp_mask);
			hueMask += tmp_mask;
		}

	}

	//medianBlur(hueMask, hueMask, 11);
	morphologyEx(hueMask, hueMask, CV_MOP_OPEN, diamond2);
	morphologyEx(hueMask, hueMask, CV_MOP_CLOSE, diamond3);
	imshow("hand", hueMask);
	//imshow("Hand", hand);
	//int c = waitKey(10); //menu input
	//if (char(c) == 'w'){
	//	cerr << "Dumping HUE to file." << endl;
	//	cv::FileStorage file("myhue.xml", cv::FileStorage::WRITE);
	//	file << "hue" << channel[0];
	//}
	return hueMask;
}


void BackgroundGenerator::computeBackgroundHue(int numFrames, FrameGrabber* grabber){
	findHomographyUsingCorners(grabber, 10);
	//cerr << "Computing hue" << endl;
	vector<cv::Mat> stack;  //stack of frames used to generate the background
	cv::Mat frame;
	for (int fcnt=0; fcnt < numFrames; fcnt++){
		grabber->getCurrentFrame().copyTo(frame);
		//cvtColor(frame,frame,CV_BGR2GRAY);
		warpPerspective(frame,frame,H_,cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
		//stack.push_back(frame);
		cv::Mat tmp;
		cv::Mat rect_hsv;
		cvtColor(frame,rect_hsv,CV_BGR2HSV);
		cv::Mat rchannel[3];
		split(rect_hsv, rchannel);
		//rchannel[0].convertTo(tmp,CV_32FC1,1.);
		imshow("BHUE", rchannel[0]);
		stack.push_back(rchannel[0]);
	}
	grabber->getCurrentFrame().copyTo(frame);
	warpPerspective(frame,backgroundImage_,H_,cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
	//imshow("BG", backgroundImage_);
	Hue_= getMean(stack, Huestd_);
	cerr << "Computing hue" << endl;
	imshow("MeanHue", Hue_);
	imshow("HueStd", Huestd_);
}



void BackgroundGenerator::computeBackgroundUsingCorners(int numFrames, FrameGrabber* grabber){
	findHomographyUsingCorners(grabber, numFrames);
	cv::Mat frame;
	grabber->getCurrentFrame().copyTo(frame);
	warpPerspective(frame,backgroundImage_,H_,cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify

}

//void BackgroundGenerator::computeBackground(int numFrames, FrameGrabber* grabber){
//	vector<aruco::Marker> Markers; //vector of detected markers
//	cv::Mat uframe; //undistorted frame
//	cv::Mat frame;
//	markerCenter_.clear();
//	//cout << "Creating background model and rectification matrix... ";
//	//waitKey(3000); //wait 3 seconds (is it necessary?)
//	try{
//		int fcnt = 0;
//		int skippedFrames = 0;
//		vector< vector <Point2f> > centers; 
//		for (int i=0;i<4;i++)
//			centers.push_back(vector<Point2f>());
//
//		cv::Mat avg;	
//		while (fcnt < numFrames && skippedFrames < 2*numFrames){
//
//			grabber->getCurrentFrame().copyTo(frame);
//
//			undistort(frame,uframe,detector_.camParams.CameraMatrix,detector_.camParams.Distorsion);
//			uframe.copyTo(frame);
//
//			//imshow("Calibration", frame);
//			Markers.clear();
//			Markers = detector_.detect(frame, false);
//			if (Markers.size() == 4){ //are we seeing all the markers?
//				fcnt++;
//				skippedFrames = 0;
//				//		cerr << "::" << fcnt << endl;
//				for (int m=0; m<4; m++){
//					centers[Markers[m].id].push_back(Markers[m].getCenter());
//
//				}
//			}
//			else{
//				//		cerr << endl << "!! Discarding frame !!" << endl;
//				skippedFrames++;
//			}
//		}
//
//		if (skippedFrames < 2*numFrames){
//			//		vector<Point2f> markerCenter;
//			for (int c=0;c<4;c++){
//				float meanx = 0;
//				float meany = 0;
//				for (int p=0; p<numFrames;p++){
//					meanx += centers[c][p].x;
//					meany += centers[c][p].y;
//				}
//				markerCenter_.push_back(Point2f(meanx/numFrames,meany/numFrames));
//			}
//
//			H_ = findHomography(markerCenter_,rectifiedPoints_,0); //compute homography once
//
//			warpPerspective(frame,backgroundImage_,H_,cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
//		}
//
//		else throw -2;
//		//	grabber.stop(); //close the stream and stop the grabber thread 
//		//	tt.join(); // wait for the thread to rejoin 
//	}
//
//	catch(std::exception& e)
//	{
//		std::cerr << "Error: " << e.what() << "\n";
//		//return -1;
//	}
//
//	catch (int e){
//		if (e==-1)
//			std::cerr << "Error opening the video stream.";
//	}
//}