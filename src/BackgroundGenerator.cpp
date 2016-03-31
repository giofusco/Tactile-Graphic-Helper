#include "BackgroundGenerator.h"


BackgroundGenerator::~BackgroundGenerator(void)
{
}


cv::Mat BackgroundGenerator::getForegroundMask_SOM(cv::Mat frame){
	cv::Mat tmp, hsvFrame;
	warpPerspective(frame, backgroundImage_, H_, cv::Size(sheetWmt_*pixelXmeter_, sheetHmt_*pixelXmeter_)); //rectify
	frame.convertTo(tmp, CV_32FC3);
	tmp *= 1. / 255;
	cv::cvtColor(tmp, hsvFrame, CV_BGR2HSV);
	std::vector<cv::Mat> channels;
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

