// FingerTracking.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "../FrameGrabber/FrameGrabber.h"
#include "../MarkerDetectorAruco/MarkerDetectorAruco.h"
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\opencv.hpp>
#include "boost\program_options.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "boost\circular_buffer.hpp"

using namespace cv;

using namespace std;

namespace po = boost::program_options;


//namespace po = boost::program_options;

void computeHomographyFromObservations(FrameGrabber& grabber, int numFrames, MarkerDetectorAruco& det, Mat& H,vector<Point2f>& markerCenter, vector<Point2f> rectifiedPoints) throw(int) {

	try{

		vector<aruco::Marker> Markers; //vector of detected markers
		Mat uframe; //undistorted frame
		Mat frame;
		markerCenter.clear();
		cout << "Creating background model and rectification matrix... ";
		//waitKey(3000); //wait 3 seconds (is it necessary?)

		int fcnt = 0;
		int skippedFrames = 0;
		vector< vector <Point2f> > centers; 
		for (int i=0;i<4;i++)
			centers.push_back(vector<Point2f>());
		while (fcnt < numFrames && skippedFrames < 2*numFrames){
			grabber.getCurrentFrame().copyTo(frame);
			undistort(frame,uframe,det.camParams.CameraMatrix,det.camParams.Distorsion);
			uframe.copyTo(frame);
		

			//imshow("Calibration", frame);
			Markers.clear();
			Markers = det.detect(frame, true);
			if (Markers.size() == 4){ //are we seeing all the markers?
				fcnt++;
				skippedFrames = 0;
				cerr << "::" << fcnt << endl;
				for (int m=0; m<4; m++)
					centers[Markers[m].id].push_back(Markers[m].getCenter());
			}
			else{
				cerr << endl << "!! Discarding frame !!" << endl;
				skippedFrames++;
			}
		}

		if (skippedFrames < 2*numFrames){
			//		vector<Point2f> markerCenter;
			for (int c=0;c<4;c++){
				float meanx = 0;
				float meany = 0;
				for (int p=0; p<numFrames;p++){
					meanx += centers[c][p].x;
					meany += centers[c][p].y;
				}
				markerCenter.push_back(Point2f(meanx/numFrames,meany/numFrames));
			}

			H = findHomography(markerCenter,rectifiedPoints,0); //compute homography once
		}
		else throw -2;
		//	grabber.stop(); //close the stream and stop the grabber thread 
		//	tt.join(); // wait for the thread to rejoin 
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

void computeHomographyFromObservationsSingleMarker(FrameGrabber& grabber, int numFrames, MarkerDetectorAruco& det, Mat& H, float w , float markerSize) throw(int) {

	try{

	

		vector<aruco::Marker> Markers; //vector of detected markers
		Mat uframe; //undistorted frame
		Mat frame;
	
		cout << "Creating background model and rectification matrix... " << endl;
		//waitKey(3000); //wait 3 seconds (is it necessary?)

		int fcnt = 0;
		int skippedFrames = 0;
		vector<Point2f> centers; 
		
	
			grabber.getCurrentFrame().copyTo(frame);
			undistort(frame,uframe,det.camParams.CameraMatrix,det.camParams.Distorsion);
			uframe.copyTo(frame);
		

			//imshow("Calibration", frame);
			Markers.clear();
			Markers = det.detect(frame, true);
			cerr << "0: " << Markers[0].id << endl;
			cerr << "1: " << Markers[1].id<< endl;
			cerr << "2: " << Markers[2].id<< endl;
			cerr << "3: " << Markers[3].id<< endl;
			if (Markers.size() > 0 ){ //are we seeing all the markers?
				fcnt++;
				skippedFrames = 0;
				//cerr << "::" << fcnt << endl;
			//	for (int m=0; m<4; m++)
				//	centers.push_back(Markers[0][m]);
				for (int m=0; m<4; m++)
					centers.push_back(Markers[3][m]);
					//centers[Markers[m].id].push_back(Markers[m].getCenter());
			}
			else{
				cerr << endl << "!! Discarding frame !!" << endl;
				skippedFrames++;
			}
		
			vector<Point2f> rectifiedPoints;
			//rectifiedPoints.push_back(Point2f(0,0));
			//rectifiedPoints.push_back(Point2f(markerSize,0));
			//rectifiedPoints.push_back(Point2f(markerSize,markerSize));
			//rectifiedPoints.push_back(Point2f(0,markerSize));

			rectifiedPoints.push_back(Point2f(0,markerSize+w));
			rectifiedPoints.push_back(Point2f(markerSize,markerSize+w));
			rectifiedPoints.push_back(Point2f(markerSize,2*markerSize+w));
			rectifiedPoints.push_back(Point2f(0,w+2*markerSize));

			H = findHomography(centers,rectifiedPoints,0); //compute homography once

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

int main_(int argc, char* argv[])
{
	try{

		float scale = 2500;
		// this should come from the outside
		float sheetWmt = 0.185;
		float sheetHmt = .25;
		vector<Point2f> rectifiedPoints;
		rectifiedPoints.push_back(Point2f(0,0));
		rectifiedPoints.push_back(Point2f(0,sheetHmt*scale));
		rectifiedPoints.push_back(Point2f(sheetWmt*scale,sheetHmt*scale));
		rectifiedPoints.push_back(Point2f(sheetWmt*scale,0));


		//************************************************************* OPTIONS PARSING
		// Declare the supported options.
		po::options_description desc("Allowed options");
		desc.add_options()
			("help", "produce help message")
			("cameraIP", po::value<string>(), "set camera ip address (1 to use usb webcam, 0 for integrated camera). Default value is 1.")
			//("template", po::value<string>(), "specify the appliance template file to use. This parameter is mandatory.")
			("calibration", po::value<string>(), "specify the camera calibration file.This parameter is mandatory.")
			("markersize", po::value<float>(), "specify the marker size in meters.This parameter is mandatory.")
			;

		po::variables_map vm;
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm); 

		if (vm.count("help") || vm.size() == 0) {
			cout << desc << "\n";
			return 1;
		}	

		string cameraURL = "1";
		if (vm.count("cameraIP")) {
			cameraURL = vm["cameraIP"].as<string>();
			cout << "### Camera URL set to: " << cameraURL << endl;
		}

		string calibrationFilename;
		if (vm.count("calibration")) {
			calibrationFilename = vm["calibration"].as<string>();
			cout << "### Loaded calibration: " << calibrationFilename << endl;
		}
		else {
			cerr << "!!! Camera calibration file is necessary !!!" << endl;
			cout << desc << "\n";
			return 1;
		}


		float markerSizeMeters;
		if (vm.count("markersize")) {
			markerSizeMeters = vm["markersize"].as<float>();
			cout << "### Marker size set to: " << markerSizeMeters << endl;
		}
		else {
			cerr << "!!! Marker size is necessary !!!" << endl;
			cout << desc << "\n";
			return 1;
		}


		//************************************************************* END OPTIONS PARSING

		//string calibrationFilename = "C:/Users/Giovanni/Documents\Visual Studio 2012/Projects/CoughlanLab/x64/Release/camera_params_logitech.yml";
		//float markerSizeMeters = .026;
		MarkerDetectorAruco det(calibrationFilename,  markerSizeMeters); //initialize the marker detector

		vector<int> IDs;
		for (int id=0;id<4;id++)
			IDs.push_back(id);



		Mat frame; //input frame
		Mat rectifiedFrame, out, BG;
		rectifiedFrame = Mat::zeros(Size(sheetWmt*scale, sheetHmt*scale),CV_8UC3);
		BG = Mat::zeros(Size(sheetWmt*scale, sheetHmt*scale),CV_8UC3);

		vector<aruco::Marker> Markers; //vector of detected markers

		bool maskInit = false;
		Mat uframe;
		Mat D;

		FrameGrabber grabber(cameraURL); //initialize the frame grabber
		thread tt = grabber.run(); //start the grabber

		Mat H;
		int numFrames = 10;
		vector<Point2f> markerCenter;



		while(1){
			grabber.getCurrentFrame().copyTo(frame);
			
			undistort(frame,uframe,det.camParams.CameraMatrix,det.camParams.Distorsion);
			uframe.copyTo(frame);
			imshow("Frame", frame);

			int c = waitKey( 10 );
		
			if( c== 32){
				cout << "Recomputing homography" << endl;
				computeHomographyFromObservations(grabber, numFrames, det, H, markerCenter, rectifiedPoints);
				warpPerspective(frame,rectifiedFrame,H,Size(sheetWmt*scale, sheetHmt*scale)); //rectify
				rectifiedFrame.copyTo(BG);
				break;
			}
			
		}

		//prepare edge map of background
	
		int grab = 0;
		
		boost::circular_buffer<Point2f> trace1(10);
		det.markerSize = 0.017;
		Point3f midPoint;
		vector<Point3f> midPt;
		vector<Point2f> midPt_prj;
		midPoint = Point3f(0,0,0);
			midPoint.x = 0.012;
			midPoint.y = -0.005;
		while (1){
			
			midPt.clear();
			//midPoint.x = 0.01;
			//midPoint.y = 0;
			//midPoint.z = 0.05;
			
			grabber.getCurrentFrame().copyTo(frame);
			undistort(frame,uframe,det.camParams.CameraMatrix,det.camParams.Distorsion);
			uframe.copyTo(frame);

			Markers.clear();
			
			Markers = det.detect(frame, true);
		
		
			//find marker with ID = 100 (let's call it cursor)                                                                                                 
			int cursor_idx = -1;
			for (int m=0;m<Markers.size();m++){
				if (Markers[m].id == 100){
		//			cerr << "**** T: " << Markers[m].Tvec << endl;
					/*midPoint.y = Markers[m].Tvec.at<float>(0,0);
					midPoint.x = Markers[m].Tvec.at<float>(0,1);
					midPoint.z = Markers[m].Tvec.at<float>(0,2);*/
					midPt.push_back(midPoint);
					cursor_idx = m;
					break;
				}
			}

			vector<Point2f> mPoints_rect, tmp_p;
			
			if (cursor_idx != -1){
				Point2f cursorCenter = Markers[cursor_idx].getCenter();
				Point2f p1, p2;
				//p1 = Markers[cursor_idx][2] - cursorCenter;
				//p2 = Markers[cursor_idx][3] - cursorCenter;
				projectPoints(midPt,Markers[cursor_idx].Rvec,Markers[cursor_idx].Tvec,det.camParams.CameraMatrix,det.camParams.Distorsion,midPt_prj);
				Markers[cursor_idx].push_back(midPt_prj[0]);
				perspectiveTransform( Markers[cursor_idx],mPoints_rect,H);
			}
			// find the orientation and 

			warpPerspective(frame,rectifiedFrame,H,Size(sheetWmt*scale, sheetHmt*scale)); //rectify
			
			
		
			if (mPoints_rect.size() > 0){
				
				trace1.push_back(mPoints_rect[4]);
			}
			else if (trace1.size()>0){ 
				trace1.pop_front();
				
			}
			

	//		cerr << "Trace : " << trace1.size() << endl;


			Mat ff;
			BG.copyTo(ff);
			
			vector<vector<Point>> pts;
			for (int t=0;t<trace1.size();t++){
			//	circle(BG,trace1[t],3,Scalar(0,0,255),2);
			//	circle(BG,trace2[t],3,Scalar(0,0,255),2);
				circle(ff,trace1[t],10,Scalar(0,255,0),2);
			}
			
			cv::imshow("Trace", ff);
		//	imshow("TouchPoint", ff);
			int c = waitKey(1);
			if(  c == 27 ){
				break;
			}
			else if( c== 32){
				cout << "Save Frame";
				string filename = "rect_" + to_string(grab) + ".png";
				imwrite(filename, rectifiedFrame);
				grab++;
				//computeHomographyFromObservations(grabber, numFrames, det, H, markerCenter, rectifiedPoints);
			}
			else if (c == 2490368) midPoint.x += 0.001; // up
			else if (c == 2621440) midPoint.x -= 0.001;   // down
			else if (c == 2424832) midPoint.y += 0.001;   //  left
			else if (c == 2555904) midPoint.y -= 0.001;  // right
			//else if (c!= -1) 
			//	cerr << "Keypressed: " << c << endl;
			//else if (c == 32 && vm.count("HRect")){
			//else 
		}


		grabber.stop(); //close the stream and stop the grabber thread 
		tt.join(); // wait for the thread to rejoin 
		return 0;
	}

	catch(std::exception& e)
	{
		std::cerr << "Error: " << e.what() << "\n";
		return -1;
	}

	catch (int e){
		if (e==-1)
			std::cerr << "Error opening the video stream.";
		else if (e == -10)
			std::cerr << "Error opening the appliance template.";
		else if (e==-20)
			std::cerr << "Error creating output folder.";
		else if (e == -2)
			std::cerr << "Markers are not visible enough.";
		return e;
	}
}

