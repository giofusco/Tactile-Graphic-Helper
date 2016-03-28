#pragma once

#include "opencv2\opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <thread>
#include <mutex>
#include <string>

using namespace std;
//using namespace cv;



	class FrameGrabber :
		public cv::VideoCapture
	{
	public:
		FrameGrabber(void) { long int grabbedFrames_ = 0; };
		FrameGrabber(string s) throw(int) {
			open_(s);
		};

		FrameGrabber(int d) throw(int) : FrameGrabber() { open(d);if (!isOpened()) throw -1;};

		FrameGrabber(int d, int w, int h) throw(int) : FrameGrabber(){
			open(d); set(CV_CAP_PROP_FRAME_WIDTH,w); 
			set(CV_CAP_PROP_FRAME_HEIGHT,h); 
			if (!isOpened()) 
				throw -1;};
		
		FrameGrabber(string s, int w, int h) throw(int) : FrameGrabber(){
			open_(s);
			set(CV_CAP_PROP_FRAME_WIDTH,w); 
			set(CV_CAP_PROP_FRAME_HEIGHT,h); 
			if (!isOpened()) 
				throw -1;};

		void pause(){ frame_mutex_.lock();}
		void resume(){ frame_mutex_.unlock();}
		void saveToVideo(string filename);
		void stopVideoSaving(){saveToVideo_= false;}
		~FrameGrabber(void);

		//return the current frame
		cv::Mat getCurrentFrame(){
			this->frame_mutex_.lock(); 
			grabbedFrames++;
			cv::Mat frame = currentFrame_;
			this->frame_mutex_.unlock();
			return frame;
		}
		
		long int grabbedFrames;

		//start the grabber thread
		thread run(){
			return std::thread(&FrameGrabber::grabFrames, this);
		}
		///Releases the frame grabber. Call this instead of release() to ensure that the other thread is stopped appropriately.
		void stop();

	private:
		//FrameGrabber(const FrameGrabber&);
		void open_(string s) throw(int){
			if (s.size()<=2)
				open(stoi(s));
			else{
				string url = "http://" + s + ":8080/videofeed?dummy=param.mjpg";
				open(url); 
			}
			cv::waitKey(2000);
			if (!isOpened()) throw -1;
		}
		
		mutex frame_mutex_; //mutex on the frame 
		cv::Mat currentFrame_; //stores the latest frame grabbed (mutex)
		cv::VideoWriter writer_;
		bool saveToVideo_;
		void grabFrames(); //the grabber thread 
	};


