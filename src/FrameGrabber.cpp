#include "stdafx.h"
#include "FrameGrabber.h"



FrameGrabber::~FrameGrabber(void)
{
	if (isOpened())
		release();
}

void FrameGrabber::grabFrames() throw(int){

	if (this->isOpened()){
		while (isOpened()){ //check that the stream is still valid (is the only exit condition for the thread)
			frame_mutex_.lock();
			read(currentFrame_);
			if (saveToVideo_)
				writer_.write(currentFrame_);
			frame_mutex_.unlock();
		}
	}
	else
		throw(-1);

}

void FrameGrabber::saveToVideo(string filename){
	saveToVideo_ = true;
	closeVideo_ = true; //flag to remember to close the video when destroying the object, even if we pause the recording
//	if (saveToVideo_){
		cv::Size S = cv::Size((int)get(CV_CAP_PROP_FRAME_WIDTH),    //Acquire input size
			(int)get(CV_CAP_PROP_FRAME_HEIGHT));
		writer_.open(filename,CV_FOURCC('M','J','P','G'), get(CV_CAP_PROP_FPS), S,true);
//	}
}

void FrameGrabber::stop()
{
	if (isOpened())
	{
		frame_mutex_.lock();
		release();
		if (closeVideo_)
			writer_.release();
		frame_mutex_.unlock();
	}
}

