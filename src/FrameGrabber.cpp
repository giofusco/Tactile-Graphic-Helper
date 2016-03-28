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
	if (saveToVideo_){
		writer_.open(filename,CV_FOURCC('M','J','P','G'),30, cv::Size(480,640));
	}
}

void FrameGrabber::stop()
{
	if (isOpened())
	{
		frame_mutex_.lock();
		release();
		frame_mutex_.unlock();
	}
}

