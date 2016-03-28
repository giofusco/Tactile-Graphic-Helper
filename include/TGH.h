#pragma once
//#include "boost\circular_buffer.hpp"
#include "FrameGrabber.h"
//#include "../MarkerDetectorAruco/MarkerDetectorAruco.h"
#include "BackgroundGenerator.h"
#include "FingerTracker.h"
#include "Cursor.h"
#include "TGModel.h"
#include <sapi.h>
#include <windows.h>

using namespace std;

class TGH
{

	 
public:

	TGH(void);
//	TGH(FrameGrabber* grabber, string cameraURL, string calibrationFileName, int traceLength = 10) : TGH(grabber, cameraURL, calibrationFileName, 1, 1, 1, traceLength);
	TGH(FrameGrabber* grabber, string cameraURL, string calibrationFileName, string modeldir, string modelfile, float scale, float w, float h,int traceLength);
	inline void initialize(int numframes){backGen_.computeBackground(numframes,grabber_);}
	inline void initializeUsingCorners(int numframes){backGen_.computeBackgroundUsingCorners(numframes,grabber_);}
	inline void initializeUsing_TM09(int numframes){backGen_.computeBackground_TM09(numframes, grabber_);}
	inline void initializeUsing_SOM(int numframes){ backGen_.computeBackground_SOM(numframes, grabber_); }
	inline void initializeUsingHue(int numframes){backGen_.computeBackgroundHue(numframes, grabber_);}
	inline cv::Mat getBackgroundImage(){return backGen_.getBackgroundImage();}
	void run(bool verbose);
//	boost::circular_buffer<Point2f> getCursorTrace(){return cursorTrace_;}
	~TGH(void);
	void adjustCursor(char dir, float delta);
	int fingerTipOffset;
	//void loadTG(string tgfile);
	void loadTG(string tgdir, string tgfile);
	string processQuery(string query);

private:
	FrameGrabber* grabber_;
	BackgroundGenerator backGen_;
	FingerTracker tracker_;
	//Cursor cursor_;
	TGModel querySys_;
	std::thread grabber_tt_; 
	cv::Mat H_;
//	boost::circular_buffer<cv::Point2f> cursorTrace_;
	float scale_;
	float sheetW_, sheetH_; //size of the sheet in meters
	bool landscape_;
	long int frameno_;
	ISpVoice * pVoice_;

};

