#pragma once
#include "FrameGrabber.h"
#include "BackgroundGenerator.h"
#include "FingerTracker.h"
#include "TGModel.h"
#include <sapi.h>
#include <windows.h>

using namespace std;

class TGH
{

	 
public:

	TGH(void) {};
	TGH(FrameGrabber* grabber, string cameraURL, string calibrationFileName, string modeldir, string modelfile, float scale,int traceLength);
	inline void initializeUsing_SOM(int numframes){ backGen_.computeBackground_SOM(numframes, grabber_); }
	inline cv::Mat getBackgroundImage(){return backGen_.getBackgroundImage();}
	void run(bool verbose);
	
	void loadTG(string tgdir, string tgfile);
	string processQuery(string query);
	inline void mute(bool status) { mute_ = status; }
	
	~TGH(void);
	
	int fingerTipOffset;
	

private:
	FrameGrabber* grabber_;
	BackgroundGenerator backGen_;
	FingerTracker tracker_;
	TGModel querySys_;
	std::thread grabber_tt_; 
	cv::Mat H_;
	float scale_;
	float sheetW_, sheetH_; //size of the sheet in meters
	bool landscape_;
	long int frameno_;
	ISpVoice * pVoice_;
	bool mute_;

};

