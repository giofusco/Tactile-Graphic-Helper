#pragma once
#include "FrameGrabber.h"
#include "BackgroundGenerator.h"
#include "FingerTracker.h"
#include "TGModel.h"
#include <iostream>
#include <fstream>
#include <sapi.h>
#include <windows.h>

using namespace std;

class TGH
{

	 
public:
	TGH(void) {};
	TGH(FrameGrabber* grabber, string cameraURL, string calibrationFileName, string modeldir, string modelfile, float scale,int traceLength);
	void run(bool verbose);
	void loadTG(string tgdir, string tgfile);
	string processQuery(string query, Fingertip tip);
	bool detectQueryGesture(Fingertip &tip, float threshold_secs);
	inline void initializeUsing_SOM(int numframes) { backGen_.computeBackground_SOM(numframes, grabber_); }
	inline cv::Mat getBackgroundImage() { return backGen_.getBackgroundImage(); }
	inline void mute(bool status) { mute_ = status; }
	inline void logEvents(bool log) { logEvents_ = log; }
	inline void logEvents(bool log, std::string eventsFilename) { eventsFilename_ = eventsFilename; 
																  logEvents(log);
																  initEventLogging();  }
	~TGH(void);
	int fingerTipOffset;
private:

	void initEventLogging();
	void logQueryEvent(Fingertip tip, std::string ans);
	FrameGrabber* grabber_;
	BackgroundGenerator backGen_;
	FingerTracker tracker_;
	Fingertip qtip_;//fingertip used in the last query

	//used to enable second level feedback
	cv::Point lastQueryLocation_;
	time_t lastQueryTime_;

	TGModel querySys_;
	ISpVoice * pVoice_;
	std::thread grabber_tt_; 
	cv::Mat H_;
	std::string eventsFilename_;
	std::ofstream eventsLogFile_;
	float scale_;
	float sheetW_, sheetH_; //size of the sheet in meters
	long int frameno_;
	bool mute_;
	bool logEvents_;
	bool landscape_;
};

