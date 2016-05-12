#include "TGH.h"


TGH::TGH(FrameGrabber* grabber, string cameraURL, string calibrationFileName, string modeldir, string modelfile, float scale, int traceLength = 10) {
	try {

		pVoice_ = NULL;

		if (FAILED(::CoInitialize(NULL)))
			throw(-100);

		HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice_);
		if (!SUCCEEDED(hr))
			throw(-100);

		loadTG(modeldir, modelfile);

		scale_ = 1.;
		sheetW_ = querySys_.getSize().width;
		sheetH_ = querySys_.getSize().height;

		//set the scale, fix largest dimension to 320
		if (sheetW_ >= sheetH_) {
			scale_ = sheetW_ / 320;
			sheetW_ = 320;
			sheetH_ /= scale_;
		}
		else {
			scale_ = sheetH_ / 320;
			sheetH_ = 320;
			sheetW_ /= scale_;
		}


		querySys_.resizeAnnotations(cv::Size(sheetW_, sheetH_));
		lastQueryLocation_ = cv::Point(-1, -1);
		lastQueryTime_ = 0;
		frameno_ = 0;

		fingerTipOffset = 0; //MODIFIED 3/29/16

		grabber_ = grabber; //initialize the frame grabber
		grabber_tt_ = grabber_->run(); //start the grabber

		//instantiate background generator
		backGen_ = BackgroundGenerator(sheetW_, sheetH_);
		tracker_ = FingerTracker();
	}
	catch (int e) {
		if (e == -1)
			std::cerr << "Error opening the video stream.";
	}
}



void TGH::run(bool verbose) {
	int c = -1;
	cv::Mat frame;
	cv::Point2f cursorCoord;
	static int numFrames = 0;

	bool show_trace = false;

	cv::Mat hand;
	grabber_->getCurrentFrame().copyTo(frame);
	//cv::medianBlur(frame, frame, 3);
	hand = backGen_.getForegroundMask_SOM(frame);


	string filenameMask = "mask_" + to_string(numFrames) + ".png";
	string filenameFrame = "frame_" + to_string(numFrames) + ".png";

//	cv::imwrite(filenameMask, hand);
//	cv::imwrite(filenameFrame, backGen_.getBackgroundImage());

	tracker_.track(hand, fingerTipOffset);
	tracker_.showTips(backGen_.getBackgroundImage().clone());



	numFrames++;
	Fingertip tip;
	//	vector<Fingertip> tips = tracker_.getLastSeenTip(3);
	//	if (tips.size() == 1) {
	//		if (tips[0].getStationaryTime() >= 1.1) {
	if (detectQueryGesture(tip, .5)) {
		if (!mute_) {
			float d = sqrt((tip.getPosition(3).x - lastQueryLocation_.x) * (tip.getPosition(3).x - lastQueryLocation_.x) +
				(tip.getPosition(3).y - lastQueryLocation_.y) * (tip.getPosition(3).y - lastQueryLocation_.y));
			cerr << "distance from last query point: " << d << endl;
			if (d > 10 || lastQueryLocation_.x < 0 || double(difftime(time(NULL), lastQueryTime_) >= 2.)){ //only give feedback if the fingertip has moved far enough, otherwise it should give more details about this location
				string ans = processQuery("okay what is this", tip);
				qtip_ = tip;
				lastQueryLocation_ = tip.getPosition(1);
				if (logEvents_)
					logQueryEvent(tip, ans);
				std::wstring stemp = std::wstring(ans.begin(), ans.end());
				LPCWSTR sw = stemp.c_str();
				clog << "TGH says: " << ans << endl;
				HRESULT hr = pVoice_->Speak(sw, 0, NULL);
				time(&lastQueryTime_);
			}
			else if (detectQueryGesture(tip, 0.5)){
				if (double(difftime(time(NULL), lastQueryTime_) < 2.)) {
					string ans = processQuery("okay more info", qtip_);
					std::wstring stemp = std::wstring(ans.begin(), ans.end());
					LPCWSTR sw = stemp.c_str();
					clog << "TGH says: " << ans << endl;
					HRESULT hr = pVoice_->Speak(sw, 0, NULL);
					//time(&lastQueryTime_);
				}
			}

		}
	}
	//check if the user is still holding the finger after a recent query
	//else if (detectQueryGesture(tip, 0.5) && lastQueryLocation_.x > 0) { //the check on the lastQueryLocation is to make sure we asked something already
	//	//if the tip is in a close location to the last query location
	//	if (sqrt((tip.getPosition(3).x - lastQueryLocation_.x) * (tip.getPosition(3).x - lastQueryLocation_.x) +
	//		(tip.getPosition(3).y - lastQueryLocation_.y) * (tip.getPosition(3).y - lastQueryLocation_.y)) < 5.) {
	//		if (double(difftime(time(NULL), lastQueryTime_) < 2.)) {
	//			string ans = processQuery("okay more info", tip);
	//		}

	//	}
	//}
}


void TGH::logQueryEvent(Fingertip tip, std::string ans) {
	time_t timer;
	time(&timer);
	eventsLogFile_ << timer << " " << tip.getPosition(3).x << " " << tip.getPosition(3).y <<  " " << ans << std::endl;
}


bool TGH::detectQueryGesture(Fingertip &tip, float threshold_secs) {
	vector<Fingertip> tips = tracker_.getLastSeenTip(3);
	if ((tips.size() == 1) && (tips[0].getStationaryTime() >= threshold_secs)) {
		tips[0].resetStationaryTime(); //to avoid repeated feedback
		tip = tips[0];
		return true;
	}
	else return false;
}

void TGH::loadTG(string tgdir, string tgfile) {

	querySys_ = TGModel(tgdir, tgfile);
}

// extract the query from the string
string TGH::processQuery(string query,Fingertip tip) {
	std::size_t found, found2;
	string ans;
	// branch #1 -- Either asking the list of items or what's in a point
	if (found = query.find("what is") != string::npos) {
		if (found2 = query.find("there", found) != string::npos) {
			//ask the list
			ans = querySys_.whatsThere();
		}
		if (found2 = query.find("this", found) != string::npos) {
			//ask what's in a location
			//need to know that there's only on tip visible
			//vector<cv::Point> tips = tracker_.getLastSeen(3);
			//if (tips.size() == 1) {
				//cerr << "Query pt: " << tips[0] << endl;
				ans = querySys_.whatsAt(tip.getPosition(1), false);
			//}
			//else if (tips.size() > 1) return "Use one finger, please!";
			//else return "I can't see your finger!";
		}
	}
	else if (found = query.find("more info") != string::npos) {
		ans = querySys_.whatsAt(tip.getPosition(1), true);
	}
	return ans;
}


TGH::~TGH(void)
{
	grabber_->stop(); //close the stream and stop the grabber thread 
	grabber_tt_.join(); // wait for the thread to rejoin  
	delete grabber_;
//	delete pVoice_;
	eventsLogFile_.close();
}


void TGH::initEventLogging() {
	if (!eventsFilename_.empty())
		eventsLogFile_.open(eventsFilename_, std::ios_base::out | std::ios_base::app);
	else{
		clog << "Events log filename not specified. Not logging.";
		logEvents_ = false;
	}
}