#include "TGH.h"
//#include "boost\circular_buffer.hpp"


TGH::TGH(FrameGrabber* grabber, string cameraURL, string calibrationFileName, string modeldir, string modelfile, float scale, float w, float h, int traceLength = 10){
	try{

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
		if (sheetW_ >= sheetH_){
			scale_ = sheetW_ / 320;
			sheetW_ = 320;
			sheetH_ /= scale_;
		}
		else {
			scale_ = sheetH_ / 320;
			sheetH_ = 320;
			sheetW_ /= scale_;
		}
		
		
		querySys_.resizeAnnotations(cv::Size(sheetW_,sheetH_));

		// this should come from the outside
		//sheetH_ = h;
		//sheetW_ = w;
		frameno_ = 0;
		
		fingerTipOffset = 0; //MODIFIED 3/29/16

		grabber_ = grabber; //initialize the frame grabber
		grabber_tt_ = grabber_->run(); //start the grabber
		//instantiate background generator
		backGen_ = BackgroundGenerator(sheetW_,sheetH_);
		tracker_ = FingerTracker();
		//instantiate Cursor
//		cursor_ = Cursor(0,0,calibrationFileName);
		//cursorTrace_ = boost::circular_buffer<cv::Point2f>(traceLength);
	}catch(int e){
		if (e==-1)
			std::cerr << "Error opening the video stream.";
	}
}


	


void TGH::adjustCursor(char dir, float delta){
	switch(dir){
	case 'u':
		fingerTipOffset--;
		break;
	case 'd':
		fingerTipOffset++;
		break;
	case 'l':
		querySys_.whatsThere();
		//cursor_.adjustCursor(0., delta, 0.);
		break;
	case 'r':
		;
//		cursor_.adjustCursor(0., -delta, 0.);
		break;
	default:
		;
	}

}

void TGH::run(bool verbose){
	int c = -1;
	cv::Mat frame;
	cv::Point2f cursorCoord;
	static int numFrames = 0;

	bool show_trace = false;

	cv::Mat hand;
	grabber_->getCurrentFrame().copyTo(frame);
	//cv::medianBlur(frame, frame, 3);
	hand = backGen_.getForegroundMask_SOM(frame);

	tracker_.track(hand,fingerTipOffset);
	tracker_.showTips(backGen_.getBackgroundImage().clone());
	numFrames++;

	vector<Fingertip> tips = tracker_.getLastSeenTip(3);
	if (tips.size() == 1){
		if (tips[0].getStationaryTime() >= 1.1){
			string ans = processQuery("okay what is this");
			std::wstring stemp = std::wstring(ans.begin(), ans.end());
			LPCWSTR sw = stemp.c_str();
			cerr << "TGH says: " << ans << endl;
			HRESULT hr = pVoice_->Speak(sw, 0, NULL);
			tips[0].resetStationaryTime(); //to avoid repeated feedback
		}
	}

}


void TGH::loadTG(string tgdir, string tgfile){

	//modelDir_ = tgdir;
	//string filename = tgdir + '/' + tgfile;

	querySys_ = TGModel(tgdir, tgfile);
	//make sure the rectified image and the annotation image dimensions match
	//Size s(floor(sheetW_*scale_), floor(sheetH_*scale_));
	//querySys_.resizeAnnotations(querySys_.getSize());
}

// extract the query from the string
string TGH::processQuery(string query){
	std::size_t found, found2;
	string ans;
	// branch #1 -- Either asking the list of items or what's in a point
	if (found = query.find("what is") != string::npos){
		if (found2 = query.find("there", found) != string::npos){
			//ask the list
			ans = querySys_.whatsThere();
		//	;
		}
		if (found2 = query.find("this", found) != string::npos){
			//ask what's in a location
			//need to know that there's only on tip visible
			vector<cv::Point> tips = tracker_.getLastSeen(3);
			if (tips.size() == 1){
				//cerr << "Query pt: " << tips[0] << endl;
				ans = querySys_.whatsAt(tips[0]);
			}
			else if (tips.size()>1) return "Use one finger, please!";
			else return "I can't see your finger!";
		}
	}
	/*else 
		if (((found = query.find("is there") != string::npos)  || (found = query.find("are there") != string::npos)) && (found = query.find("what") == string::npos)){
			ans = querySys_.isThereA(query);
		}*/

	return ans;
}


TGH::~TGH(void)
{
	grabber_->stop(); //close the stream and stop the grabber thread 
	grabber_tt_.join(); // wait for the thread to rejoin 
	delete grabber_;
}


