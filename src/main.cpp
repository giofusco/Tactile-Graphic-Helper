#include "FrameGrabber.h"
#include "BackgroundGenerator.h"
#include "TGH.h"
#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <voce.h>

using namespace cv;
using namespace std;


namespace {
	/// Parameters and command line arguments
	struct Options
	{
		std::string input;              // input file stream to process
		std::string tgdir;
		std::string tgfilename;
		std::string videofilename;
		bool saveVideo;
		int k;
		float tgH;
		float tgW;
	};


	/// Parses command line options
	/// @param[in] argc number of command line arguments (including program name)
	/// @param[in] argv list of arguments
	/// @return an options structure populated by parsing the command line
	/// @throw runtime_error if there was a problem parsing the command line arguments,=
	Options parseOptions(int argc, char* argv[]) throw(std::runtime_error)
	{
		const char* keys =
		{
			"{ h       | help      | false       | help}"
			"{ tgdir   |           |             | directory containing the model}"
			"{ tgfilename |        |             | model filename }"
			"{ i       |		   |  1           | input. Either a file name, or a digit indicating webcam id }"
			"{ k |	| 100          | frames for background               }"
			"{ v |  |              | save video to the specified filename}"
		};
		cv::CommandLineParser parser(argc, argv, keys);
		if ((1 == argc) || (parser.get<bool>("h")))
		{
			parser.printParams();
			exit(EXIT_SUCCESS);
		}
		
		Options opts;

		opts.input = parser.get<std::string>("i");
		std::cerr << opts.input << std::endl;
		if (opts.input.empty())
		{
			throw std::runtime_error("Parser Error :: No input source specified");
		}

		opts.tgdir = parser.get<std::string>("tgdir");
		opts.tgfilename = parser.get<std::string>("tgfilename");
		opts.k = parser.get<int>("k");
		opts.videofilename = parser.get<std::string>("v");
		if (!opts.videofilename.empty())
			opts.saveVideo = true;
		else opts.saveVideo = false;

		return opts;
	}
}


int main(int argc, char* argv[])
{
	try{

		//************************************************************* OPTIONS PARSING
		// Declare the supported options.
		auto options = parseOptions(argc, argv);

		string modeldir = "";
		if (!options.tgdir.empty()){
			modeldir = options.tgdir;
			cout << "### TG dir set to: " << modeldir << endl;
		} else throw(-1);


		string modelfile = "";
		if (!options.tgfilename.empty()) {
			modelfile = options.tgfilename;
			cout << "### TG annotation file set to: " << modelfile << endl;
		}
		else throw(-1);


		bool isLandscape = 0;
	
		string calibrationFilename;
		
		int numFramesBackground = options.k;
		cout << "### numFramesBackground set to: " << numFramesBackground << endl;
		

		//float sheetWmt = 0.185;
		//float sheetHmt = .25;
		float sheetWmt = 1.;
		float sheetHmt = 1.;

		/*if (vm.count("tgH")) {
			sheetHmt = vm["tgH"].as<float>();
			cout << "### sheetHmt set to: " << sheetHmt << endl;
		}

		if (vm.count("tgW")) {
			sheetWmt = vm["tgW"].as<float>();
			cout << "### sheetWmt set to: " << sheetWmt << endl;
		}*/
		




//		voce::init("C:/dev/voce-0.9.1/lib", true, true, "file://localhost/C:/dev/workspace/SKERI/x64/Release/TGH/grammar", "query_short");


		//************************************************************* END OPTIONS PARSING

		float scale = 1500;
		 
		int c = -1;
		//FrameGrabber* grabber = new FrameGrabber(cameraURL,1024,768); //initialize the frame grabber
		FrameGrabber* grabber = new FrameGrabber(options.input); //initialize the frame grabber
		if (isLandscape){
			float tmp = sheetHmt;
			sheetHmt = sheetWmt;
			sheetWmt = tmp;
		}
		

		int ex = static_cast<int>(grabber->get(CV_CAP_PROP_FOURCC));
		char EXT[] = { ex & 0XFF, (ex & 0XFF00) >> 8, (ex & 0XFF0000) >> 16, (ex & 0XFF000000) >> 24, 0 };
		//grabber->set(CV_CAP_PROP_FOURCC, CV_FOURCC('H', '2', '6', '4'));
		grabber->set(CV_CAP_PROP_FRAME_WIDTH, 320);//2304);//1829//1200//800
		grabber->set(CV_CAP_PROP_FRAME_HEIGHT, 240);//1536); //1080//800//600   
		grabber->set(CV_CAP_PROP_FPS, 30);
		
		

		  
		TGH tgh(grabber,options.input,calibrationFilename,modeldir, modelfile, scale,sheetWmt, sheetHmt, 10);
		//TGH tgh(grabber, cameraURL, calibrationFilename);
		//tgh.loadTG(modeldir, modelfile);
		Mat frame; //current frame


		c = -1;

		cout << "TGH v.0.2" << endl;
		cout << "Menu:" <<endl;
		cout << "\t . [b] to (re)generate background" <<endl;
		cout << "\t . [s] to show/hide background image" << endl;
		cout << "\t . [t] to start/stop TGH" << endl;
		cout << "\t . [w] save TG to file for annotation" << endl;
		cout << "\t . [Esc] to quit" << endl;


		bool bkg_showing = false;
		bool bkg_computed = false;
		bool track_finger = false;
		bool show_trace = false;

		while (c != 27){
			grabber->getCurrentFrame().copyTo(frame);
			cv::imshow("Input", frame);

			if (track_finger){ //track finger
				if (!bkg_computed){		// make sure we have computed the Background
					if (options.saveVideo)	//start saving the video when starting to create the background model.
						grabber->saveToVideo(options.videofilename);
					tgh.initializeUsing_SOM(numFramesBackground);
					bkg_computed = true;
				}
				tgh.run(false); 
			}

			c = waitKey(1); //menu input

			switch(char(c)){
			case 'b':
				tgh.initializeUsing_SOM(numFramesBackground);
				bkg_computed = true;
				break;
			case 's':	// show/hide background image (check if it's available too)
				if (bkg_showing){
					destroyWindow("Background");
					bkg_showing = false;
				}
				else if(bkg_computed){
					cv::imshow("Background", tgh.getBackgroundImage());
					bkg_showing = true;
				}
				break;
			case 't':

				track_finger = !track_finger;

				break;
			case 'v':
				show_trace = !show_trace;
				break;
			case 'w':
				if (bkg_computed)
					imwrite("TG_export.png",tgh.getBackgroundImage());
				break;
			default:
				break;
			}
		} 
		return 0;
	}


	catch (int e){
		if (e==-1)
			std::cerr << "Error opening the video stream.";
		else if (e == -10)
			std::cerr << "Error opening the appliance template.";
		else if (e==-20)
			std::cerr << "Error creating output folder.";
		else if (e == -2)
			std::cerr << "Markers are not visible.";
		else if (e==-100)
			cerr << "Error initializing the TTS system";
		return e;
	}
}