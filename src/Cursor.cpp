#include "Cursor.h"


Point2f Cursor::detectCursor(Mat frame, bool verbose){
	vector<aruco::Marker> Markers; //vector of detected markers
	Markers = detector_.detect(frame, verbose);
	int cursor_idx = -1;
	vector<Point3f> tipPointVect; //it's a vector because OpenCV is dumb!
	vector<Point2f> projTipPointVect;

	for (int m=0;m<Markers.size();m++){
		if (Markers[m].id == markerID_){
			tipPointVect.push_back(tipPoint_);
			cursor_idx = m;
			break;
		}
	}
	vector<Point2f> mPoints_rect, tmp_p;
	//cerr << "CURSOR IDX " << cursor_idx << endl;
	if (cursor_idx != -1){
		Point2f cursorCenter = Markers[cursor_idx].getCenter();
		Point2f p1, p2;

		projectPoints(tipPointVect,Markers[cursor_idx].Rvec,Markers[cursor_idx].Tvec,detector_.camParams.CameraMatrix,detector_.camParams.Distorsion,projTipPointVect); //project 3D point to 2D
		//Markers[cursor_idx].push_back(midPt_prj[0]);
		perspectiveTransform(projTipPointVect,projTipPointVect,H_);
	}
	else 
		projTipPointVect.push_back(Point2f(0,0));
	return projTipPointVect[0];
}



Cursor::~Cursor(void)
{
}
