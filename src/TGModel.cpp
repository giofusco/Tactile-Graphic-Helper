#include "stdafx.h"
#include "TGModel.h"
#include "opencv2\imgproc\imgproc.hpp"
#include <opencv2\opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>

TGModel::TGModel(void)
{
}

TGModel::TGModel(string tgdir, string tgfilename)
{
	cv::FileNode n;
	cv::FileNode fn; //features node
	string filename = tgdir + '/' + tgfilename;
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	if (fs.isOpened()){
		cv::FileNodeIterator it, fit;
		n = fs["Info"]; 
		string annotationfile;
		int rows,cols;
		for (it = n.begin();it!=n.end();++it){
			cv::FileNode tmp = *it;
			if (tmp.name() == "Title")
				tgTitle_ = string(tmp);
			else if (tmp.name() == "Type")
				tgType_ = string(tmp);
			else if (tmp.name() == "Buffer_Image_Filename")
				annotationfile = tgdir + '/' + string(tmp);
			else if (tmp.name() == "Buffer_Rows") {
				rows = (int)tmp;
				imageSize_.height = rows;
			}
			else if (tmp.name() == "Buffer_Columns") {
				cols = (int)tmp;
				imageSize_.width = cols;
			}

		}
		//filename = "C:/Users/Giovanni/Documents/Visual Studio 2012/Projects/SKERI/x64/Debug/TG_export_matrix.bmp"; //TODO: remove this
		loadAnnotations(annotationfile, rows, cols);
		n = fs["Features"];
		for (it = n.begin();it!=n.end();++it){
			cv::FileNode tmp = *it;
			//	cerr << tmp.name() << endl;
			fn = n[tmp.name()];
			FeatureInfo info(fn);
			features.insert(make_pair(info.id,info));

		}
	}
}

TGModel::~TGModel(void)
{
}

void TGModel::loadAnnotations(string filename, int rows, int cols){

	/*std::ifstream in(filename);
	std::string contents((std::istreambuf_iterator<char>(in)), 
	std::istreambuf_iterator<char>()); */

	//annotations_ = cv::Mat( rows, cols, CV_8UC1, (char)contents.c_str());
	annotations_ = cv::imread(filename,-1);
	vector<cv::Vec4i> hierarchy;
	cv::Mat src_copy = annotations_.clone();
	cv::findContours( src_copy, annotContours_, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
	//drawContours(src_copy,contours,-1,Scalar(255,0,0),1);
	//imshow("CNT", src_copy);


}

void TGModel::resizeAnnotations(cv::Size sz){
	if (sz != annotations_.size()){
		cv::Size asz = annotations_.size();
		//learn the transformation from the input frame to the annotation image
		vector<cv::Point2f> pt_frame, pt_annot;
		pt_frame.push_back(cv::Point2f(0,0));
		pt_frame.push_back(cv::Point2f(0,sz.width));
		pt_frame.push_back(cv::Point2f(sz.height,sz.width));
		pt_frame.push_back(cv::Point2f(sz.height,0));
		pt_annot.push_back(cv::Point2f(0,0));
		pt_annot.push_back(cv::Point2f(0,asz.width));
		pt_annot.push_back(cv::Point2f(asz.height,asz.width));
		pt_annot.push_back(cv::Point2f(asz.height,0));
		T_ = cv::findHomography(pt_frame,pt_annot,0); //compute homography once

		//resize(annotations_, annotations_,sz, INTER_NEAREST);
	}
	//imshow("Annotations", annotations_);
}


string TGModel::isThereA(string query){
	map<int,FeatureInfo>::iterator it;
	string ans;
	string list = "";
	int isthere = 0;

	//cerr << "Query: " << endl;
	for (it=features.begin();it!=features.end(); ++it){
		string title = string(it->second.title);
		transform(title.begin(), title.end(), title.begin(), ::tolower);
		//cerr << "Title: " << it->second.title << endl;
		std::size_t found=query.find(title);
		std::size_t found2 =title.find(query);
		if (found!=std::string::npos || found2!=std::string::npos){
			isthere++;
			int q = it->second.qty;
			string title = it->second.title;
			list += to_string(q) + " ";
			(q>1)? list+= title+'s' : list+= title;
			list += " ";

		}
	}
	if (isthere > 1)
		ans = "There are " + list;
	else if (isthere == 1)
		ans = "There are " + list;
	else ans = "No.";
	return ans;
}

string TGModel::whatsAt(cv::Point pt){
	cv::Mat tmp;
	double min;
	double max;
	minMaxIdx(annotations_, &min, &max);
	convertScaleAbs(annotations_, tmp, 255 / max);
	//map point from current frame to annotation
	vector<cv::Point2f> apt, p;
	p.push_back(pt);
	perspectiveTransform(p,apt,T_);
	//cerr << "Query pt: " << p << " --> " << Point2i(apt[0]) << endl;
	circle(tmp,apt[0],2, cv::Scalar(2550,0,0));

	uchar idx = annotations_.at<uchar>(cv::Point2i(apt[0]));
	int id = int(idx);
	//cerr << "ID: " << id << endl;

	if (id == 0){ //no annotation found, try to find the closest label
		double minDist = 1e10;
		int minidx = -1;
		double d;
		for (int c=0;c<annotContours_.size();c++){
			d = fabs(pointPolygonTest(annotContours_[c],apt[0],true));	
			if (d<minDist){
				minDist = d;
				minidx = c;
			}
		}
		if (minidx >= 0){


			double minDistC= 1e10;
			int minidxC = -1;

			for(int c=0;c<annotContours_[minidx].size();c++){
				cv::Point2f ptCt = annotContours_[minidx][c];
				double d = norm(ptCt-apt[0]);
				if (d < minDistC){
					minDistC = d;
					minidxC = c;
				}

			}

			if (minDistC < 10.){
				circle(tmp,annotContours_[minidx][minidxC],3, cv::Scalar(2550,0,0),2);
				id = int(annotations_.at<uchar>(annotContours_[minidx][minidxC]));
			}
			//cerr << "Closest contour: " << minidx << ", dist: " << minDistC << " Label: " << int(annotations_.at<uchar>(annotContours_[minidx][minidxC])) << endl;
		}
	}
	//imshow("Query point", tmp);

	string ans;
	if (id > 0){
		map<int,FeatureInfo>::iterator it;
		it=features.find(id);
		if (it!=features.end())
			ans = "That is " + features[id].title;
		else
			ans = "I am not sure, try again";
	}
	else ans = "Nothing";

	return ans;
}


string TGModel::whatsThere(){
	string ans = "there are: ";
	map<int,FeatureInfo>::iterator it;
	for (it=features.begin();it!=features.end(); ++it){
		//find quantity first
		int q = it->second.qty;
		string title = it->second.title;
		if (q < 10)
			ans += num2string(q) + " ";
		else ans += to_string(q) + " ";
		(q>1)? ans+= title+'s' : ans+= title;
		ans+= ", ";
	}
	//cerr << ans << endl;
	return ans;

}

string TGModel::num2string(int n){
	if (n == 0) return "zero";
	else if (n==1) return "one";
	else if (n==2) return "two";
	else if (n==3) return "three";
	else if (n==4) return "four";
	else if (n==5) return "five";
	else if (n==6) return "six";
	else if (n==7) return "seven";
	else if (n==8) return "eight";
	else if (n==9) return "nine";
}