#include "FeatureInfo.h"


FeatureInfo::FeatureInfo(void)
{
}

FeatureInfo::FeatureInfo(cv::FileNode fn){
	cv::FileNodeIterator fit;

	for (fit = fn.begin();fit!=fn.end();++fit){
		cv::FileNode ftmp = *fit;
		if (ftmp.name() == "Number")
			id = int(ftmp);
		else if (ftmp.name() == "Title")
			title = string(ftmp);
		else if (ftmp.name() == "Type")
			type = string(ftmp);
		else if (ftmp.name() == "Style")
			style = string(ftmp);
		else if (ftmp.name() == "Thickness")
			thickness = string(ftmp);
		else if (ftmp.name() == "Purpose")
			purpose = string(ftmp);
		else if (ftmp.name() == "Relates_Label_From")
			from = string(ftmp);
		else if (ftmp.name() == "Relates_Label_To")
			to = string(ftmp);
		else if (ftmp.name() == "Quantity")
			qty = int(ftmp);
	}
}

FeatureInfo::~FeatureInfo(void)
{
}
