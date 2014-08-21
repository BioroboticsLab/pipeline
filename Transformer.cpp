/*
 * Transformer.cpp
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#include "Transformer.h"

namespace decoder {


/**************************************
 *
 * 			constructor
 *
 **************************************/
Transformer::Transformer() {
	// TODO Auto-generated constructor stub

}

Transformer::~Transformer() {
	// TODO Auto-generated destructor stub
}


/**************************************
 *
 * 			stuff
 *
 **************************************/

void Transformer::process(TagList &taglist){

	for (int i = 0; i < taglist.size(); i ++){
		Tag tag = taglist.getTag(i);
		if(tag.isValid()){
			this->_transformImages(tag);
		}
	}
}


void Transformer::_transformImages(Tag &tag){

	Mat originalImage = tag.getOrigSubImage();
	vector <TagCandidate> candidates = tag.getCandidates();

	for(int i = 0; i < candidates.size(); i++){
		TagCandidate candidate = candidates[i];
		Mat transformedImage = this->_ellipseTransform(candidate.getEllipse(), originalImage);
		candidate.setTransformedImage(transformedImage);
	}
}


Mat Transformer::_ellipseTransform( Ellipse ell, Mat originalImage) {

	Mat rot = Mat(2, 3, CV_64F);

	// rotation angle is the ellipse' orientation
	double a = (ell.angle*CV_PI)/180.0;

	// scale factor in y-direction
	double s = ((double)ell.axis.width)/ ((double) ell.axis.height);

	//center of the transformation
	float x0 = ell.cen.x;
	float y0 = ell.cen.y;

	// create the following rotation matrix: http://goo.gl/H3kZDj
	rot.at<double>(0,0) = cos(a)*cos(a) + s*sin(a)*sin(a);
	rot.at<double>(0,1) = cos(a)*sin(a) - s*cos(a)*sin(a);
	rot.at<double>(0,2) = -(cos(a)*cos(a) + s*sin(a)*sin(a))*x0 + x0 - y0*(cos(a)*sin(a) - s*cos(a)*sin(a));
	rot.at<double>(1,0) = cos(a)*sin(a) - s*cos(a)*sin(a);
	rot.at<double>(1,1) = s*cos(a)*cos(a) + sin(a)*sin(a);
	rot.at<double>(1,2) = -(s*cos(a)*cos(a) + sin(a)*sin(a))*y0 + y0 - x0*(cos(a)*sin(a) - s*cos(a)*sin(a));

	//apply transformation described by the matrix rot

	Mat transformedImage;
	originalImage.copyTo(transformedImage);


	cv::warpAffine(transformedImage, transformedImage, rot, transformedImage.size());

	return transformedImage;
}

} /* namespace decoder */
