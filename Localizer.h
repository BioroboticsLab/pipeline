/*
 * Localizer.h
 *
 *  Created on: 03.07.2014
 *      Author: mareikeziese
 */

#ifndef LOCALIZER_H_
#define LOCALIZER_H_

#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <math.h>
#include <fstream>
#include "BoundingBox.h"
#include "../config.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

using namespace std;
using namespace cv;

namespace decoder {

class Localizer {
public:
	Localizer();
	Localizer(string configFile);

	virtual ~Localizer();

	vector<BoundingBox> process(Mat image);
	void reset();

	const Mat& getBlob() const {
		return blob_;
	}

	void setBlob(const Mat& blob) {
		blob_ = blob;
	}

	const Mat& getCannyMap() const {
		return canny_map_;
	}

	void setCannyMap(const Mat& cannyMap) {
		canny_map_ = cannyMap;
	}

	const Mat& getGrayImage() const {
		return gray_image_;
	}

	void setGrayImage(const Mat& grayImage) {
		gray_image_ = grayImage;
	}

	const Mat& getSobel() const {
		return sobel_;
	}

	void setSobel(const Mat& sobel) {
		sobel_ = sobel;
	}

private:

	Mat gray_image_;
	Mat sobel_;
	Mat blob_;
	Mat canny_map_;

	Mat computeSobelMap(Mat grayImage);
	Mat computeBlobs(Mat sobel);
	Mat highlightTags(Mat &grayImage);
	vector<BoundingBox> locateTagCandidates(Mat blobImage, Mat cannyEdgeMap, Mat grayImage);
	Mat computeCannyEdgeMap(Mat grayImage);
	void loadConfigVars(string filename);

	 int LOCALIZER_LCANNYTHRES;
	 int LOCALIZER_HCANNYTHRES;
	 int LOCALIZER_BINTHRES;
	 int LOCALIZER_DILATION_1_ITERATIONS;
	 int LOCALIZER_DILATION_1_SIZE;
	 int LOCALIZER_EROSION_SIZE;
	 int LOCALIZER_DILATION_2_SIZE;
	 int LOCALIZER_MAXTAGSIZE;
	 int LOCALIZER_MINTAGSIZE;







};

} /* namespace decoder */

#endif /* LOCALIZER_H_ */
