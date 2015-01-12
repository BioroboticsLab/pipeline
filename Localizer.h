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
#include <boost/lexical_cast.hpp>
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <math.h>
#include <fstream>
#include "./datastructure/BoundingBox.h"
#include "./datastructure/Tag.h"
#include "../config.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

using namespace std;
using namespace cv;

namespace decoder {


struct SubImage{
	Mat image;
	int x;
	int y;
};



/*
 * vars that are usually be filled by an ini file
 */

struct LocalizerOptions{
	float resizeRatio;
	int initThres1;
	float minMean1;
	float maxMean1;
	int initThres2;
	float minMean2;
	float maxMean2;
	int dilateSize1;
	int erodeSize;
	int dilateSize2;
	int minSizeComb;
	int blurSize;
	int maxTagSize;
	int minTagSize;
};

class Localizer {

private:

	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/

	LocalizerOptions _options;


	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

	Mat _computeSobel(Mat grayImage);
	Mat computeBlobs(Mat binarizedImage, Mat grayImage);
	vector <Tag> _locateTagCandidates(Mat blobImage, Mat grayImage);
	cv::Mat _generateThresholdImage(cv::Mat image, int initial_thres, float min_mean, float max_mean);
	void _removeCombs(cv::Mat &image);
	void _filterNoise(cv::Mat &image);
	void loadConfigVars(string filename);
	void _filterColors(vector<Tag> &taglist);
	vector <SubImage> _divideImage(Mat image);

public:

	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/

	Localizer();
	Localizer(string configFile);
	Localizer(LocalizerOptions options);

	virtual ~Localizer();

	/**************************************
	 *
	 * 			getter/setter
	 *
	 **************************************/


	const LocalizerOptions& getOptions() const;
	void setOptions(const LocalizerOptions& options);


	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

	vector<Tag> process(Mat image);
	void reset();

};

} /* namespace decoder */


#endif /* LOCALIZER_H_ */
