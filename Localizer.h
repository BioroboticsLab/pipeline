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
#include "./datastructure/BoundingBox.h"
#include "./datastructure/Tag.h"

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#endif

using namespace std;
using namespace cv;

namespace decoder {

//TODO
namespace LocalizerParams {
// Threshold for binarisation
static const int binary_threshold = 29;

// number of first dilation iterations
static const int dilation_1_interation_number = 4;

// size of dilation-radius
static const int dilation_1_size= 2;

// erosion-size
static const int erosion_size = 25;

// second dilation-size
static const int dilation_2_size = 2;

// maximal size of a possible tag
static const int max_tag_size =  250;

// minimal size of bounding box
static const int min_tag_size =  100;
}

class Localizer {

private:

	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/
	Mat gray_image_;
	Mat sobel_;
	Mat blob_;
	Mat canny_map_;

	/*
	 * vars that we be filled by an ini file
	 */

	int LOCALIZER_BINTHRES;
	int LOCALIZER_DILATION_1_ITERATIONS;
	int LOCALIZER_DILATION_1_SIZE;
	int LOCALIZER_EROSION_SIZE;
	int LOCALIZER_DILATION_2_SIZE;
	unsigned int LOCALIZER_MAXTAGSIZE;
	int LOCALIZER_MINTAGSIZE;

	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

	Mat computeSobelMap(Mat grayImage);
	Mat computeBlobs(Mat sobel);
	Mat highlightTags(Mat &grayImage);
	vector<Tag> locateTagCandidates(Mat blobImage, Mat cannyEdgeMap,
			Mat grayImage);

	void loadConfigVars(string filename);
    void loadConfigVars();

public:

	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/

	Localizer();
	Localizer(string configFile);

	virtual ~Localizer();

	/**************************************
	 *
	 * 			getter/setter
	 *
	 **************************************/

	const Mat& getBlob() const;
	void setBlob(const Mat& blob);
	const Mat& getCannyMap() const;
	void setCannyMap(const Mat& cannyMap);
	const Mat& getGrayImage() const;
	void setGrayImage(const Mat& grayImage);
	int getLocalizerBinthres() const;
	void setLocalizerBinthres(int localizerBinthres);
	int getLocalizerDilation1Iterations() const;
	void setLocalizerDilation1Iterations(int localizerDilation1Iterations);
	int getLocalizerDilation1Size() const;
	void setLocalizerDilation1Size(int localizerDilation1Size);
	int getLocalizerDilation2Size() const;
	void setLocalizerDilation2Size(int localizerDilation2Size);
	int getLocalizerErosionSize() const;
	void setLocalizerErosionSize(int localizerErosionSize);
	int getLocalizerHcannythres() const;
	void setLocalizerHcannythres(int localizerHcannythres);
	int getLocalizerLcannythres() const;
	void setLocalizerLcannythres(int localizerLcannythres);
	int getLocalizerMaxtagsize() const;
	void setLocalizerMaxtagsize(int localizerMaxtagsize);
	int getLocalizerMintagsize() const;
	void setLocalizerMintagsize(int localizerMintagsize);
	const Mat& getSobel() const;
	void setSobel(const Mat& sobel);

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
