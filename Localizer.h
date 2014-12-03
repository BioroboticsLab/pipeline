/*
 * Localizer.h
 *
 *  Created on: 03.07.2014
 *      Author: mareikeziese
 */

#ifndef LOCALIZER_H_
#define LOCALIZER_H_

#include "./datastructure/BoundingBox.h"
#include "./datastructure/Tag.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

using namespace std;
using namespace cv;

namespace decoder {
typedef struct {
	// Threshold for binarisation
	int binary_threshold = 29;

	// number of first dilation iterations
	int dilation_1_iteration_number = 4;

	// size of dilation-radius
	int dilation_1_size = 2;

	// erosion-size
	int erosion_size = 25;

	// second dilation-size
	int dilation_2_size = 2;

	// maximal size of a possible tag
	unsigned int max_tag_size =  250;

	// minimal size of bounding box
	int min_tag_size =  100;
} localizer_settings_t;

class Localizer {
private:
    Mat gray_image_;
    Mat sobel_;
    Mat blob_;
    Mat canny_map_;

	localizer_settings_t _settings;

    Mat computeSobelMap(Mat grayImage);
    Mat computeBlobs(Mat sobel);
    Mat highlightTags(Mat &grayImage);
	vector<Tag> locateTagCandidates(Mat blobImage, Mat cannyEdgeMap, Mat grayImage);

#ifdef PipelineStandalone
	void loadConfigVars(string filename);
#endif

public:
	Localizer();
#ifdef PipelineStandalone
	Localizer(string configFile);
#endif
	virtual ~Localizer();

	void loadSettings(localizer_settings_t&& settings);

	const Mat& getBlob() const;
    void setBlob(const Mat& blob);
    const Mat& getCannyMap() const;
    void setCannyMap(const Mat& cannyMap);
    const Mat& getGrayImage() const;
    void setGrayImage(const Mat& grayImage);
    const Mat& getSobel() const;
    void setSobel(const Mat& sobel);

    vector<Tag> process(Mat &&image);
    void reset();
};
} /* namespace decoder */

#endif /* LOCALIZER_H_ */
