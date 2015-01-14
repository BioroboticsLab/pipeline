/*
 * Localizer.h
 *
 *  Created on: 03.07.2014
 *      Author: mareikeziese
 */

#ifndef LOCALIZER_H_
#define LOCALIZER_H_

#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "datastructure/BoundingBox.h"
#include "datastructure/Tag.h"

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#endif


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
	cv::Mat gray_image_;
	cv::Mat sobel_;
	cv::Mat blob_;
	cv::Mat canny_map_;

	localizer_settings_t _settings;

	cv::Mat computeSobelMap(const cv::Mat &grayImage) const;
	cv::Mat computeBlobs(const cv::Mat &sobel) const;
	cv::Mat highlightTags(const cv::Mat &grayImage) const;
	std::vector<Tag> locateTagCandidates(cv::Mat blobImage, cv::Mat cannyEdgeMap, cv::Mat grayImage);


#ifdef PipelineStandalone
	void loadConfigVars(const std::string &filename);
#endif

public:
	Localizer();
#ifdef PipelineStandalone
	Localizer(const std::string &configFile);
#endif
	virtual ~Localizer();

	void loadSettings(localizer_settings_t&& settings);

	const cv::Mat& getBlob() const;
	void setBlob(const cv::Mat& blob);
	const cv::Mat& getCannyMap() const;
	void setCannyMap(const cv::Mat& cannyMap);
	const cv::Mat& getGrayImage() const;
	void setGrayImage(const cv::Mat& grayImage);
	const cv::Mat& getSobel() const;
	void setSobel(const cv::Mat& sobel);

	std::vector<Tag> process(cv::Mat &&image);
	void reset();
};
} /* namespace decoder */

#endif /* LOCALIZER_H_ */
