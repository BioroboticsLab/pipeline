/*
 * Recognizer.h
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#ifndef RECOGNIZER_H_
#define RECOGNIZER_H_

#include "./datastructure/Tag.h"
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


namespace decoder {

typedef struct {
	// Lower Threshold for Canny
	int canny_threshold_low = 70;

	// Higher Threshold for Canny
	int canny_threshold_high = 90;

	// major axis' minimum length
	int min_major_axis = 42;

	// major axis' maximum length
	int max_major_axis = 54;

	// minor axis' minimum length
	int min_minor_axis = 30;

	// minor axis' maximum length
	int max_minor_axis = 54;

	// threshold minimum number of edge pixels required to support an ellipse
	int threshold_edge_pixels = 25;

	// threshold vote
	int threshold_vote = 1800;

	// threshold best vote: if this vote is reached, the algorithm stops searching for other ellipses
	int threshold_best_vote = 3000;
} recognizer_settings_t;

class Recognizer {
private:
	recognizer_settings_t _settings;

    void detectXieEllipse(Tag &tag);

#ifdef PipelineStandalone
    void loadConfigVars(std::string filename);
    void visualizeEllipse(Ellipse const& ell, std::string const& title);
#endif

public:
    Recognizer();
#ifdef PipelineStandalone
    Recognizer(std::string configFile);
#endif
    virtual ~Recognizer() {}

	void loadSettings(recognizer_settings_t &&settings);

    std::vector<Tag> process(std::vector<Tag> &&taglist);
    void visualizeEllipse(Tag const& tag , Ellipse const& ell, std::string const& title);

	cv::Mat computeCannyEdgeMap(const cv::Mat &grayImage) const;
};
} /* namespace decoder */

#endif /* RECOGNIZER_H_ */
