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

using namespace std;
using namespace cv;

namespace decoder {
//TODO
namespace RecognizerParams {
// Lower Threshold for Canny
static const int canny_threshold_low = 70;

// Higher Threshold for Canny
static const int canny_threshold_high = 90;

// major axis' minimum length
static const int min_major_axis = 42;

// major axis' maximum length
static const int max_major_axis = 54;

// minor axis' minimum length
static const int min_minor_axis = 30;

// minor axis' maximum length
static const int max_minor_axis = 54;

// threshold minimum number of edge pixels required to support an ellipse
static const int threshold_edge_pixels = 25;

// threshold vote
static const int threshold_vote = 1800;

// threshold best vote: if this vote is reached, the algorithm stops searching for other ellipses
static const int threshold_best_vote = 3000;
}

class Recognizer {
private:
    int RECOGNIZER_LCANNYTHRES;
    int RECOGNIZER_HCANNYTHRES;

    int RECOGNIZER_MIN_MAJOR;
    int RECOGNIZER_MAX_MAJOR;
    int RECOGNIZER_MIN_MINOR;
    int RECOGNIZER_MAX_MINOR;
    int RECOGNIZER_THRESHOLD_EDGE;
    int RECOGNIZER_THRESHOLD_VOTE;
    int RECOGNIZER_THRESHOLD_BEST_VOTE;

    void detectXieEllipse(Tag &tag);
    Mat computeCannyEdgeMap(Mat grayImage);

#ifdef PipelineStandalone
    void loadConfigVars(string filename);
#endif
    void loadConfigVars();

public:
    Recognizer();
#ifdef PipelineStandalone
    Recognizer(string configFile);
#endif
    virtual ~Recognizer() {}

    vector<Tag> process(vector<Tag> &&taglist);
    void visualizeEllipse(Tag const& tag , Ellipse const& ell, std::string const& title);
};
} /* namespace decoder */

#endif /* RECOGNIZER_H_ */
