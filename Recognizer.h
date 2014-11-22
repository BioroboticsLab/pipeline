/*
 * Recognizer.h
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#ifndef RECOGNIZER_H_
#define RECOGNIZER_H_

#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include "./datastructure/Tag.h"

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#endif

using namespace std;
using namespace cv;

namespace decoder {


class Recognizer {
private:

	/**************************************
	 *
	 * 			members
	 *
	 **************************************/

	int RECOGNIZER_LCANNYTHRES;
	int RECOGNIZER_HCANNYTHRES;

	int RECOGNIZER_MIN_MAJOR;
	int RECOGNIZER_MAX_MAJOR;
	int RECOGNIZER_MIN_MINOR;
	int RECOGNIZER_MAX_MINOR;
	int RECOGNIZER_THRESHOLD_EDGE;
	int RECOGNIZER_THRESHOLD_VOTE;
	int RECOGNIZER_THRESHOLD_BEST_VOTE;


	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

	void loadConfigVars(string filename);
	void detectXieEllipse(Tag &tag);
	Mat computeCannyEdgeMap(Mat grayImage);

public:

	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/
	Recognizer();
	Recognizer(string configFile);
	virtual ~Recognizer();

	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

	vector<Tag> process(vector<Tag> taglist);
};


/**************************************
 *
 * 			other stuff
 *
 **************************************/

bool compareVote(Ellipse a, Ellipse b);

} /* namespace decoder */

#endif /* RECOGNIZER_H_ */
