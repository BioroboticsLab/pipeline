/*
 * Transformer.h
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#ifndef TRANSFORMER_H_
#define TRANSFORMER_H_


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

class Transformer {
private:

	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

	Mat _ellipseTransform( Ellipse ell, Mat originalImage);
	void _transformImages(Tag &t);

public:

	/**************************************
	 *
	 * 			constructor
	 *
	 **************************************/
	Transformer();
	virtual ~Transformer();

	/**************************************
	 *
	 * 			stuff
	 *
	 **************************************/

    vector<Tag> process(const vector<Tag> &taglist);
};

} /* namespace decoder */

#endif /* TRANSFORMER_H_ */
