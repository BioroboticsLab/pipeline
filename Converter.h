/*
 * Converter.h
 *
 *  Created on: 29.06.2014
 *      Author: mareikeziese
 */

#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <stdio.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include <fstream>

using namespace std;
using namespace cv;

namespace decoder {

class Converter {
public:
	Converter();
	virtual ~Converter();

	cv::Mat process(const string& filename);
    cv::Mat process(cv::Mat& image);

private:
	bool checkValidFilename(const string& filename);
};
} /* namespace decoder_interface */

#endif /* CONVERTER_H_ */
