/*
 * Converter.h
 *
 *  Created on: 29.06.2014
 *      Author: mareikeziese
 */

#ifndef CONVERTER_H_
#define CONVERTER_H_

#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>
#include <string>

using namespace cv;

namespace decoder {
class Converter {
public:
    Converter();
    virtual ~Converter();

    cv::Mat process(const std::string& filename);
    cv::Mat process(cv::Mat& image);

private:
    bool checkValidFilename(const std::string& filename);
};
} /* namespace decoder_interface */

#endif /* CONVERTER_H_ */
