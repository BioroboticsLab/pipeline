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


namespace decoder {
class Converter {
public:
    Converter();
    ~Converter();

    cv::Mat process(const std::string& filename) const;
    cv::Mat process(const cv::Mat& image) const;

private:
    bool checkValidFilename(const std::string& filename) const;
};
} /* namespace decoder_interface */

#endif /* CONVERTER_H_ */
