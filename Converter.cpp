/*
 * Converter.cpp
 *
 *  Created on: 29.06.2014
 *      Author: mareikeziese
 */

#include "Converter.h"
#include <iostream>
#include <string>

namespace decoder {
Converter::Converter() {
}

cv::Mat Converter::process(const std::string& filename) const {
    cv::Mat image = cv::imread(filename);
    return process(image);
}

cv::Mat Converter::process(const cv::Mat &image) const
{
    cv::Mat grayImage;

    // convert image to grayscale (not needed later because images will already be grayscale)
    cvtColor(image, grayImage, CV_BGR2GRAY);

    return grayImage;
}

Converter::~Converter() = default;

/**
 * checks if the filename is valid for this converter
 */
bool Converter::checkValidFilename(const std::string& filename) const {
    //check if filename is given
    if (filename.size() == 0) {
        return false;
    }

    //check if file exists
    std::ifstream f(filename.c_str());
    if (!f.good()) {
        f.close();
        return false;
    }
    f.close();

    //check if file has the right extension
    if (filename.substr(filename.find_last_of(".") + 1) != "png" ||
      filename.substr(filename.find_last_of(".") + 1) != "jpeg") {
        return false;
    }
    return true;
}
} /* namespace decoder_interface */
