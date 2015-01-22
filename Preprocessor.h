/*
 * Preprocessor.h
 *
 *  Created on: 15.01.2015
 *      Author: mareikeziese
 */

#ifndef PREPROCESSOR_H_
#define PREPROCESSOR_H_


#include <fstream>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "datastructure/settings.h"


#ifdef PipelineStandalone
	#include "../config.h"
#endif



namespace decoder {


class Preprocessor{


private:

	preprocessor_settings_t _options;
	cv::Mat _thresholdImage;
	cv::Mat _sobel;
	void _filterCombs(cv::Mat& image);
	cv::Mat _computeSobel(const cv::Mat &grayImage) const;



protected:
public:
	Preprocessor();
	virtual ~Preprocessor();
	virtual cv::Mat process(const cv::Mat& image);
	virtual cv::Mat process(const std::string& filename);
	preprocessor_settings_t getOptions() const;
	void setOptions(preprocessor_settings_t options);
	void loadSettings(preprocessor_settings_t&& settings);
	const cv::Mat& getThresholdImage() const;
	void setThresholdImage(const cv::Mat& thresholdImage);
	const cv::Mat& getSobel() const;
	void setSobel(const cv::Mat& sobel);
};

} /* namespace decoder */

#endif /* PREPROCESSOR_H_ */
