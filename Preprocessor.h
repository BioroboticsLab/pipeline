#pragma once

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

namespace pipeline {
class Preprocessor{
private:
	settings::preprocessor_settings_t _options;
	cv::Mat _thresholdImage;
	cv::Mat _sobel;
	cv::Mat _optsImage;
	cv::Mat _honeyImage;


	cv::Mat _computeSobel(const cv::Mat &grayImage) const;
	std::vector<cv::Mat> _rasterImage(cv::Mat image, int frame_size);
	void _contrastStretching(cv::Mat& image);
	void _equalizeHistogramm(cv::Mat& image);
	void _filterCombs(cv::Mat& image);
	bool _filterHoney(cv::Mat& image);


public:
	Preprocessor();
	virtual ~Preprocessor();

	virtual cv::Mat process(const cv::Mat& image);
	virtual cv::Mat process(const std::string& filename);

	settings::preprocessor_settings_t getOptions() const;
	void setOptions(settings::preprocessor_settings_t options);
	void loadSettings(settings::preprocessor_settings_t&& settings);

	const cv::Mat& getThresholdImage() const;
	void setThresholdImage(const cv::Mat& thresholdImage);

	const cv::Mat& getSobel() const;
	void setSobel(const cv::Mat& sobel);
	const cv::Mat& getHoneyImage() const;
	void setHoneyImage(const cv::Mat& honeyImage);
	const cv::Mat& getOptsImage() const;
	void setOptsImage(const cv::Mat& optsImage);
};
}
