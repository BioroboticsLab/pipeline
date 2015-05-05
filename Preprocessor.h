#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "datastructure/settings.h"


namespace pipeline {

typedef struct{
	cv::Range row_range;
	cv::Range col_range;
}image_raster;

class Preprocessor{
private:
	settings::preprocessor_settings_t _options;
	cv::Mat _thresholdImage;
	cv::Mat _sobel;
	cv::Mat _optsImage;
	cv::Mat _honeyImage;


	cv::Mat _computeSobel(const cv::Mat &grayImage) const;
	std::vector<image_raster> _getRaster(cv::Mat image, int frame_size);
	cv::Mat _contrastStretching(const cv::Mat& image);
	void _equalizeHistogramm(cv::Mat& image);
	void _filterCombs(cv::Mat& image);
	cv::Mat _filterHoney(const cv::Mat& opt_image, const cv::Mat& orig_image);


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
