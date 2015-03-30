#pragma once

#include <opencv2/opencv.hpp>

#include "datastructure/settings.h"



namespace pipeline {

class BoundingBox;
class Tag;

class Localizer {
private:
	cv::Mat blob_;
	cv::Mat canny_map_;
	cv::Mat _threshold_image;

	settings::localizer_settings_t _settings;

	cv::Mat computeBlobs(const cv::Mat &sobel) ;
	cv::Mat highlightTags(const cv::Mat &grayImage) ;
	std::vector<Tag> locateTagCandidates(cv::Mat blobImage, cv::Mat cannyEdgeMap, cv::Mat grayImage);


public:
	Localizer();
#ifdef PipelineStandalone
	Localizer(const std::string &configFile);
#endif
	virtual ~Localizer();

	void loadSettings(settings::localizer_settings_t&& settings);

	const cv::Mat& getBlob() const;
	void setBlob(const cv::Mat& blob);
	const cv::Mat& getCannyMap() const;
	void setCannyMap(const cv::Mat& cannyMap);
	const cv::Mat& getGrayImage() const;
	void setGrayImage(const cv::Mat& grayImage);

	std::vector<Tag> process(cv::Mat &&originalImage, cv::Mat &&preprocessedImage);
	void reset();
	const cv::Mat& getThresholdImage() const;
	void setThresholdImage(const cv::Mat& thresholdImage);
};
}
