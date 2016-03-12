#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "datastructure/PreprocessorResults.h"
#include "settings/PreprocessorSettings.h"

namespace pipeline {

class Preprocessor{
public:
    Preprocessor() {};
    virtual ~Preprocessor() {};

    virtual PreprocessorResult process(const cv::Mat& image);
    virtual PreprocessorResult process(const std::string& filename);

    settings::preprocessor_settings_t getSettings() const;
    void loadSettings(settings::preprocessor_settings_t&& settings);
    void loadSettings(settings::preprocessor_settings_t& settings);

    cv::Mat dnnPreprocess(const cv::Mat& grayImage);
    cv::Mat pipelinePreprocess(const cv::Mat& grayImage);

private:
    settings::preprocessor_settings_t _settings;

    void adaptiveThresholding(cv::Mat& image, bool use_binary_image = false);
    void localHistEq(cv::Mat& image);

    void contrastStretching(cv::Mat& image);
    void filterHoney(cv::Mat& contrastStretchedImage, cv::Mat const& originalImage);
    void equalizeHistogram(cv::Mat& image);
    void filterCombs(cv::Mat& sobel);
    void computeSobel(cv::Mat& image);

    typedef struct {
        cv::Range rowRange;
        cv::Range colRange;
    } ImageRaster;
    static std::vector<ImageRaster> getRaster(const cv::Mat& image, const unsigned int frameSize);

};
}
