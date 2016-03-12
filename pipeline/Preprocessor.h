#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

#include "settings/PreprocessorSettings.h"

namespace pipeline {

class Preprocessor{
public:
    Preprocessor() {};
    virtual ~Preprocessor() {};

    virtual cv::Mat process(const cv::Mat& image);
    virtual cv::Mat process(const std::string& filename);

    settings::preprocessor_settings_t getOptions() const;
    void setOptions(settings::preprocessor_settings_t options);
    void loadSettings(settings::preprocessor_settings_t&& settings);
    void loadSettings(settings::preprocessor_settings_t& settings);

    static cv::Mat dnnPreprocess(const cv::Mat& grayImage);
    static void adaptiveThresholding(cv::Mat& image, bool use_binary_image = false);
    static void localHistEq(cv::Mat& image);

    const cv::Mat& getPreprocessedImage() const;

private:
    settings::preprocessor_settings_t _settings;

    cv::Mat _preprocessedImage;
};
}
