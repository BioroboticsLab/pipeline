#include "../Preprocessor.h"

namespace pipeline {

settings::preprocessor_settings_t Preprocessor::getOptions() const {
    return _settings;
}

void Preprocessor::setOptions(settings::preprocessor_settings_t options) {
    _settings = options;
}

cv::Mat Preprocessor::process(const std::string& filename) {
    cv::Mat image = cv::imread(filename);

    return process(image);
}

cv::Mat Preprocessor::process(const cv::Mat &image) {
    cv::Mat grayImage;

    if (image.type() == CV_8UC1) {
        image.copyTo(grayImage);
    } else {
        cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    }

    _preprocessedImage = dnnPreprocess(grayImage);

    return _preprocessedImage;
}

void Preprocessor::loadSettings(settings::preprocessor_settings_t &&settings) {
    _settings = std::move(settings);
}

void Preprocessor::loadSettings(settings::preprocessor_settings_t &settings) {
    _settings = settings;
}

cv::Mat Preprocessor::dnnPreprocess(const cv::Mat &grayImage)
{
    cv::Mat preprocessed = grayImage.clone();

    localHistEq(preprocessed);
    adaptiveThresholding(preprocessed);

    return preprocessed;
}

void Preprocessor::adaptiveThresholding(cv::Mat &image, bool use_binary_image)
{
    static double max_value = 255;
    static size_t block_size = 51;
    static double weight_original = 0.7;
    static double weight_threshold = 0.3;

    cv::Mat imageThreshold(image.rows, image.cols, CV_8UC1);
    cv::adaptiveThreshold(image, imageThreshold, max_value,
                          cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, block_size, 0);
    if (use_binary_image) {
        image = imageThreshold;
    } else {
        cv::addWeighted(image, weight_original, imageThreshold,
                        weight_threshold, 0 /*gamma*/, image);
    }
}

void Preprocessor::localHistEq(cv::Mat &image)
{
    // TODO: add to settings
    static size_t tag_size = 100;

    static const int clip_limit = 2;
    static const cv::Size tile_size(tag_size, tag_size);
    auto clahe = cv::createCLAHE(clip_limit, tile_size);

    clahe->apply(image, image);
}

const cv::Mat &Preprocessor::getPreprocessedImage() const
{
    return _preprocessedImage;
}

} /* namespace pipeline */

