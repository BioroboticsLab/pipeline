#pragma once

#include <opencv2/opencv.hpp>

#include "settings/LocalizerSettings.h"

#ifdef USE_DEEPLOCALIZER
#include <deeplocalizer/classifier/CaffeClassifier.h>
#endif

namespace pipeline {

class BoundingBox;
class Tag;

class Localizer {
private:
    cv::Mat _blob;
    cv::Mat _canny_map;
    cv::Mat _threshold_image;

    settings::localizer_settings_t _settings;

    /**
     * Highlight tag candidates in a comb image by intensity values
     *
     * @param grayImage the input image
     * @return image with highlighted tags
     */
    cv::Mat highlightTags(const cv::Mat &grayImage) ;

    /**
     *  Find blobs in the binary input image Ib and filter them by their size
     *
     * @param blobImage binary comb image with highlighted tag candidates
     * @return boundingBoxes output vector of size-filtered bounding boxes
     */
    std::vector<Tag> locateTagCandidates(cv::Mat blobImage, cv::Mat cannyEdgeMap, cv::Mat grayImage);

    std::vector<Tag> locateAllPossibleCandidates(cv::Mat const& grayImage);

#ifdef USE_DEEPLOCALIZER
    std::unique_ptr<deeplocalizer::CaffeClassifier> _caffeNet;
    std::unique_ptr<caffe::DataTransformer<float>> _caffeTransformer;

    std::vector<Tag> filterTagCandidates(std::vector<Tag>&& candidates);

    boost::optional<std::string> _modelPath;
    boost::optional<std::string> _paramPath;

    void initializeDeepLocalizer(deeplocalizer::CaffeClassifier* weightSharingNet);
#endif

    std::vector<Tag> filterDuplicates(std::vector<Tag>&& candidates);

public:
    Localizer();
#ifdef PipelineStandalone
    Localizer(const std::string &configFile);
#endif
    virtual ~Localizer() {}

    void loadSettings(settings::localizer_settings_t&& settings);
    void loadSettings(settings::localizer_settings_t const& settings);
#ifdef USE_DEEPLOCALIZER
    void loadSettings(settings::localizer_settings_t const& settings,
                      deeplocalizer::CaffeClassifier* weightSharingNet);
#endif

    settings::localizer_settings_t getSettings() const;

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

#ifdef USE_DEEPLOCALIZER
    deeplocalizer::CaffeClassifier* getCaffeNet() const;
#endif
};
}
