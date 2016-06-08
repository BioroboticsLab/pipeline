#include "../Localizer.h"

#include <set>

#include <boost/filesystem.hpp>
#include <boost/range/adaptor/reversed.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <mxnetpredictor/MXNetPredictor.h>

#include "../datastructure/Tag.h"

/**
 * Scales a given OpenCV rectangle by a factor, conserving the rectangle's center.
 *
 * \param rectangle OpenCV rectangle to be scaled
 * \param scale factor by which the rectangle is scaled
 */
cv::Rect operator*(const cv::Rect rectangle, double scale) {
    cv::Size    s(static_cast<int>(rectangle.height * scale), static_cast<int>(rectangle.width * scale));
    cv::Point2i c(static_cast<int>(rectangle.x - (0.5 * (s.width - rectangle.width))),
                  static_cast<int>(rectangle.y - (0.5 * (s.height - rectangle.height))));
    return (cv::Rect(c, s));
}

namespace pipeline {

Localizer::Localizer() {}

Localizer::Localizer(const settings::localizer_settings_t &settings)
    : _settings(settings)
{
    initializeFilterNet();
}

Localizer::~Localizer()
{

}

void Localizer::initializeFilterNet()
{
    if (_settings.get_deeplocalizer_filter()) {
        const std::string modelPath = _settings.get_deeplocalizer_model_file();
        const std::string paramPath = _settings.get_deeplocalizer_param_file();
        if (!boost::filesystem::exists(modelPath) || !boost::filesystem::exists(paramPath)) {
            throw std::runtime_error( "Invalid path. Could not initalize FilterNet");
        } else {
            const auto tagSize = _settings.get_tag_size();
            _filterNet = std::make_unique<mx::MXNetPredictor>(
                            modelPath, paramPath, tagSize, tagSize, mx::CPU);
        }
    }
}

void Localizer::loadSettings(settings::localizer_settings_t &&settings)
{
    _settings = std::move(settings);

    initializeFilterNet();
}

void Localizer::loadSettings(settings::localizer_settings_t const& settings)
{
    _settings = settings;

    initializeFilterNet();
}

settings::localizer_settings_t Localizer::getSettings() const
{
    return _settings;
}

const cv::Mat& Localizer::getBlob() const
{
    return _blob;
}

void Localizer::setBlob(const cv::Mat& blob)
{
    _blob = blob;
}

const cv::Mat& Localizer::getCannyMap() const
{
    return _canny_map;
}

void Localizer::setCannyMap(const cv::Mat& cannyMap)
{
    _canny_map = cannyMap;
}

const cv::Mat& Localizer::getThresholdImage() const
{
    return _threshold_image;
}

void Localizer::setThresholdImage(const cv::Mat& thresholdImage)
{
    _threshold_image = thresholdImage;
}

std::vector<Tag> Localizer::process(PreprocessorResult&& preprocesorResult)
{
    _blob = highlightTags(preprocesorResult.preprocessedImage);

    std::vector<Tag> taglist = locateTagCandidates(_blob, preprocesorResult);

    if (_settings.get_deeplocalizer_filter()) {
        taglist = filterTagCandidates(std::move(taglist));
    }

    //taglist = filterDuplicates(std::move(taglist));

    return taglist;
}

cv::Mat Localizer::highlightTags(const cv::Mat &grayImage)
{
    cv::Mat thresholded = grayImage.clone();
    // Threshold each block (3x3 grid) of the image separately to
    // correct for minor differences in contrast across the image.
    for (int i = 0; i < thresholded.rows / 100; i++) {
        for (int j = 0; j < thresholded.cols / 100; j++) {
            cv::Mat block = thresholded.rowRange(100*i, 100*(i+1)).colRange(100*j, 100*(j+1));
            cv::Scalar mean_sobel = mean(block);
            double average_value = mean_sobel.val[0];
            cv::threshold(block, block, average_value + _settings.get_binary_threshold(),
                          255, cv::THRESH_BINARY);
        }
    }
    setThresholdImage(thresholded);

    const cv::Mat structDilation = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(2 * _settings.get_first_dilation_size() + 1,
                         2 * _settings.get_first_dilation_size() + 1),
                cv::Point(_settings.get_first_dilation_size(),
                          _settings.get_first_dilation_size())
    );

    cv::Mat dilated;
    cv::dilate(thresholded, dilated, structDilation, cv::Point(-1, -1),
               _settings.get_first_dilation_num_iterations());


    const cv::Mat structErosion = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(2 * _settings.get_erosion_size() + 1,
                         2 * _settings.get_erosion_size() + 1),
                cv::Point(_settings.get_erosion_size(),
                          _settings.get_erosion_size())
    );
    cv::Mat eroded;
    cv::erode(dilated, eroded, structErosion);


    const cv::Mat structSecondDilation = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(2 * _settings.get_second_dilation_size() + 1,
                         2 * _settings.get_second_dilation_size() + 1),
                cv::Point(_settings.get_second_dilation_size(),
                          _settings.get_second_dilation_size())
    );
    cv::Mat dilatedSecond;
    cv::dilate(eroded, dilatedSecond, structSecondDilation);

#ifdef PipelineStandalone
#ifdef DEBUG_LOCALIZER
    cv::namedWindow("original", cv::WINDOW_NORMAL);
    cv::imshow("original", grayImage);
    cv::namedWindow("thresholded", cv::WINDOW_NORMAL);
    cv::imshow("thresholded", thresholded);
    cv::namedWindow("dilated", cv::WINDOW_NORMAL);
    cv::imshow("dilated", dilated);
    cv::namedWindow("eroded", cv::WINDOW_NORMAL);
    cv::imshow("eroded", eroded);
    cv::namedWindow("dilatedSecond", cv::WINDOW_NORMAL);
    cv::imshow("dilatedSecond", dilatedSecond);
    cv::waitKey(0);
#endif
#endif

    return dilatedSecond;
}

std::vector<Tag> Localizer::locateTagCandidates(const cv::Mat &blobs, const PreprocessorResult &preprocessorResults)
{
    cv::Mat blobImage = blobs.clone();

    // find intra-connected white pixels
    std::vector<std::vector<cv::Point2i>> contours;
    cv::findContours(blobImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // extract contour bounding boxes for tag candidates
    std::vector<Tag> taglist;
    for (const auto &contour : contours) {
        // filter contours which are too big
        if (contour.size() < _settings.get_max_num_pixels() &&
            contour.size() > _settings.get_min_num_pixels())
        {
            cv::Rect rec = cv::boundingRect(contour) * 2;

            if (rec.width > static_cast<int>(_settings.get_tag_size()) ||
                rec.height > static_cast<int>(_settings.get_tag_size())) {
                continue;
            }

            if (rec.width < static_cast<int>(_settings.get_tag_size())) {
                const int offset = abs(rec.width - static_cast<int>(_settings.get_tag_size()));
                rec.x     = rec.x - offset / 2;
                rec.width = rec.width + offset;
            }

            if (rec.height < static_cast<int>(_settings.get_tag_size())) {
                const int offset = abs(rec.height - static_cast<int>(_settings.get_tag_size()));
                rec.y      = rec.y - offset / 2;
                rec.height = rec.height + offset;
            }

            //if rectangle is outside the possible image-coordinates => resize rectangle
            if ((rec.x + rec.width) > blobImage.cols) {
                rec.x -= abs(rec.x + rec.width - blobImage.cols);
            }

            if ((rec.y + rec.height) > blobImage.rows) {
                rec.y -= abs(rec.y + rec.height - blobImage.rows);
            }

            if (rec.x < 0) {
                rec.x = 0;
            }

            if (rec.y < 0) {
                rec.y = 0;
            }

            // if rectangle-size is big/small enough add it to Bounding Boxes
            taglist.emplace_back(rec, taglist.size() + 1, preprocessorResults);
        }
    }

    return taglist;
}

std::vector<Tag> Localizer::filterTagCandidates(std::vector<Tag> &&candidates)
{
    assert(_filterNet);

    std::cout << candidates.size() << " candidates before filtering" << std::endl;

    if (candidates.empty()) {
        return candidates;
    }

    for (Tag& candidate : candidates) {
        cv::Mat blob;
        candidate.getRepresentations().clahe.convertTo(blob, CV_32FC1, 1. / 255.);

        assert(unsigned(blob.cols) == _settings.get_tag_size() &&
               unsigned(blob.rows) == _settings.get_tag_size());
        assert(blob.channels() == 1);

        const float prob = _filterNet->predict(blob);

#ifdef PipelineStandalone
#ifdef DEBUG_LOCALIZER
        const double threshold = _settings.get_deeplocalizer_probability_threshold();
        if (prob >= threshold) {
            std::string title = std::string("filternet-") + std::to_string(prob);
            cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
            cv::imshow(title, blob);
            cv::waitKey(0);
            cv::destroyWindow(title);
        }
#endif
#endif

        candidate.setLocalizerScore(prob);
    }

    const double threshold = _settings.get_deeplocalizer_probability_threshold();

    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                       [threshold](pipeline::Tag const& tag)
                       {
                           return tag.getLocalizerScore() < threshold;
                       }
                    ), candidates.end());

    std::cout << candidates.size() << " candidates after filtering" << std::endl;

    return candidates;
}

std::vector<Tag> Localizer::filterDuplicates(std::vector<Tag> &&candidates)
{
    const double minOverlap = std::pow(_settings.get_tag_size() / 3., 2.);

    std::set<size_t> removalIndices;

    for (size_t firstIdx = 0; firstIdx < candidates.size(); ++firstIdx) {

        pipeline::Tag const& firstTag = candidates.at(firstIdx);

        double maxScore = firstTag.getLocalizerScore();

        typedef std::pair<size_t, double> idxScorePair;
        std::set<idxScorePair> overlappingTags;
        overlappingTags.insert({firstIdx, firstTag.getLocalizerScore()});

        for (size_t secondIdx = 0; secondIdx < candidates.size(); ++secondIdx) {

            if ((firstIdx != secondIdx) && !removalIndices.count(firstIdx) && !removalIndices.count(secondIdx)) {

                pipeline::Tag const& secondTag = candidates.at(secondIdx);

                const cv::Rect firstRoi = firstTag.getRoi();
                const cv::Rect secondRoi = secondTag.getRoi();
                const cv::Rect overlap = firstRoi & secondRoi;

                if (overlap.area() >= minOverlap) {
                    overlappingTags.insert({secondIdx, secondTag.getLocalizerScore()});
                    maxScore = std::max(maxScore, secondTag.getLocalizerScore());
                }
            }
        }


        size_t removalCount = 0;
        for (idxScorePair const& pair : overlappingTags) {

            if ((pair.second <= maxScore) && (removalCount < (overlappingTags.size() - 1))) {

                removalIndices.insert(pair.first);
                ++removalCount;
            }
        }
    }

    for (const size_t idx : boost::adaptors::reverse(removalIndices)) {

        candidates.erase(candidates.begin() + idx);
    }

    return candidates;
}
}
