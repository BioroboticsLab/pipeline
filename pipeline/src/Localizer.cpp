#include "../Localizer.h"

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
            std::cerr << "Invalid path. Could not initialize FilterNet" << std::endl;
            _filterNet = std::unique_ptr<mx::MXNetPredictor>();
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

std::vector<Tag> Localizer::process(cv::Mat &&originalImage, cv::Mat &&preprocessedImage)
{
    _blob = highlightTags(preprocessedImage);

    //std::vector<Tag> taglist= locateTagCandidates(_blob, preprocessedImage, originalImage);
    std::vector<Tag> taglist= locateAllPossibleCandidates(originalImage, preprocessedImage);

    if (_settings.get_deeplocalizer_filter()) {
        taglist = filterTagCandidates(std::move(taglist));
    }

    //taglist = filterDuplicates(std::move(taglist));

    return taglist;
}

cv::Mat Localizer::highlightTags(const cv::Mat &grayImage)
{
    cv::Mat image;
    grayImage.copyTo(image);
    cv::Mat binarizedImage;

    // Threshold each block (3x3 grid) of the image separately to
    // correct for minor differences in contrast across the image.
    for (int i = 0; i < image.rows / 100; i++) {
        for (int j = 0; j < image.cols / 100; j++) {
            cv::Mat block = image.rowRange(100*i, 100*(i+1)).colRange(100*j, 100*(j+1));
            cv::Scalar mean_sobel = mean(block);
            double average_value = mean_sobel.val[0];
            cv::threshold(block, block, average_value+ _settings.get_binary_threshold(), 255, cv::THRESH_BINARY);
        }
    }

    cv::Mat imageCopy;
    setThresholdImage(image);
    image.copyTo(imageCopy);

#ifdef PipelineStandalone
#ifdef DEBUG_LOCALIZER
    cv::namedWindow("binarized Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("binarized Image", imageCopy);
    cv::waitKey(0);
    cv::destroyWindow("binarized Image");
#endif
#endif

    cv::Mat imageCopy2 = binarizedImage.clone();

    cv::Mat dilatedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                     cv::Size(2 * _settings.get_first_dilation_size() + 1,
                                                              2 * _settings.get_first_dilation_size() + 1),
                                                     cv::Point(_settings.get_first_dilation_size(),
                                                               _settings.get_first_dilation_size()));
    cv::dilate(imageCopy, imageCopy, dilatedImage, cv::Point(-1, -1),
               _settings.get_first_dilation_num_iterations());

#ifdef PipelineStandalone
#ifdef DEBUG_LOCALIZER
    cv::namedWindow("First Dilate", cv::WINDOW_AUTOSIZE);
    cv::imshow("First Dilate", imageCopy);
    cv::waitKey(0);
    cv::destroyWindow("First Dilate");
#endif
#endif

    //erosion
    cv::Mat erodedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                                    cv::Size(2 * _settings.get_erosion_size() + 1,
                                                             2 * _settings.get_erosion_size() + 1),
                                                    cv::Point(_settings.get_erosion_size(),
                                                              _settings.get_erosion_size()));
    cv::erode(imageCopy, imageCopy, erodedImage);

#ifdef PipelineStandalone
#ifdef DEBUG_LOCALIZER
    cv::namedWindow("First Erode", cv::WINDOW_AUTOSIZE);
    cv::imshow("First Erode", imageCopy);
    cv::waitKey(0);
    cv::destroyWindow("First Erode");
#endif
#endif

    dilatedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                             cv::Size(2 * _settings.get_second_dilation_size() + 1,
                                                      2 * _settings.get_second_dilation_size() + 1),
                                             cv::Point(_settings.get_second_dilation_size(),
                                                       _settings.get_second_dilation_size()));
    cv::dilate(imageCopy, imageCopy, dilatedImage);

#ifdef PipelineStandalone
#ifdef DEBUG_LOCALIZER
    cv::namedWindow("My Window", cv::WINDOW_AUTOSIZE);
    cv::imshow("My Window", imageCopy);
    cv::waitKey(0);
    cv::destroyWindow("My Window");
#endif
#endif

    return imageCopy;
}

std::vector<Tag> Localizer::locateAllPossibleCandidates(const cv::Mat &grayImage, const cv::Mat &preprocessedImage)
{
    const int roiSize  = _settings.get_tag_size();
    const int stepSize = static_cast<int>(roiSize * 0.45);

    cv::Mat imageWithBorder;
    cv::copyMakeBorder(grayImage, imageWithBorder, roiSize, roiSize, roiSize, roiSize, cv::BORDER_REPLICATE);
    cv::Mat ppWithBorder;
    cv::copyMakeBorder(preprocessedImage, ppWithBorder, roiSize, roiSize, roiSize, roiSize, cv::BORDER_REPLICATE);

    std::vector<Tag> taglist;

    auto addRoi = [&](const int x, const int y) {
        const cv::Rect roi(x, y, roiSize, roiSize);
        const cv::Mat subImage(imageWithBorder, roi);

        const cv::Rect originalImageRoi(x - roiSize, y - roiSize, roiSize, roiSize);
        Tag tag(originalImageRoi, subImage.clone(), taglist.size() + 1);

        const cv::Mat subImagePp(ppWithBorder, roi);
        tag.setCannySubImage(subImagePp.clone());

        taglist.push_back(tag);
    };

    int y = roiSize + 1;
    do {

        int x = roiSize + 1;
        do {
            addRoi(x, y);

            x += stepSize;
        } while (x < (grayImage.size[1] + roiSize - 1));

        y += stepSize;
    } while (y < (grayImage.size[0] + roiSize - 1));

    return taglist;
}

std::vector<Tag> Localizer::locateTagCandidates(cv::Mat blobImage_old,
                                                cv::Mat preprocessedImage, cv::Mat grayImage)
{
    std::vector<Tag>  taglist;
    std::vector<std::vector<cv::Point2i> > contours;

    cv::Mat blobImage = blobImage_old.clone();

    //find intra-connected white pixels
    cv::findContours(blobImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    //extract contour bounding boxes for tag candidates
    for (const auto &contour : contours) {
        //filter contours which are too big
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

            Tag tag(rec, taglist.size() + 1);
            cv::Mat sub_image_orig(grayImage, rec);
            cv::Mat subImageOrig_cp = sub_image_orig.clone();
            tag.setOrigSubImage(subImageOrig_cp);
            cv::Mat sub_image_pp(preprocessedImage, rec);
            tag.setCannySubImage(sub_image_pp.clone());

            taglist.push_back(tag);

        }
    }

    return taglist;
}

std::vector<Tag> Localizer::filterTagCandidates(std::vector<Tag> &&candidates)
{
    assert(_filterNet);

    if (candidates.empty()) {
        return candidates;
    }

    for (Tag& candidate : candidates) {
        // TODO: check if float
        cv::Mat blob;// = candidate.getCannySubImage().clone();
        candidate.getCannySubImage().convertTo(blob, CV_32FC1, 1. / 255.);

        assert(unsigned(blob.cols) == _settings.get_tag_size() &&
               unsigned(blob.rows) == _settings.get_tag_size());
        assert(blob.channels() == 1);

        const float prob = _filterNet->predict(blob);

        /*
        cv::namedWindow(std::to_string(prob), cv::WINDOW_AUTOSIZE);
        cv::imshow(std::to_string(prob), blob);
        cv::waitKey(0);
        cv::destroyWindow(std::to_string(prob));
        */

        candidate.setLocalizerScore(prob);
    }

    const double threshold = _settings.get_deeplocalizer_probability_threshold();

    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
                       [threshold](pipeline::Tag const& tag)
                       {
                           return tag.getLocalizerScore() < threshold;
                       }
                    ), candidates.end());

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

                const cv::Rect firstRoi = firstTag.getBox();
                const cv::Rect secondRoi = secondTag.getBox();
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
