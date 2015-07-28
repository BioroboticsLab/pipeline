#include "Localizer.h"

#include <opencv2/highgui/highgui.hpp>

#include "datastructure/Tag.h"

#ifdef USE_DEEPLOCALIZER
#include <deeplocalizer/classifier/DataReader.h>

namespace deeplocalizer_config {
const std::string model_file("/home/ben/dev/deeplocalizer-data/from_aws/models/conv12_conv48_fc1024_fc_2/deploy.prototxt");
const std::string trained_file("/home/ben/dev/deeplocalizer-data/from_aws/models/conv12_conv48_fc1024_fc_2/model_iter_20000.caffemodel");
const float probability_threshold = 0.5f;
}

namespace {

caffe::TransformationParameter getTransformationParameter() {
    caffe::TransformationParameter param;
    param.set_scale(1.f / 255.f);

    return param;
}

}

#endif

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

Localizer::Localizer()
#ifdef USE_DEEPLOCALIZER
    : _caffeNet(deeplocalizer_config::model_file, deeplocalizer_config::trained_file)
    // TODO: set batch size according to number of localizer candidates
    , _caffeTransformer(getTransformationParameter(), caffe::TEST)
    #endif
{
}

const cv::Mat& Localizer::getBlob() const {
    return _blob;
}

void Localizer::setBlob(const cv::Mat& blob) {
    _blob = blob;
}

const cv::Mat& Localizer::getCannyMap() const {
    return _canny_map;
}

void Localizer::setCannyMap(const cv::Mat& cannyMap) {
    _canny_map = cannyMap;
}

const cv::Mat& Localizer::getThresholdImage() const {
    return _threshold_image;
}

void Localizer::setThresholdImage(const cv::Mat& thresholdImage) {
    _threshold_image = thresholdImage;
}

std::vector<Tag> Localizer::process(cv::Mat &&originalImage, cv::Mat &&preprocessedImage){

    // locate the tags using the sobel map
    _blob = highlightTags(preprocessedImage);

    std::vector<Tag> taglist= locateTagCandidates(_blob, _canny_map,originalImage);
    //std::vector<Tag> taglist= locateAllPossibleCandidates(originalImage);

    return taglist;
}

cv::Mat Localizer::highlightTags(const cv::Mat &grayImage)  {
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

std::vector<Tag> Localizer::locateTagCandidates(cv::Mat blobImage_old,
                                                cv::Mat /*cannyEdgeMap*/, cv::Mat grayImage) {
    std::vector<Tag>  taglist;
    std::vector<std::vector<cv::Point2i> > contours;

    cv::Mat blobImage = blobImage_old.clone();

    //find intra-connected white pixels
    cv::findContours(blobImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    //extract contour bounding boxes for tag candidates
    for (const auto &contour : contours) {
        //filter contours which are too big
        if (contour.size() < static_cast<unsigned int>( _settings.get_max_tag_size())) {
            cv::Rect rec = cv::boundingRect(contour) * 2;

            if (rec.width < _settings.get_min_bounding_box_size()) {
                const int offset = abs(rec.width - _settings.get_min_bounding_box_size());
                rec.x     = rec.x - offset / 2;
                rec.width = rec.width + offset;
            }

            if (rec.height < _settings.get_min_bounding_box_size()) {
                const int offset = abs(rec.height - _settings.get_min_bounding_box_size());
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

            taglist.push_back(tag);

        }
    }

#ifdef USE_DEEPLOCALIZER
    taglist = filterTagCandidates(std::move(taglist));
#endif

    return taglist;
}

std::vector<Tag> Localizer::locateAllPossibleCandidates(const cv::Mat &grayImage)
{
    const int roiSize  = _settings.get_min_bounding_box_size();
    const int stepSize = static_cast<int>(roiSize * 0.45);

    cv::Mat imageWithBorder;
    cv::copyMakeBorder(grayImage, imageWithBorder, roiSize, roiSize, roiSize, roiSize, cv::BORDER_REPLICATE);

    std::vector<Tag> taglist;

    auto addRoi = [&](const int x, const int y) {
        const cv::Rect roi(x, y, roiSize, roiSize);
        const cv::Mat subImage(imageWithBorder, roi);

        const cv::Rect originalImageRoi(x - roiSize, y - roiSize, roiSize, roiSize);
        Tag tag(originalImageRoi, subImage.clone(), taglist.size() + 1);
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

#ifdef USE_DEEPLOCALIZER
    taglist = filterTagCandidates(std::move(taglist));
#endif

    return taglist;
}

#ifdef USE_DEEPLOCALIZER
std::vector<Tag> Localizer::filterTagCandidates(std::vector<Tag> &&candidates)
{
    _caffeNet.setBatchSize(candidates.size());

    std::vector<caffe::Datum> data;

    for (Tag const& candidate : candidates) {
        cv::Mat const& blob = candidate.getOrigSubImage();
        assert(blob.cols == 100 && blob.rows == 100);
        assert(blob.channels() == 1);

        caffe::Datum datum;
        caffe::CVMatToDatum(blob, &datum);

        data.push_back(std::move(datum));
    }

    caffe::Blob<float> caffeData;
    caffeData.Reshape(candidates.size(), 1, 100, 100);
    _caffeTransformer.Transform(data, &caffeData);

    std::vector<std::vector<float>> probabilityMatrix = _caffeNet.forward(caffeData);

    std::vector<size_t> removalIndices;
    size_t idx = probabilityMatrix.size();
    while (idx > 0) {
        if (probabilityMatrix[idx-1][0] > deeplocalizer_config::probability_threshold) {
            removalIndices.push_back(idx-1);
        }
        --idx;
    }

    for (const size_t idx : removalIndices) {
        candidates.erase(candidates.begin() + idx);
    }

    return candidates;
}
#endif

void Localizer::loadSettings(settings::localizer_settings_t &&settings)
{
    _settings = std::move(settings);
}

void Localizer::loadSettings(settings::localizer_settings_t const& settings)
{
    _settings = settings;
}

settings::localizer_settings_t Localizer::getSettings() const
{
    return _settings;
}
}
