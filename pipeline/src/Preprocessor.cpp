#include "../Preprocessor.h"

namespace pipeline {

PreprocessorResult Preprocessor::process(const std::string& filename) {
    cv::Mat image = cv::imread(filename);

    return process(image);
}

PreprocessorResult Preprocessor::process(const cv::Mat &image) {
    cv::Mat grayImage;

    if (image.type() == CV_8UC1) {
        image.copyTo(grayImage);
    } else {
        cv::cvtColor(image, grayImage, CV_BGR2GRAY);
    }

    PreprocessorResult result;

    result.originalImage = grayImage;
    result.preprocessedImage = pipelinePreprocess(grayImage);
    result.claheImage = dnnPreprocess(grayImage);

    return result;
}

settings::preprocessor_settings_t Preprocessor::getSettings() const {
    return _settings;
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

cv::Mat Preprocessor::pipelinePreprocess(const cv::Mat &grayImage)
{
    cv::Mat contrastStretched = grayImage.clone();
    if (_settings.get_opt_use_contrast_streching()) {
        contrastStretching(contrastStretched);
    }

    cv::Mat honeyFiltered = contrastStretched.clone();
    if (_settings.get_honey_enabled()) {
        filterHoney(honeyFiltered, grayImage);
    }

    cv::Mat contrastEqualized = honeyFiltered.clone();
    if (_settings.get_opt_use_equalize_histogram()) {
        equalizeHistogram(contrastEqualized);
    }

    cv::Mat sobelImage = contrastEqualized.clone();
    computeSobel(sobelImage);

    cv::Mat combFiltered = sobelImage.clone();
    if (_settings.get_comb_enabled()) {
        filterCombs(combFiltered);
    }

    return combFiltered;
}

void Preprocessor::contrastStretching(cv::Mat &image)
{
    const std::vector<ImageRaster> rasters = getRaster(image, _settings.get_opt_frame_size());

    for (ImageRaster const& raster : rasters) {
        cv::Mat block = image.rowRange(raster.rowRange).colRange(raster.colRange);
        const cv::Scalar meanSobel = cv::mean(block);
        const double averageValue = meanSobel.val[0];
        if (averageValue < _settings.get_opt_average_contrast_value()) {
            cv::normalize(block, block, 0, 255, CV_MINMAX);
        }
    }
}

void Preprocessor::filterHoney(cv::Mat &contrastStretchedImage, cv::Mat const& originalImage)
{
    const std::vector<ImageRaster> rasters = getRaster(contrastStretchedImage, _settings.get_opt_frame_size());

    for (ImageRaster const& raster : rasters) {
        const cv::Mat origBlock = originalImage.rowRange(raster.rowRange).colRange(raster.colRange);

        cv::Scalar mean, std;
        cv::meanStdDev(origBlock, mean, std);

        if (std[0] < _settings.get_honey_std_dev() &&
            mean[0] > _settings.get_honey_average_value())
        {
            cv::Mat optBlock = contrastStretchedImage.rowRange(raster.rowRange).colRange(raster.colRange);
            optBlock.setTo(mean);
        }
    }
}

void Preprocessor::equalizeHistogram(cv::Mat &image)
{
    cv::equalizeHist(image, image);
}

void Preprocessor::filterCombs(cv::Mat &sobel)
{
    // calculate binarized image for comb-detection
    cv::Mat thresholdImage;
    cv::threshold(sobel, thresholdImage, _settings.get_comb_threshold(), 255, cv::THRESH_BINARY);

#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
    cv::namedWindow("binarized Image with combs", cv::WINDOW_NORMAL);
    cv::imshow("binarized Image with combs", threshold_image);
    cv::waitKey(0);

#endif
#endif

    // find contours
    cv::Mat thresholdImageContours = thresholdImage.clone();
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresholdImageContours, contours, hierarchy,
                     CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // find the rotated rectangles and ellipses for each contour
    std::vector<cv::RotatedRect> minRect(contours.size());
    std::vector<cv::RotatedRect> minEllipse(contours.size());

    for (size_t i = 0; i < contours.size(); i++) {
        minRect[i] = cv::minAreaRect(cv::Mat(contours[i]));

        // check the size of the contours
        if (minRect[i].size.area() > 100 && contours[i].size() > 5) {
            minEllipse[i] = fitEllipse(cv::Mat(contours[i]));
        }
    }

    // draw ellipses of the contours
    const cv::Scalar ellColor(_settings.get_comb_line_color(),
                              _settings.get_comb_line_color(),
                              _settings.get_comb_line_color());

    cv::Mat drawing;
    for (size_t i = 0; i < contours.size(); i++) {
        const cv::RotatedRect ell = minEllipse[i];

        //check for the right features to be a comb
        if ((ell.size.height > _settings.get_comb_min_size() ||
             ell.size.width > _settings.get_comb_min_size()) &&
            (std::abs(ell.size.height - ell.size.width) < _settings.get_comb_diff_size()))
        {
            // draw the ellipse into the sobel image
            cv::ellipse(sobel, ell, ellColor, _settings.get_comb_line_width());
        }
    }

#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
        cv::namedWindow("Image without combs", cv::WINDOW_NORMAL);
        cv::imshow("Image without combs", sobel);
        cv::waitKey(0);

#endif
#endif
}

void Preprocessor::computeSobel(cv::Mat &image)
{
    static const int scale = 1;
    static const int delta = 0;
    static const int ddepth = CV_16S;

    cv::GaussianBlur(image, image, cv::Size(7, 7), 0, 0, cv::BORDER_DEFAULT);

    cv::Mat gradX;
    cv::Sobel(image, gradX, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(gradX, gradX);

    cv::Mat gradY;
    cv::Sobel(image, gradY, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(gradY, gradY);

    cv::addWeighted(gradX, 0.5, gradY, 0.5, 0, image);

#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
    cv::namedWindow("Sobel", cv::WINDOW_AUTOSIZE);
    cv::imshow("Sobel", image);
    cv::waitKey(0);
    cv::destroyWindow("Sobel");
#endif
#endif
}

std::vector<Preprocessor::ImageRaster> Preprocessor::getRaster(const cv::Mat &image, const unsigned int frameSize)
{
    std::vector<ImageRaster> raster;

    const div_t divRows = div(image.rows, frameSize);
    const div_t divCols = div(image.cols, frameSize);

    const int rowIterations = (divRows.rem > 0 ? divRows.quot + 1 : divRows.quot);
    const int colIterations = (divCols.rem > 0 ? divCols.quot + 1 : divCols.quot);

    for (int i = 0; i < rowIterations; i++) {
        for (int j = 0; j < colIterations; j++) {
            const cv::Range rows = ((i == rowIterations - 1) ?
                    cv::Range(frameSize * i, image.rows) :
                    cv::Range(frameSize * i, frameSize * (i + 1)));
            const cv::Range cols = ((j == colIterations - 1) ?
                    cv::Range(frameSize * j, image.cols) :
                    cv::Range(frameSize * j, frameSize * (j + 1)));
            raster.push_back(ImageRaster { rows, cols });
        }
    }

    return raster;
}

} /* namespace pipeline */

