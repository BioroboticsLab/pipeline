/*
 * Localizer.cpp
 *
 *  Created on: 03.07.2014
 *      Author: mareikeziese
 */

#include "Localizer.h"

#include <opencv2/highgui/highgui.hpp>

/**
 * Scales a given OpenCV rectangle by a factor, conserving the rectangle's center.
 *
 * \param rectangle OpenCV rectangle to be scaled
 * \param scale factor by which the rectangle is scaled
 */
cv::Rect operator*(const cv::Rect rectangle, double scale) {
    cv::Size    s((rectangle.height * scale), (rectangle.width * scale));
    cv::Point2i c(rectangle.x - (0.5 * (s.width - rectangle.width)),
        rectangle.y - (0.5 * (s.height - rectangle.height)));
    return (cv::Rect(c, s));
}

namespace decoder {
/**************************************
*
*           constructor
*
**************************************/

Localizer::Localizer() {
#ifdef PipelineStandalone
	loadConfigVars(config::DEFAULT_LOCALIZER_CONFIG);
#endif
}

#ifdef PipelineStandalone
Localizer::Localizer(const std::string &configFile) {
	loadConfigVars(configFile);
}
#endif

Localizer::~Localizer() {}

/**************************************
*
*           getter/setter
*
**************************************/

const cv::Mat& Localizer::getBlob() const {
    return blob_;
}

void Localizer::setBlob(const cv::Mat& blob) {
    blob_ = blob;
}

const cv::Mat& Localizer::getCannyMap() const {
    return canny_map_;
}

void Localizer::setCannyMap(const cv::Mat& cannyMap) {
    canny_map_ = cannyMap;
}

const cv::Mat& Localizer::getGrayImage() const {
    return gray_image_;
}

void Localizer::setGrayImage(const cv::Mat& grayImage) {
    gray_image_ = grayImage;
}

const cv::Mat& Localizer::getSobel() const {
    return sobel_;
}

void Localizer::setSobel(const cv::Mat& sobel) {
    sobel_ = sobel;
}

/**************************************
*
*           stuff
*
**************************************/

std::vector<Tag> Localizer::process(cv::Mat&& grayImage) {
    this->gray_image_ = grayImage;

    // compute the sobel derivative first
    this->sobel_ = this->computeSobelMap(grayImage);

    // and then locate the tags using the sobel map
    this->blob_ = this->computeBlobs(this->sobel_);

    // compute canny edge map. Needed for ellipse detection but needs to be done only once per image.
    //this->canny_map_ = this->computeCannyEdgeMap(grayImage);

    std::vector<Tag> taglist = this->locateTagCandidates(this->blob_,
        this->canny_map_, this->gray_image_);

    return taglist;
}

/**
 * Highlight tag candidates in a comb image by intensity values
 *
 * @param grayImage
 * @return image with highlighted tags
 */
cv::Mat Localizer::highlightTags(cv::Mat &grayImage) {

    //binarization
	cv::Mat binarizedImage;
    cv::threshold(grayImage, binarizedImage, this->_settings.binary_threshold, 255,
      CV_THRESH_BINARY);

    cv::Mat imageCopy = binarizedImage.clone();

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("binarized Image", cv::WINDOW_NORMAL);
        cv::imshow("binarized Image", imageCopy);
        cv::waitKey(0);
        cv::destroyWindow("binarized Image");
    }
#endif

    cv::Mat imageCopy2 = binarizedImage.clone();

    //cv::MORPH_OPEN
    cv::Mat dilatedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * this->_settings.dilation_1_size + 1,
        2 * this->_settings.dilation_1_size + 1),
        cv::Point(this->_settings.dilation_1_size,
        this->_settings.dilation_1_size));
    cv::dilate(imageCopy, imageCopy, dilatedImage, cv::Point(-1, -1),
	  this->_settings.dilation_1_iteration_number);

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
		cv::namedWindow("First Dilate", cv::WINDOW_NORMAL);
		cv::imshow("First Dilate", imageCopy);
		cv::waitKey(0);
		cv::destroyWindow("First Dilate");
    }
#endif

    //erosion
    cv::Mat erodedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * this->_settings.erosion_size + 1,
        2 * this->_settings.erosion_size + 1),
        cv::Point(this->_settings.erosion_size,
        this->_settings.erosion_size));
    cv::erode(imageCopy, imageCopy, erodedImage);

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("First Erode", cv::WINDOW_NORMAL);
        cv::imshow("First Erode", imageCopy);
        cv::waitKey(0);
        cv::destroyWindow("First Erode");
    }
#endif

    dilatedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * this->_settings.dilation_2_size + 1,
        2 * this->_settings.dilation_2_size + 1),
        cv::Point(this->_settings.dilation_2_size,
        this->_settings.dilation_2_size));
    cv::dilate(imageCopy, imageCopy, dilatedImage);

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("My Window", cv::WINDOW_NORMAL);
        cv::imshow("My Window", imageCopy);
        cv::waitKey(0);
        cv::destroyWindow("My Window");
    }
#endif
    return imageCopy;
}

/**
 *  Find blobs in the binary input image Ib and filter them by their size
 *
 * @param blobImage binary comb image with highlighted tag candidates
 * @return boundingBoxes output vector of size-filtered bounding boxes
 */

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
        if (contour.size() < this->_settings.max_tag_size) {
            cv::Rect rec = cv::boundingRect(contour) * 2;

            if (rec.width < this->_settings.min_tag_size) {
                const int offset = abs(rec.width - this->_settings.min_tag_size);
                rec.x     = rec.x - offset / 2;
                rec.width = rec.width + offset;
            }

            if (rec.height < this->_settings.min_tag_size) {
                const int offset = abs(rec.height - this->_settings.min_tag_size);
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
            if ((rec.height * rec.width) > 800
              && (rec.height * rec.width) < 20000) {
                Tag tag(rec, taglist.size() + 1);

                cv::Mat sub_image_orig(grayImage, rec);
                cv::Mat subImageOrig_cp = sub_image_orig.clone();
                tag.setOrigSubImage(subImageOrig_cp);

                taglist.push_back(tag);
            }
        }
    }

    return taglist;
}

/**
 * Computes the Sobel map for a given grayscale image.
 * @return sobelmap
 */
cv::Mat Localizer::computeSobelMap(cv::Mat grayImage) {

    // We need a copy because the GuassianBlur makes changes to the image
    cv::Mat imageCopy = grayImage.clone();

    int scale  = 1;
    int delta  = 0;
    int ddepth = CV_16S;
    cv::GaussianBlur(imageCopy, imageCopy, cv::Size(7, 7), 0, 0, cv::BORDER_DEFAULT);

    /// Generate grad_x and grad_y
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
    cv::Sobel(imageCopy, grad_x, ddepth, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    cv::Sobel(imageCopy, grad_y, ddepth, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    /// Total Gradient (approximate)
    cv::Mat sobel;
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel);
#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("Sobel", cv::WINDOW_NORMAL);
        cv::imshow("Sobel", sobel);
        cv::waitKey(0);
        cv::destroyWindow("Sobel");
    }
#endif

    return sobel;

    //DEBUG_IMSHOW( "sobel", sobel );
}

/*
   Computes Blobs and finally finds the ROI's using the sobel map. The ROI's are stored in the boundingBoxes Vector.
 */

cv::Mat Localizer::computeBlobs(cv::Mat sobel) {
    cv::Mat blob = this->highlightTags(sobel);

    //DEBUG_IMSHOW("blob", blob);

    //vector<cv::Rect> boundingBoxes = this->locateTagCandidates(blob);

    //#ifdef _DEBUG
    //	image.copyTo(output);
    //
    //	for ( unsigned int i = 0; i < boundingBoxes.size(); i++) {
    //
    //		cv::rectangle(output, boundingBoxes[i], cv::Scalar(0, 0, 255), 3);
    //	}
    //#endif
    return blob;
}

#ifdef PipelineStandalone
/**
 * loads param from config
 *
 * @param filename absolute path to the config file
 */
void Localizer::loadConfigVars(const std::string &filename) {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(filename, pt);

	_settings.binary_threshold =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".binary_threshold");
	_settings.dilation_1_iteration_number =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".dilation_1_interation_number");
	_settings.dilation_1_size =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".dilation_1_size");
	_settings.erosion_size =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".erosion_size");
	_settings.dilation_2_size =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".dilation_2_size");
	_settings.max_tag_size =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".max_tag_size");
	_settings.min_tag_size =
			pt.get<int>(config::APPlICATION_ENVIROMENT + ".min_tag_size");
}
#endif

void Localizer::loadSettings(localizer_settings_t &&settings)
{
	_settings = std::move(settings);
}
}
