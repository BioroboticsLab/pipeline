/*
 * Localizer.cpp
 *
 *  Created on: 03.07.2014
 *      Author: mareikeziese
 */

#include "Localizer.h"

#include <opencv2/highgui/highgui.hpp>

#include "datastructure/BoundingBox.h"
#include "datastructure/Tag.h"

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


const cv::Mat& Localizer::getThresholdImage() const {
	return _threshold_image;
}

void Localizer::setThresholdImage(const cv::Mat& thresholdImage) {
	_threshold_image = thresholdImage;
}

/**************************************
*
*           stuff
*
**************************************/

std::vector<Tag> Localizer::process(cv::Mat &&originalImage, cv::Mat &&preprocessedImage){



    // and then locate the tags using the sobel map
    this->blob_ = this->computeBlobs(preprocessedImage);

    // compute canny edge map. Needed for ellipse detection but needs to be done only once per image.
    //this->canny_map_ = this->computeCannyEdgeMap(grayImage);

    std::vector<Tag> taglist= this->locateTagCandidates(this->blob_, this->canny_map_,originalImage);

    return taglist;
}

/**
 * Highlight tag candidates in a comb image by intensity values
 *
 * @param grayImage the input image
 * @return image with highlighted tags
 */
cv::Mat Localizer::highlightTags(const cv::Mat &grayImage)  {

	cv::Mat image;
	grayImage.copyTo(image);
    //binarization
	cv::Mat binarizedImage;


	// Threshold each block (3x3 grid) of the image separately to
	// correct for minor differences in contrast across the image.
	for (int i = 0; i < 30; i++) {
	    for (int j = 0; j < 40; j++) {
	        cv::Mat block = image.rowRange(100*i, 100*(i+1)).colRange(100*j, 100*(j+1));
	        cv::Scalar mean_sobel = mean(block);
	        double average_value = mean_sobel.val[0];
	        cv::threshold(block, block, average_value+ this->_settings.get_binary_threshold(), 255, cv::THRESH_BINARY);
	    }
	}

    cv::Mat imageCopy; //= binarizedImage.clone();
    this->setThresholdImage(image);
    image.copyTo(imageCopy);

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("binarized Image", cv::WINDOW_AUTOSIZE);
        cv::imshow("binarized Image", imageCopy);
        cv::waitKey(0);
        cv::destroyWindow("binarized Image");
    }
#endif

    cv::Mat imageCopy2 = binarizedImage.clone();

    //cv::MORPH_OPEN
    cv::Mat dilatedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * this->_settings.get_first_dilation_size() + 1,
        2 * this->_settings.get_first_dilation_size() + 1),
        cv::Point(this->_settings.get_first_dilation_size(),
        this->_settings.get_first_dilation_size()));
    cv::dilate(imageCopy, imageCopy, dilatedImage, cv::Point(-1, -1),
	  this->_settings.get_first_dilation_num_iterations());

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
		cv::namedWindow("First Dilate", cv::WINDOW_AUTOSIZE);
		cv::imshow("First Dilate", imageCopy);
		cv::waitKey(0);
		cv::destroyWindow("First Dilate");
    }
#endif

    //erosion
    cv::Mat erodedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * this->_settings.get_erosion_size() + 1,
        2 * this->_settings.get_erosion_size() + 1),
        cv::Point(this->_settings.get_erosion_size(),
        this->_settings.get_erosion_size()));
    cv::erode(imageCopy, imageCopy, erodedImage);

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("First Erode", cv::WINDOW_AUTOSIZE);
        cv::imshow("First Erode", imageCopy);
        cv::waitKey(0);
        cv::destroyWindow("First Erode");
    }
#endif

    dilatedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2 * this->_settings.get_second_dilation_size() + 1,
        2 * this->_settings.get_second_dilation_size() + 1),
        cv::Point(this->_settings.get_second_dilation_size(),
        this->_settings.get_second_dilation_size()));
    cv::dilate(imageCopy, imageCopy, dilatedImage);

#ifdef PipelineStandalone
    if (config::DEBUG_MODE_LOCALIZER) {
        cv::namedWindow("My Window", cv::WINDOW_AUTOSIZE);
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
 * @param blobImage_old binary comb image with highlighted tag candidates
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
        if (contour.size() < this->_settings.get_max_tag_size()) {
            cv::Rect rec = cv::boundingRect(contour) * 2;

            if (rec.width < this->_settings.get_min_bounding_box_size()) {
                const int offset = abs(rec.width - this->_settings.get_min_bounding_box_size());
                rec.x     = rec.x - offset / 2;
                rec.width = rec.width + offset;
            }

            if (rec.height < this->_settings.get_min_bounding_box_size()) {
                const int offset = abs(rec.height - this->_settings.get_min_bounding_box_size());
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

    return taglist;
}


/*
   Computes Blobs and finally finds the ROI's using the sobel map. The ROI's are stored in the boundingBoxes Vector.
 */

cv::Mat Localizer::computeBlobs(const cv::Mat &sobel)  {
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

void Localizer::loadSettings(settings::localizer_settings_t &&settings)
{
	_settings = std::move(settings);
}
}


