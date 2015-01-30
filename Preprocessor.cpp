/*
 * Preprocessor.cpp
 *
 *  Created on: 15.01.2015
 *      Author: mareikeziese
 */

#include <tracking/algorithm/BeesBook/BeesBookImgAnalysisTracker/pipeline/Preprocessor.h>

namespace pipeline {
/**************************************
 *
 *           getter/setter
 *
 **************************************/
preprocessor_settings_t Preprocessor::getOptions() const {
	return _options;
}

void Preprocessor::setOptions(preprocessor_settings_t options) {
	_options = options;
}

/**************************************
 *
 *           constructor
 *
 **************************************/

Preprocessor::Preprocessor() {
	this->_options = preprocessor_settings_t();

#ifdef PipelineStandalone

	this->_loadConfig();
#endif
}

#ifdef PipelineStandalone
virtual IPreprocessor(const std::string &configFile) {
	this->_options = PreprocessorOptions();
	this->_options->loadFromIni(configFile, config::APPLICATION_ENV);
}

virtual IPreprocessor(const std::string &configFile, std:.string enviroment) {
	this->_options = PreprocessorOptions();
	this->_options->loadFromIni(configFile, enviroment);
}
#endif

Preprocessor::~Preprocessor() {
}

/**************************************
 *
 *           stuff
 *
 **************************************/

cv::Mat Preprocessor::process(const std::string& filename) {
	cv::Mat image = cv::imread(filename);

	return process(image);
}

cv::Mat Preprocessor::process(const cv::Mat &image) {
	cv::Mat grayImage;

	setThresholdImage(image);

	// convert image to grayscale (not needed later because images will already be grayscale)
	cv::cvtColor(image, grayImage, CV_BGR2GRAY);

	if (this->_options.use_equalize_histogram == 1) {
		for (int i = 0; i < 15; i++) {
			for (int j = 0; j < 20; j++) {
				cv::Mat block =
						grayImage.rowRange(200 * i, 200 * (i + 1)).colRange(
								200 * j, 200 * (j + 1));
				cv::Scalar mean_sobel = mean(block);
				double average_value = mean_sobel.val[0];
				if (average_value < 120) {
					cv::normalize(block, block, 0, 255, CV_MINMAX );
				}
			}
		}
	}

	_sobel = this->computeSobel(grayImage);

	if (this->_options.use_comb_detection == 1) {
		this->filterCombs(_sobel);
	}

	return _sobel;
}

const cv::Mat& Preprocessor::getThresholdImage() const {
	return _thresholdImage;
}

const cv::Mat& Preprocessor::getSobel() const {
	return _sobel;
}

void Preprocessor::setSobel(const cv::Mat& sobel) {
	_sobel = sobel;
}

void Preprocessor::setThresholdImage(const cv::Mat& thresholdImage) {
	_thresholdImage = thresholdImage;
}

/**
 * @TODO write other setting loaders in this form
 *
 */

void Preprocessor::loadSettings(preprocessor_settings_t &&settings) {
	_options = std::move(settings);
}

/**
 * Computes the Sobel map for a given grayscale image.
 * @return sobelmap
 */
cv::Mat Preprocessor::computeSobel(const cv::Mat &grayImage) const {

	// We need a copy because the GuassianBlur makes changes to the image
	cv::Mat imageCopy = grayImage.clone();
   /* cv::GaussianBlur(imageCopy, imageCopy, cv::Size(3, 3), 0, 0,
      cv::BORDER_DEFAULT);
	cv::Canny(imageCopy, sobel,  this->_options.comb_threshold/2, this->_options.comb_threshold);
	cv::threshold(sobel,sobel,1, 255, cv::THRESH_BINARY_INV);

	cv::Mat erodedImage = cv::getStructuringElement(cv::MORPH_ELLIPSE,
	        cv::Size(2*this->_options.max_size_comb+1,2*this->_options.max_size_comb+1),
	        cv::Point(2*this->_options.max_size_comb,2*this->_options.max_size_comb));
	    cv::erode(sobel, sobel, erodedImage);*/



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
		cv::namedWindow("Sobel", cv::WINDOW_AUTOSIZE);
		cv::imshow("Sobel", sobel);
		cv::waitKey(0);
		cv::destroyWindow("Sobel");
	}
#endif

	return sobel;

	//DEBUG_IMSHOW( "sobel", sobel );
}

void Preprocessor::filterCombs(cv::Mat& sobel) {
	cv::Mat threshold_image, img_copy, threshold_image_contours;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
#ifdef PipelineStandalone
	if (config::DEBUG_MODE_LOCALIZER) {
		std::cout << "combs will be removed, calculate binarized image"<< std::endl;
	}
#endif

	/// calculate binarized image for comb-detection
	cv::threshold(sobel, threshold_image, this->_options.comb_threshold, 255,
			cv::THRESH_BINARY);

#ifdef PipelineStandalone
	if (config::DEBUG_MODE_LOCALIZER_IMAGE) {
		cv::namedWindow("binarized Image with combs", WINDOW_NORMAL);
		cv::imshow("binarized Image with combs", threshold_image);
		cv::waitKey(0);

	}
#endif

	setThresholdImage(threshold_image);
	threshold_image.copyTo(threshold_image_contours);
	/// Find contours
	cv::findContours(threshold_image_contours, contours, hierarchy,
			CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	/// Find the rotated rectangles and ellipses for each contour
	std::vector<cv::RotatedRect> minRect(contours.size());
	std::vector<cv::RotatedRect> minEllipse(contours.size());

	for (size_t i = 0; i < contours.size(); i++) {
		minRect[i] = minAreaRect(cv::Mat(contours[i]));
		//check the size of the contours
		if (minRect[i].size.area() > 100 && contours[i].size() > 5) {
			minEllipse[i] = fitEllipse(cv::Mat(contours[i]));
		} else {
			cv::Vec<int, 4> entry = hierarchy[i];
		}
	}

	/// Draw  ellipses of the contours
	cv::Mat drawing;

	double maxLen = 2 * M_PI * this->_options.max_size_comb;

	for (size_t i = 0; i < contours.size(); i++) {

		cv::RotatedRect ell = minEllipse[i];

		///check for the right features to be a comb
		if ((ell.size.height > this->_options.min_size_comb
				|| ell.size.width > this->_options.min_size_comb)
				&& (std::abs(ell.size.height - ell.size.width)
						< this->_options.diff_size_combs)) {

			/// draw the ellipscd so	e of the possible comb in the sobel image
			//if(ell.center.x + ell.size.width <= image.cols && ell.center.y + ell.size.height <= image.rows){
			cv::ellipse(sobel, ell,
					cv::Scalar(this->_options.line_color_combs,
							this->_options.line_color_combs,
							this->_options.line_color_combs),
					this->_options.line_width_combs);
			//}
		} /*else if (ell.size.height >= this->_options.min_size_comb &&  ell.size.width
		 >= this->_options.min_size_comb && cv::arcLength(contours[i],true) <= maxLen ) {
		 cv::drawContours(sobel, contours, static_cast<int>(i), cv::Scalar(this->_options.line_color_combs, this->_options.line_color_combs, this->_options.line_color_combs),
		 this->_options.line_width_combs, 8, std::vector<cv::Vec4i>(),
		 0, cv::Point());

		 //cv::ellipse(sobel,ell, cv::Scalar(this->_options.line_color_combs, this->_options.line_color_combs, this->_options.line_color_combs), this->_options.line_width_combs);
		 }*/
	}

#ifdef PipelineStandalone
	if (config::DEBUG_MODE_LOCALIZER_IMAGE) {
		cv::namedWindow("Image without combs", WINDOW_NORMAL);
		cv::imshow("Image without combs", sobel);
		cv::waitKey(0);

	}
#endif

}

} /* namespace decoder */

