/*
 * Preprocessor.cpp
 *
 *  Created on: 15.01.2015
 *      Author: mareikeziese
 */

#include "Preprocessor.h"

namespace pipeline {
/**************************************
 *
 *           getter/setter
 *
 **************************************/
settings::preprocessor_settings_t Preprocessor::getOptions() const {
	return _options;
}

void Preprocessor::setOptions(settings::preprocessor_settings_t options) {
	_options = options;
}

const cv::Mat& Preprocessor::getThresholdImage() const {
	return _thresholdImage;
}

const cv::Mat& Preprocessor::getSobel() const {
	return _sobel;
}

const cv::Mat& Preprocessor::getHoneyImage() const {
	return _honeyImage;
}

void Preprocessor::setHoneyImage(const cv::Mat& honeyImage) {
	_honeyImage = honeyImage;
}

const cv::Mat& Preprocessor::getOptsImage() const {
	return _optsImage;
}

void Preprocessor::setOptsImage(const cv::Mat& optsImage) {
	_optsImage = optsImage;
}

void Preprocessor::setSobel(const cv::Mat& sobel) {
	_sobel = sobel;
}

void Preprocessor::setThresholdImage(const cv::Mat& thresholdImage) {
	_thresholdImage = thresholdImage;
}

/**************************************
 *
 *           constructor
 *
 **************************************/

Preprocessor::Preprocessor() {
	this->_options = settings::preprocessor_settings_t();
}

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

	// convert image to grayscale (not needed later because images will already be grayscale)
	cv::cvtColor(image, grayImage, CV_BGR2GRAY);

	cv::Mat opt_image;
	if (this->_options.get_opt_use_contrast_streching()) {
		opt_image = this->_contrastStretching(grayImage);
	} else {
		opt_image = grayImage.clone();
	}
	setOptsImage(opt_image);

	if (this->_options.get_honey_enabled()) {
		grayImage = this->_filterHoney(opt_image, grayImage);
	}

	setHoneyImage(grayImage.clone());

	if (this->_options.get_opt_use_equalize_histogram()) {
		this->_equalizeHistogramm(grayImage);
	}

	_sobel = this->_computeSobel(grayImage);

	if (this->_options.get_comb_enabled()) {
		this->_filterCombs(_sobel);
	}

	return _sobel;
}

void Preprocessor::_equalizeHistogramm(cv::Mat& image) {
	cv::equalizeHist(image, image);
}

cv::Mat Preprocessor::_contrastStretching(const cv::Mat& image) {

	cv::Mat opt = image.clone();
	int frame_size = static_cast<int>(this->_options.get_opt_frame_size());
	std::vector<image_raster> rasters = this->_getRaster(opt, frame_size);

	for (image_raster &raster : rasters) {

		cv::Mat block = opt.rowRange(raster.row_range).colRange(
				raster.col_range);
		cv::Scalar mean_sobel = mean(block);
		double average_value = mean_sobel.val[0];
		if (average_value < this->_options.get_opt_average_contrast_value()) {
			cv::normalize(block, block, 0, 255, CV_MINMAX);
		}

	}
	return opt;

}

/**
 * @TODO write other setting loaders in this form
 *
 */

void Preprocessor::loadSettings(settings::preprocessor_settings_t &&settings) {
	_options = std::move(settings);
}

/**
 * Computes the Sobel map for a given grayscale image.
 * @return sobelmap
 */
cv::Mat Preprocessor::_computeSobel(const cv::Mat &grayImage) const {

	cv::Mat imageCopy = grayImage.clone();

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	cv::GaussianBlur(imageCopy, imageCopy, cv::Size(7, 7), 0, 0,
			cv::BORDER_DEFAULT);

	/// Generate grad_x and grad_y
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	cv::Sobel(imageCopy, grad_x, ddepth, 1, 0, 3, scale, delta,
			cv::BORDER_DEFAULT);
	cv::convertScaleAbs(grad_x, abs_grad_x);

	/// Gradient Y
	//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	cv::Sobel(imageCopy, grad_y, ddepth, 0, 1, 3, scale, delta,
			cv::BORDER_DEFAULT);
	cv::convertScaleAbs(grad_y, abs_grad_y);

	/// Total Gradient (approximate)
	cv::Mat sobel;
	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel);
#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
	cv::namedWindow("Sobel", cv::WINDOW_AUTOSIZE);
	cv::imshow("Sobel", sobel);
	cv::waitKey(0);
	cv::destroyWindow("Sobel");
#endif
#endif

	return sobel;

}

cv::Mat Preprocessor::_filterHoney(const cv::Mat& opt_image,
		const cv::Mat& orig_image) {

	cv::Mat image = opt_image.clone();
	int frame_size = static_cast<int>(this->_options.get_honey_frame_size());
	std::vector<image_raster> rasters = this->_getRaster(image, frame_size);
	cv::Mat opt_block, orig_block;

	for (image_raster &raster : rasters) {

		orig_block = orig_image.rowRange(raster.row_range).colRange(
				raster.col_range);
		cv::Scalar mean, std_dev;

		cv::meanStdDev(orig_block, mean, std_dev);
		if (std_dev[0] < this->_options.get_honey_std_dev()
				&& mean[0] > this->_options.get_honey_average_value()) {
			opt_block = image.rowRange(raster.row_range).colRange(
					raster.col_range);
			opt_block.setTo(mean);
		}
	}
	return image;

}

std::vector<image_raster> Preprocessor::_getRaster(cv::Mat image,
		int frame_size) {
	std::vector<image_raster> raster = std::vector<image_raster>();

	div_t div_rows, div_cols;
	div_rows = div(image.rows, frame_size);
	div_cols = div(image.cols, frame_size);

	int row_iterations = (div_rows.rem > 0 ? div_rows.quot + 1 : div_rows.quot);
	int col_iterations = (div_cols.rem > 0 ? div_cols.quot + 1 : div_cols.quot);
	cv::Range rows, cols;

	for (int i = 0; i < row_iterations; i++) {
		for (int j = 0; j < col_iterations; j++) {

			rows = ((i == row_iterations - 1) ?
					cv::Range(frame_size * i, image.rows) :
					cv::Range(frame_size * i, frame_size * (i + 1)));
			cols = ((j == col_iterations - 1) ?
					cv::Range(frame_size * j, image.cols) :
					cv::Range(frame_size * j, frame_size * (j + 1)));
			raster.push_back(image_raster { rows, cols });
		}
	}
	return raster;
}

void Preprocessor::_filterCombs(cv::Mat& sobel) {
	cv::Mat threshold_image, img_copy, threshold_image_contours;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
	std::cout << "combs will be removed, calculate binarized image"<< std::endl;
#endif
#endif

	/// calculate binarized image for comb-detection
	cv::threshold(sobel, threshold_image, this->_options.get_comb_threshold(),
			255, cv::THRESH_BINARY);

#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
	cv::namedWindow("binarized Image with combs", cv::WINDOW_NORMAL);
	cv::imshow("binarized Image with combs", threshold_image);
	cv::waitKey(0);

#endif
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

	for (size_t i = 0; i < contours.size(); i++) {

		cv::RotatedRect ell = minEllipse[i];

		///check for the right features to be a comb
		if ((ell.size.height > this->_options.get_comb_min_size()
				|| ell.size.width > this->_options.get_comb_min_size())
				&& (std::abs(ell.size.height - ell.size.width)
						< this->_options.get_comb_diff_size())) {

			/// draw the ellipse into the sobel image
			//if(ell.center.x + ell.size.width <= image.cols && ell.center.y + ell.size.height <= image.rows){
			cv::ellipse(sobel, ell,
					cv::Scalar(this->_options.get_comb_line_color(),
							this->_options.get_comb_line_color(),
							this->_options.get_comb_line_color()),
					this->_options.get_comb_line_width());
			//}

		}
		/*else if (ell.size.height >= this->_options.min_size_comb &&  ell.size.width
		 >= this->_options.make min_size_comb && cv::arcLength(contours[i],true) <= maxLen ) {
		 cv::drawContours(sobel, contours, static_cast<int>(i), cv::Scalar(this->_options.line_color_combs, this->_options.line_color_combs, this->_options.line_color_combs),
		 this->_options.line_width_combs, 8, std::vector<cv::Vec4i>(),
		 0, cv::Point());

		 //cv::ellipse(sobel,ell, cv::Scalar(this->_options.line_color_combs, this->_options.line_color_combs, this->_options.line_color_combs), this->_options.line_width_combs);
		 }*/

		//}
#ifdef PipelineStandalone
#ifdef DEBUG_PREPROCESSOR
		cv::namedWindow("Image without combs", cv::WINDOW_NORMAL);
		cv::imshow("Image without combs", sobel);
		cv::waitKey(0);

#endif
#endif

	}

}

} /* namespace decoder */

