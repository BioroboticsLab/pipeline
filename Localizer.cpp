/*
 * Localizer.cpp
 *
 *  Created on: 03.07.2014
 *      Author: mareikeziese
 */

#include "Localizer.h"

/**
 * Scales a given OpenCV rectangle by a factor, conserving the rectangle's center.
 *
 * \param rectangle OpenCV rectangle to be scaled
 * \param scale factor by which the rectangle is scaled
 */
Rect operator*(const Rect rectangle, double scale) {
	Size s = Size((rectangle.height * scale), (rectangle.width * scale));
	Point2i c = Point(rectangle.x - (0.5 * (s.width - rectangle.width)),
			rectangle.y - (0.5 * (s.height - rectangle.height)));
	return (Rect(c, s));
}

namespace decoder {

/**************************************
 *
 * 			constructor
 *
 **************************************/

Localizer::Localizer() {
	this->loadConfigVars(
			config::APPLICATION_PATH + config::DEFAULT_LOCALIZER_CONFIG);

}

Localizer::Localizer(string configFile) {
	this->loadConfigVars(configFile);

}

Localizer::Localizer(LocalizerOptions options) {
	this->_options = options;

}

Localizer::~Localizer() {

	// TODO Auto-generated destructor stub
}

/**************************************
 *
 * 			getter/setter
 *
 **************************************/

const LocalizerOptions& Localizer::getOptions() const {
	return _options;
}

void Localizer::setOptions(const LocalizerOptions& options) {
	_options = options;
}

/**************************************
 *
 * 			stuff
 *
 **************************************/

vector<Tag> Localizer::process(cv::Mat grayImage) {
	Mat thresholdMat, resizedImage, sobel, blob;

	resize(grayImage, resizedImage,
			Size(round(this->_options.resizeRatio * grayImage.cols),
					round(this->_options.resizeRatio * grayImage.rows)));

	sobel = this->_computeSobel(resizedImage);

	///remove combs from the sobel-image
	this->_removeCombs(sobel);

	///calculate binarized image for morphological operations
	blob = this->_generateThresholdImage(sobel, this->_options.initThres2,
			this->_options.minMean2, this->_options.maxMean2);

	if (config::DEBUG_MODE_LOCALIZER_IMAGE) {

		cv::namedWindow("threshold without combs", WINDOW_NORMAL);
		cv::imshow("threshold without combs", blob);
		cv::waitKey(0);

	}

	///apply morphological operations
	this->_filterNoise(blob);

	///find candidates
	vector<Tag> taglist = this->_locateTagCandidates(blob, grayImage);
	cv::destroyAllWindows();
	return taglist;

}

void Localizer::_filterNoise(Mat &image) {

	///optional apply median blur
	if (this->_options.blurSize > 0) {
		medianBlur(image, image, this->_options.blurSize);
		if (config::DEBUG_MODE_LOCALIZER) {
			std::cout << "apply median blur" << std::endl;
		}
		if (config::DEBUG_MODE_LOCALIZER_IMAGE) {

			cv::namedWindow("blur", WINDOW_NORMAL);
			cv::imshow("blur", image);
			cv::waitKey(0);

		}
	}

	///optional dilate the first time
	if (this->_options.dilateSize1 > 0) {

		Mat dilate1 = getStructuringElement(MORPH_ELLIPSE,
				Size(2 * this->_options.dilateSize1 + 1,
						2 * (this->_options.dilateSize1 - 1) + 1),
				Point(this->_options.dilateSize1,
						(this->_options.dilateSize1 - 1)));
		dilate(image, image, dilate1, Point(-1, -1), 1);
		if (config::DEBUG_MODE_LOCALIZER) {
			std::cout << "dilate 1" << std::endl;
		}
		if (config::DEBUG_MODE_LOCALIZER_IMAGE) {

			cv::namedWindow("dilate1", WINDOW_NORMAL);
			cv::imshow("dilate1", image);
			cv::waitKey(0);

		}
	}

	///optional erode
	if (this->_options.erodeSize > 0) {

		Mat erodeMat = getStructuringElement(MORPH_ELLIPSE,
				Size(2 * this->_options.erodeSize + 1,
						2 * (this->_options.erodeSize - 1) + 1),
				Point(this->_options.erodeSize,
						(this->_options.erodeSize - 1)));
		cv::erode(image, image, erodeMat, Point(-1, -1), 1);
		if (config::DEBUG_MODE_LOCALIZER) {
			std::cout << "erode" << std::endl;
		}
		if (config::DEBUG_MODE_LOCALIZER_IMAGE) {

			cv::namedWindow("erode", WINDOW_NORMAL);
			cv::imshow("erode", image);
			cv::waitKey(0);

		}
	}

	///optional dilate the second time
	if (this->_options.dilateSize2 > 0) {

		Mat dilate2 = getStructuringElement(MORPH_ELLIPSE,
				Size(2 * this->_options.dilateSize2 + 1,
						2 * (this->_options.dilateSize2 - 1) + 1),
				Point(this->_options.dilateSize2,
						(this->_options.dilateSize2 - 1)));
		dilate(image, image, dilate2, Point(-1, -1), 1);
		if (config::DEBUG_MODE_LOCALIZER) {
			std::cout << "dilate 1" << std::endl;
		}
		if (config::DEBUG_MODE_LOCALIZER_IMAGE) {

			cv::namedWindow("dilate2", WINDOW_NORMAL);
			cv::imshow("dilate2", image);
			cv::waitKey(0);

		}
	}

}

void Localizer::_removeCombs(Mat &image) {

	cv::Mat threshold_image, threshold_image_contours;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	if (config::DEBUG_MODE_LOCALIZER) {
		std::cout << "combs will be removed, calculate binarized image";
	}

	/// calculate binarized image for comb-detection
	threshold_image = this->_generateThresholdImage(image,
			this->_options.initThres1, this->_options.minMean1,
			this->_options.maxMean1);

	if (config::DEBUG_MODE_LOCALIZER_IMAGE) {
		cv::namedWindow("binarized Image with combs", WINDOW_NORMAL);
		cv::imshow("binarized Image with combs", threshold_image);
		cv::waitKey(0);

	}

	/// Find contours
	cv::findContours(threshold_image, contours, hierarchy, CV_RETR_TREE,
			CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Find the rotated rectangles and ellipses for each contour
	vector<RotatedRect> minRect(contours.size());
	vector<RotatedRect> minEllipse(contours.size());

	for (size_t i = 0; i < contours.size(); i++) {
		minRect[i] = minAreaRect(Mat(contours[i]));
		//check the size of the contours
		if (minRect[i].size.area() > this->_options.resizeRatio * 100
				&& contours[i].size() > 5) {
			minEllipse[i] = fitEllipse(Mat(contours[i]));
		} else {
			Vec<int, 4> entry = hierarchy[i];
		}
	}
	/// Draw  ellipses of the contours
	Mat drawing;

	for (size_t i = 0; i < contours.size(); i++) {

		RotatedRect ell = minEllipse[i];

		///check for the right features to be a comb
		if ((ell.size.height
				> this->_options.minSizeComb * this->_options.resizeRatio
				|| ell.size.width
						> this->_options.minSizeComb
								* this->_options.resizeRatio)
				&& (abs(ell.size.height - ell.size.width)
						< 15 * this->_options.resizeRatio)) {

			/// draw the ellipse of the possible comb in the sobel image
			cv::ellipse(image, ell, Scalar(0, 0, 0),
					round(10 * this->_options.resizeRatio));

		} else if ((ell.size.height
				> this->_options.minSizeComb * this->_options.resizeRatio
				&& ell.size.width
						> this->_options.minSizeComb
								* this->_options.resizeRatio)) {

			cv::drawContours(image, contours, (int) i, Scalar(0, 0, 0),
					round(7 * this->_options.resizeRatio), 8, vector<Vec4i>(),
					0, Point());
		}
	}

	if (config::DEBUG_MODE_LOCALIZER_IMAGE) {
		cv::namedWindow("Image without combs", WINDOW_NORMAL);
		cv::imshow("Image without combs", image);
		cv::waitKey(0);

	}

}

/**
 * Highlight tag candidates in a comb image by intensity values
 *
 * @param grayImage
 * @return image with highlighted tags
 */

/**
 *  Find blobs in the binary input image Ib and filter them by their size
 *
 * @param blobImage binary comb image with highlighted tag candidates
 * @return boundingBoxes output vector of size-filtered bounding boxes
 */

vector<Tag> Localizer::_locateTagCandidates(Mat blobImage, Mat grayImage) {

	vector<Tag> taglist = vector<Tag>();
	vector<vector<Point2i> > contours;

	//find intra-connected white pixels
	findContours(blobImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	int i = 0;
	//extract contour bounding boxes for tag candidates
	for (vector<vector<Point2i> >::iterator contour = contours.begin();
			contour != contours.end(); ++contour) {

		//filter contours which are too big
		if (contour->size() < this->_options.maxTagSize) {

			//Rect rec_t = boundingRect(*contour) * 2;
			Rect rec_org = boundingRect(*contour) * 2;

			Rect rec(rec_org.x * (1 / this->_options.resizeRatio),
					rec_org.y * (1 / this->_options.resizeRatio),
					rec_org.width * (1 / this->_options.resizeRatio),
					rec_org.height * (1 / this->_options.resizeRatio));

			if (rec.width < this->_options.minTagSize) {
				int offset = abs(rec.width - this->_options.minTagSize);
				rec.x = rec.x - offset / 2;
				rec.width = rec.width + offset;
			}

			if (rec.height < this->_options.minTagSize) {
				int offset = abs(rec.height - this->_options.minTagSize);
				rec.y = rec.y - offset / 2;
				rec.height = rec.height + offset;
			}

			//if rectangle is outside the possible image-coordinates => resize rectangle
			if ((rec.x + rec.width) > grayImage.cols) {
				rec.x -= abs(rec.x + rec.width - grayImage.cols);
			}

			if ((rec.y + rec.height) > grayImage.rows) {
				rec.y -= abs(rec.y + rec.height - grayImage.rows);
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

				Tag tag = Tag(rec, i);

				tag.setId(taglist.size() + 1);

				Mat subImageOrig_cp;
				Mat sub_image_orig(grayImage, rec);
				sub_image_orig.copyTo(subImageOrig_cp);
				tag.setOrigSubImage(subImageOrig_cp);

				taglist.push_back(tag);
				i++;
			}
		}
	}
	return taglist;

}

/**
 * Computes the Sobel map for a given grayscale image.
 * @return sobelmap
 */
Mat Localizer::_computeSobel(Mat grayImage) {

	Mat sobel;
	Mat imageCopy;
	// We need a copy because the GuassianBlur makes changes to the image

	grayImage.copyTo(imageCopy);

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	cv::GaussianBlur(imageCopy, imageCopy, Size(7, 7), 0, 0, BORDER_DEFAULT);

	/// Generate grad_x and grad_y
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	//Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	cv::Sobel(imageCopy, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	cv::convertScaleAbs(grad_x, abs_grad_x);

	/// Gradient Y
	//Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	cv::Sobel(imageCopy, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	cv::convertScaleAbs(grad_y, abs_grad_y);

	/// Total Gradient (approximate)
	cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, sobel);
	if (config::DEBUG_MODE_LOCALIZER) {

		namedWindow("Sobel", WINDOW_NORMAL);
		imshow("Sobel", sobel);
		waitKey(0);

	}

	return sobel;
}

/**
 * loads param from config
 *
 * @param filename absolute path to the config file
 */
void Localizer::loadConfigVars(string filename) {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(filename, pt);

	LocalizerOptions options = { };

	options.resizeRatio = pt.get<float>(
			config::APPlICATION_ENVIROMENT + ".resize_ratio");
	options.initThres1 = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".init_thres_1");
	options.minMean1 = pt.get<float>(
			config::APPlICATION_ENVIROMENT + ".min_mean_thres_1");
	options.maxMean1 = pt.get<float>(
			config::APPlICATION_ENVIROMENT + ".max_mean_thres_1");
	options.initThres2 = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".init_thres_2");
	options.minMean2 = pt.get<float>(
			config::APPlICATION_ENVIROMENT + ".min_mean_thres_2");
	options.maxMean2 = pt.get<float>(
			config::APPlICATION_ENVIROMENT + ".max_mean_thres_2");
	options.dilateSize1 = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".dilate_size_1");
	options.erodeSize = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".erode_size");
	options.dilateSize2 = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".dilate_size_2");
	options.minSizeComb = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".min_size_comb");
	options.blurSize = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".blur_size");
	options.maxTagSize = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".max_size_tag");
	options.minTagSize = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".min_size_tag");

	this->_options = options;

}

cv::Mat Localizer::_generateThresholdImage(cv::Mat image, int thresh,
		float min_mean, float max_mean) {

	cv::Mat threshold_image;

	double average_old = 0, average_old_2 = 0;

	threshold: threshold(image, threshold_image, thresh, 255, THRESH_BINARY);

	Scalar mean = cv::mean(threshold_image);
	double average_value = mean.val[0];

	if (config::DEBUG_MODE_LOCALIZER) {
		cout << average_value << endl;
	}

	if (average_old_2 != average_value) {

		if (average_value < min_mean) {
			thresh--;
			average_old_2 = average_old;
			average_old = average_value;
			if (config::DEBUG_MODE_LOCALIZER)
				std::cout << "new threshold " << thresh << std::endl;

			goto threshold;
		} else if (average_value > max_mean) {
			thresh++;
			average_old_2 = average_old;
			average_old = average_value;
			if (config::DEBUG_MODE_LOCALIZER)
				std::cout << "new threshold " << thresh << std::endl;

			goto threshold;
		}
	}
	return threshold_image;
}

void Localizer::_filterColors(vector<Tag> &taglist) {
	for (int i = 0; i < taglist.size(); i++) {
		Tag tag = taglist[i];
		Mat subimage = tag.getOrigSubImage();

		Scalar average;
		Scalar stdDev;

		double minVal, maxVal;

		minMaxLoc(subimage, &minVal, &maxVal);
		if (minVal >= 60) {
			taglist.erase(taglist.begin() + i);
		} else if (minVal >= 35) {
			Scalar average;
			Scalar stdDev;
			meanStdDev(subimage, average, stdDev);
			double average_value = average.val[0];
			double std_value = stdDev.val[0];
			if (std_value < 25) {

				taglist.erase(taglist.begin() + i);

			}

		}

	}

}

}

