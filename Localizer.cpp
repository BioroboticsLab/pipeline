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
	this->loadConfigVars(config::DEFAULT_LOCALIZER_CONFIG);

}

Localizer::Localizer(string configFile) {
	this->loadConfigVars(configFile);

}

Localizer::~Localizer() {
	// TODO Auto-generated destructor stub
}


/**************************************
 *
 * 			getter/setter
 *
 **************************************/

const Mat& Localizer::getBlob() const {
	return blob_;
}

void Localizer::setBlob(const Mat& blob) {
	blob_ = blob;
}

const Mat& Localizer::getCannyMap() const {
	return canny_map_;
}

void Localizer::setCannyMap(const Mat& cannyMap) {
	canny_map_ = cannyMap;
}

const Mat& Localizer::getGrayImage() const {
	return gray_image_;
}

void Localizer::setGrayImage(const Mat& grayImage) {
	gray_image_ = grayImage;
}

int Localizer::getLocalizerBinthres() const {
	return LOCALIZER_BINTHRES;
}

void Localizer::setLocalizerBinthres(int localizerBinthres) {
	LOCALIZER_BINTHRES = localizerBinthres;
}

int Localizer::getLocalizerDilation1Iterations() const {
	return LOCALIZER_DILATION_1_ITERATIONS;
}

void Localizer::setLocalizerDilation1Iterations(
		int localizerDilation1Iterations) {
	LOCALIZER_DILATION_1_ITERATIONS = localizerDilation1Iterations;
}

int Localizer::getLocalizerDilation1Size() const {
	return LOCALIZER_DILATION_1_SIZE;
}

void Localizer::setLocalizerDilation1Size(int localizerDilation1Size) {
	LOCALIZER_DILATION_1_SIZE = localizerDilation1Size;
}

int Localizer::getLocalizerDilation2Size() const {
	return LOCALIZER_DILATION_2_SIZE;
}

void Localizer::setLocalizerDilation2Size(int localizerDilation2Size) {
	LOCALIZER_DILATION_2_SIZE = localizerDilation2Size;
}

int Localizer::getLocalizerErosionSize() const {
	return LOCALIZER_EROSION_SIZE;
}

void Localizer::setLocalizerErosionSize(int localizerErosionSize) {
	LOCALIZER_EROSION_SIZE = localizerErosionSize;
}

int Localizer::getLocalizerHcannythres() const {
	return LOCALIZER_HCANNYTHRES;
}

void Localizer::setLocalizerHcannythres(int localizerHcannythres) {
	LOCALIZER_HCANNYTHRES = localizerHcannythres;
}

int Localizer::getLocalizerLcannythres() const {
	return LOCALIZER_LCANNYTHRES;
}

void Localizer::setLocalizerLcannythres(int localizerLcannythres) {
	LOCALIZER_LCANNYTHRES = localizerLcannythres;
}

int Localizer::getLocalizerMaxtagsize() const {
	return LOCALIZER_MAXTAGSIZE;
}

void Localizer::setLocalizerMaxtagsize(int localizerMaxtagsize) {
	LOCALIZER_MAXTAGSIZE = localizerMaxtagsize;
}

int Localizer::getLocalizerMintagsize() const {
	return LOCALIZER_MINTAGSIZE;
}

void Localizer::setLocalizerMintagsize(int localizerMintagsize) {
	LOCALIZER_MINTAGSIZE = localizerMintagsize;
}

const Mat& Localizer::getSobel() const {
	return sobel_;
}

void Localizer::setSobel(const Mat& sobel) {
	sobel_ = sobel;
}


/**************************************
 *
 * 			stuff
 *
 **************************************/


TagList Localizer::process(cv::Mat grayImage) {

	this->gray_image_ = grayImage;

	// compute the sobel derivative first
	this->sobel_ = this->computeSobelMap(grayImage);

	// and then locate the tags using the sobel map
	this->blob_ = this->computeBlobs(this->sobel_);

	// compute canny edge map. Needed for ellipse detection but needs to be done only once per image.
	this->canny_map_ = this->computeCannyEdgeMap(grayImage);

	TagList taglist = this->locateTagCandidates(this->blob_,
			this->canny_map_, this->gray_image_);

	return taglist;

}

/**
 * Highlight tag candidates in a comb image by intensity values
 *
 * @param grayImage
 * @return image with highlighted tags
 */
Mat Localizer::highlightTags(Mat &grayImage) {

	Mat imageCopy, imageCopy2;
	//eroded image
	Mat erodedImage;
	//dilated image
	Mat dilatedImage;
	Mat binarizedImage;

	//binarization
	threshold(grayImage, binarizedImage, this->LOCALIZER_BINTHRES, 255,
			CV_THRESH_BINARY);

	binarizedImage.copyTo(imageCopy);


	if (config::DEBUG_MODE) {

		namedWindow("binarized Image", WINDOW_NORMAL);
		imshow("binarized Image", imageCopy);
		waitKey(0);
		destroyWindow("binarized Image");
	}

	binarizedImage.copyTo(imageCopy2);

	//cv::MORPH_OPEN
	dilatedImage = getStructuringElement(MORPH_ELLIPSE,
				Size(2 * this->LOCALIZER_DILATION_1_SIZE + 1,
						2 * this->LOCALIZER_DILATION_1_SIZE + 1),
				Point(this->LOCALIZER_DILATION_1_SIZE,
						this->LOCALIZER_DILATION_1_SIZE));
	dilate(imageCopy, imageCopy, dilatedImage, Point(-1, -1),
			this->LOCALIZER_DILATION_1_ITERATIONS);

	if (config::DEBUG_MODE) {

		namedWindow("First Dilate", WINDOW_NORMAL);
		imshow("First Dilate", imageCopy);
		waitKey(0);
		destroyWindow("First Dilate");
	}

	//erosion
	erodedImage = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * this->LOCALIZER_EROSION_SIZE + 1,
					2 * this->LOCALIZER_EROSION_SIZE + 1),
			Point(this->LOCALIZER_EROSION_SIZE,
					this->LOCALIZER_EROSION_SIZE));
	erode(imageCopy, imageCopy, erodedImage);
	if (config::DEBUG_MODE) {
		namedWindow("First Erode", WINDOW_NORMAL);
		imshow("First Erode", imageCopy);
		waitKey(0);
		destroyWindow("First Erode");
	}

	dilatedImage = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * this->LOCALIZER_DILATION_2_SIZE + 1,
					2 * this->LOCALIZER_DILATION_2_SIZE + 1),
			Point(this->LOCALIZER_DILATION_2_SIZE,
					this->LOCALIZER_DILATION_2_SIZE));
	dilate(imageCopy, imageCopy, dilatedImage);
	if (config::DEBUG_MODE) {
		namedWindow("My Window", WINDOW_NORMAL);
		imshow("My Window", imageCopy);
		waitKey(0);
		destroyWindow("My Window");
	}
	return imageCopy;
}

/**
 *  Find blobs in the binary input image Ib and filter them by their size
 *
 * @param blobImage binary comb image with highlighted tag candidates
 * @return boundingBoxes output vector of size-filtered bounding boxes
 */

TagList Localizer::locateTagCandidates(Mat blobImage_old,
		Mat cannyEdgeMap, Mat grayImage) {

	TagList  taglist  = TagList();
	vector<vector<Point2i> > contours;

	Mat blobImage;
	blobImage_old.copyTo(blobImage);

	//find intra-connected white pixels
	findContours(blobImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	//extract contour bounding boxes for tag candidates
	for (vector<vector<Point2i> >::iterator contour = contours.begin();
			contour != contours.end(); ++contour) {

		//filter contours which are too big
		if (contour->size() < this->LOCALIZER_MAXTAGSIZE) {

			Rect rec = boundingRect(*contour) * 2;

			if (rec.width < this->LOCALIZER_MINTAGSIZE) {
				int offset = abs(rec.width - this->LOCALIZER_MINTAGSIZE);
				rec.x = rec.x - offset / 2;
				rec.width = rec.width + offset;
			}

			if (rec.height < this->LOCALIZER_MINTAGSIZE) {
				int offset = abs(rec.height - this->LOCALIZER_MINTAGSIZE);
				rec.y = rec.y - offset / 2;
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

				Tag tag = Tag(rec);
				tag.setId(taglist.size()+1);

				Mat sub_image(cannyEdgeMap, rec);
				Mat subImageCanny_cp, subImageOrig_cp;
				sub_image.copyTo(subImageCanny_cp);
				tag.setCannySubImage(subImageCanny_cp);
				Mat sub_image_orig(grayImage, rec);
				sub_image_orig.copyTo(subImageOrig_cp);
				tag.setOrigSubImage(subImageOrig_cp);

				taglist.AddTag(tag);
			}
		}
	}

	return taglist;
}

/**
 * Computes the Sobel map for a given grayscale image.
 * @return sobelmap
 */
Mat Localizer::computeSobelMap(Mat grayImage) {

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
	if (config::DEBUG_MODE) {

		namedWindow("Sobel", WINDOW_NORMAL);
		imshow("Sobel", sobel);
		waitKey(0);
		destroyWindow("Sobel");
	}

	return sobel;

	//DEBUG_IMSHOW( "sobel", sobel );
}

/*
 Computes Blobs and finally finds the ROI's using the sobel map. The ROI's are stored in the boundingBoxes Vector.
 */

Mat Localizer::computeBlobs(Mat sobel) {

	Mat blob;
	blob = this->highlightTags(sobel);

	//DEBUG_IMSHOW("blob", blob);

	//vector<Rect> boundingBoxes = this->locateTagCandidates(blob);

//#ifdef _DEBUG
//	image.copyTo(output);
//
//	for ( unsigned int i = 0; i < boundingBoxes.size(); i++) {
//
//		cv::rectangle(output, boundingBoxes[i], Scalar(0, 0, 255), 3);
//	}
//#endif
	return blob;
}

/**
 *
 *
 *
 * @param grayImage
 * @return
 */
Mat Localizer::computeCannyEdgeMap(Mat grayImage) {
	Mat localGrayImage;
	grayImage.copyTo(localGrayImage);

	cv::GaussianBlur(localGrayImage, localGrayImage, Size(3, 3), 0, 0,
			BORDER_DEFAULT);

	Mat cannyEdgeMap;
	Canny(localGrayImage, cannyEdgeMap, this->LOCALIZER_LCANNYTHRES,
			this->LOCALIZER_HCANNYTHRES);

	return cannyEdgeMap;
	//DEBUG_IMSHOW( "cannyEdgeMap", cannyEdgeMap );
}

/**
 * loads param from config
 *
 * @param filename absolute path to the config file
 */
void Localizer::loadConfigVars(string filename) {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(filename, pt);

	 this->LOCALIZER_LCANNYTHRES= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".canny_threshold_low");
		 this->LOCALIZER_HCANNYTHRES= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".canny_threshold_high");
		 this->LOCALIZER_BINTHRES= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".binary_threshold");
		 this->LOCALIZER_DILATION_1_ITERATIONS= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".dilation_1_interation_number");
		 this->LOCALIZER_DILATION_1_SIZE= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".dilation_1_size");
		 this->LOCALIZER_EROSION_SIZE= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".erosion_size");
		 this->LOCALIZER_DILATION_2_SIZE= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".dilation_2_size");
		 this->LOCALIZER_MAXTAGSIZE= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".max_tag_size");
		 this->LOCALIZER_MINTAGSIZE= pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".min_tag_size");



}

}
