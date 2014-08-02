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

Localizer::Localizer() {
	// TODO Auto-generated constructor stub

}

Localizer::~Localizer() {
	// TODO Auto-generated destructor stub
}

vector<BoundingBox> Localizer::process(cv::Mat grayImage) {

	this->gray_image_ = grayImage;

	// compute the sobel derivative first
	this->sobel_ = this->computeSobelMap(grayImage);

	// and then locate the tags using the sobel map
	this->blob_ = this->computeBlobs(this->sobel_);

	// compute canny edge map. Needed for ellipse detection but needs to be done only once per image.
	this->canny_map_ = this->computeCannyEdgeMap(grayImage);

	vector<BoundingBox> bounding_boxes = this->locateTagCandidates(this->blob_,
			this->canny_map_, this->gray_image_);

	return bounding_boxes;

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
	threshold(grayImage, binarizedImage, config::LOCALIZER_BINTHRES, 255,
			CV_THRESH_BINARY);


	binarizedImage.copyTo(imageCopy);

	if (config::DEBUG_MODE) {
			dilate(imageCopy, imageCopy, dilatedImage, Point(-1, -1),
					config::LOCALIZER_DILATIONNUM_1);
			namedWindow("binarized Image", WINDOW_NORMAL);
			imshow("binarized Image", imageCopy);
			waitKey(0);
			destroyWindow("binarized Image");
		}


	binarizedImage.copyTo(imageCopy2);

	//cv::MORPH_OPEN
	dilatedImage = getStructuringElement(MORPH_OPEN, Size(3, 4),
			Point(1, 1));
	dilate(imageCopy, imageCopy, dilatedImage, Point(-1, -1),
					config::LOCALIZER_DILATIONNUM_1);

	if (config::DEBUG_MODE) {

		namedWindow("First Dilate", WINDOW_NORMAL);
		imshow("First Dilate", imageCopy);
		waitKey(0);
		destroyWindow("First Dilate");
	}

	//erosion
	erodedImage = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * config::LOCALIZER_EROSIONNUM_1 + 1,
					2 * config::LOCALIZER_EROSIONNUM_1 + 1),
			Point(config::LOCALIZER_EROSIONNUM_1,
					config::LOCALIZER_EROSIONNUM_1));
	erode(imageCopy, imageCopy, erodedImage);
	if (config::DEBUG_MODE) {
		namedWindow("First Erode", WINDOW_NORMAL);
		imshow("First Erode", imageCopy);
		waitKey(0);
		destroyWindow("First Erode");
	}
/*
	 Mat element = getStructuringElement( MORPH_ELLIPSE, Size( config::LOCALIZER_DILATIONNUM_1, config::LOCALIZER_DILATIONNUM_1 ), Point(1,1 ) );

	  /// Apply the specified morphology operation
	  morphologyEx( imageCopy, imageCopy, MORPH_OPEN, element );
	  */

	/*//erosion
	 erodedImage = getStructuringElement(MORPH_ELLIPSE,
	 Size(2 * config::LOCALIZER_EROSIONNUM + 1,
	 2 * config::LOCALIZER_EROSIONNUM + 1),
	 Point(config::LOCALIZER_EROSIONNUM, config::LOCALIZER_EROSIONNUM));
	 erode(imageCopy, imageCopy, erodedImage);
	 namedWindow("My Window", WINDOW_NORMAL);
	 imshow("My Window", imageCopy);
	 waitKey(0);
	 destroyWindow("My Window");*/
	//dilation
	dilatedImage = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * config::LOCALIZER_DILATIONNUM + 1,
					2 * config::LOCALIZER_DILATIONNUM + 1),
			Point(config::LOCALIZER_DILATIONNUM,
					config::LOCALIZER_DILATIONNUM));
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

vector<BoundingBox> Localizer::locateTagCandidates(Mat blobImage,
		Mat cannyEdgeMap, Mat grayImage) {

	vector<BoundingBox> boundingBoxes;
	vector<vector<Point2i> > contours;

	boundingBoxes = vector<BoundingBox>();

	//find intra-connected white pixels
	findContours(blobImage, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	//extract contour bounding boxes for tag candidates
	for (vector<vector<Point2i> >::iterator contour = contours.begin();
			contour != contours.end(); ++contour) {

		//filter contours which are too big
		if (contour->size() < config::LOCALIZER_MAXTAGSIZE) {

			Rect rec = boundingRect(*contour) * 2;

			if (rec.width < config::LOCALIZER_MINTAGSIZE) {
				int offset = abs(rec.width - config::LOCALIZER_MINTAGSIZE);
				rec.x = rec.x - offset / 2;
				rec.width = rec.width + offset;
			}

			if (rec.height < config::LOCALIZER_MINTAGSIZE) {
				int offset = abs(rec.height - config::LOCALIZER_MINTAGSIZE);
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

				BoundingBox bb = BoundingBox();
				bb.box_ = rec;

				Mat sub_image(cannyEdgeMap, rec);
				sub_image.copyTo(bb.sub_image_);
				Mat sub_image_orig(grayImage, rec);
				sub_image_orig.copyTo(bb.sub_image_orig_);

				boundingBoxes.push_back(bb);
			}
		}
	}

	return boundingBoxes;
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
	Canny(localGrayImage, cannyEdgeMap, config::LOCALIZER_LCANNYTHRES,
			config::LOCALIZER_HCANNYTHRES);

	return cannyEdgeMap;
	//DEBUG_IMSHOW( "cannyEdgeMap", cannyEdgeMap );
}

}
