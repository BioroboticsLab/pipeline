#include "Ellipse.h"

#include <opencv2/imgproc/imgproc.hpp> // cv::adaptiveThreshold
#include <stdexcept>                   // std::invalid argument

namespace decoder {

Ellipse::Ellipse()
	: vote(0)
	, cen()
	, axis()
	, angle(0)
	, transformedImage()
	, binarizedImage()
{ }

Ellipse::Ellipse(int vote, cv::Point2i center, cv::Size axis_length, double angle)
	: vote(vote)
	, cen(center)
	, axis(axis_length)
	, angle(angle)
	, transformedImage()
	, binarizedImage()
{ }

const cv::Mat& Ellipse::getBinarizedImage() const {
	if (binarizedImage.empty() && ! transformedImage.empty() ) {
		cv::adaptiveThreshold(this->getTransformedImage(), binarizedImage, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 21, 3);
	}
	return binarizedImage;
}

void Ellipse::setTransformedImage(const cv::Mat& transformedImage) {
	if (transformedImage.empty()) {
		throw std::invalid_argument("emtpy image");
	}
	if (transformedImage.type() != CV_8U) {
		throw std::invalid_argument("not an 8 bit greyscale image");
	}
	else {
		this->transformedImage = transformedImage;
	}
	this->binarizedImage.release();
}

}
