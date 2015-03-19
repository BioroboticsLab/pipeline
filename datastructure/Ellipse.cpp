#include "Ellipse.h"

#include <opencv2/imgproc/imgproc.hpp> // cv::adaptiveThreshold
#include <stdexcept>                   // std::invalid argument

namespace pipeline {

Ellipse::Ellipse()
	: _vote(0)
	, _cen()
	, _axis()
	, _angle(0)
	, _roiSize(cv::Size(0, 0))
{ }

Ellipse::Ellipse(int vote, cv::Point2i center, cv::Size axis_length, double angle, cv::Size roiSize)
	: _vote(vote)
	, _cen(center)
	, _axis(axis_length)
	, _angle(angle)
	, _roiSize(roiSize)
{ }

const cv::Mat Ellipse::getMask(const cv::Size axisBorder) const
{
	static const cv::Scalar COLOR_WHITE(255);

	// 8 bit grayscale
	cv::Mat mask(_roiSize, CV_8UC1, cv::Scalar(0));
	cv::ellipse(mask, _cen, _axis + axisBorder, _angle, 0, 360, COLOR_WHITE, -1);

	return mask;
}

}
