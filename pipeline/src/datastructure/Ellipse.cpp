#include "../../datastructure//Ellipse.h"

#include <opencv2/imgproc/imgproc.hpp> // cv::adaptiveThreshold
#include <stdexcept>                   // std::invalid argument

namespace pipeline {
Ellipse::Ellipse(){

}

Ellipse::Ellipse(int vote, cv::Point2i center, cv::Size2d axis_length, double angle,
                 cv::Size roiSize)
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
    cv::ellipse(mask, _cen, cv::Size2i(static_cast<int>(_axis.width), static_cast<int>(_axis.height)) +
                axisBorder, _angle, 0, 360, COLOR_WHITE, -1);

	return mask;
}


}
