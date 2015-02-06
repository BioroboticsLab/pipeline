#include "PipelineGrid.h"

#include <algorithm>
#include <random>

#include "source/utility/CvHelper.h"

PipelineGrid::PipelineGrid(cv::Point2i center, double radius, double angle_z, double angle_y, double angle_x)
	: Grid(center, radius, angle_z, angle_y, angle_x)
{}

cv::Mat PipelineGrid::getProjectedImage(const cv::Size2i size) const
{
	static const cv::Scalar white(255, 255, 255);
	static const cv::Scalar black(0, 0, 0);
	static const cv::Scalar gray(128, 128, 128);

	static std::default_random_engine generator;
	static std::uniform_int_distribution<int> distribution(0, 1);

	// use B&W matrix
	cv::Mat img(size, CV_8UC3);
	img = black;

	typedef std::vector<std::vector<cv::Point>> pointvecvec_t;

	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_OUTER_WHITE_RING]}, white, 8, 0, _center);
	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_INNER_WHITE_SEMICIRCLE]}, white, 8, 0, _center);
	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_INNER_BLACK_SEMICIRCLE]}, black, 8, 0, _center);
	for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
	{
		cv::Scalar col = white; // distribution(generator) ? black : white;
		std::vector<std::vector<cv::Point>> points { _coordinates2D[i] };
		cv::fillPoly(img, points, col, 8, 0, _center);
	}

	for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
	{
		CvHelper::drawPolyline(img, _coordinates2D, i, gray, false, _center);
	}
	CvHelper::drawPolyline(img, _coordinates2D, INDEX_OUTER_WHITE_RING,       gray, false, _center);

	return img;
}

cv::Mat PipelineGrid::getInnerCircleMask(const cv::Size2i size) const
{
	static const cv::Scalar white(255, 255, 255);
	static const cv::Scalar black(0, 0, 0);

	// use B&W matrix
	cv::Mat img(size, CV_8UC3);
	img = black;

	typedef std::vector<std::vector<cv::Point>> pointvecvec_t;

	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_OUTER_WHITE_RING]}, white, 8, 0, _center);
	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_INNER_WHITE_SEMICIRCLE]}, white, 8, 0, _center);
	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_INNER_BLACK_SEMICIRCLE]}, white, 8, 0, _center);
	for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
	{
		std::vector<std::vector<cv::Point>> points { _coordinates2D[i] };
		cv::fillPoly(img, points, black, 8, 0, _center);
	}

	return img;
}

#include <iostream>
cv::Point2i PipelineGrid::getOuterRingCentroid() const
{
	int64_t sumx = 0;
	int64_t sumy = 0;
	for (const cv::Point2i& point : _coordinates2D[INDEX_OUTER_WHITE_RING]) {
		sumx += point.x;
		sumy += point.y;
	}
	const int64_t num = _coordinates2D[INDEX_OUTER_WHITE_RING].size();
	std::cout << sumx << std::endl;
	std::cout << sumy << std::endl;
	std::cout << num << std::endl;
	const cv::Point2i centroid(static_cast<int>(sumx / num), static_cast<int>(sumy / num));

	return centroid;
}
