#include "PipelineGrid.h"

#include <algorithm>
#include <random>

#include "source/utility/CvHelper.h"

namespace {
static const cv::Scalar whiteC1(255);
static const cv::Scalar blackC1(0);
}

PipelineGrid::PipelineGrid(cv::Point2i center, double radius, double angle_z, double angle_y, double angle_x)
	: Grid(center, radius, angle_z, angle_y, angle_x)
	, _innerWhiteRingCached(false)
	, _innerBlackRingCached(false)
	, _gridCellsCached(false)
	, _outerRingCached(false)
	, _gridCellCoordinates(NUM_MIDDLE_CELLS)
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

cv::Mat PipelineGrid::getOuterRingMask(const cv::Size2i size) const
{
	static const cv::Scalar white(255, 255, 255);
	static const cv::Scalar black(0, 0, 0);

	// use B&W matrix
	cv::Mat img(size, CV_8UC1, black);

	typedef std::vector<std::vector<cv::Point>> pointvecvec_t;

	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_OUTER_WHITE_RING]}, white, 8, 0, _center);

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

const cv::Mat& PipelineGrid::getInnerWhiteRingCoordinates(const cv::Size2i& size)
{
	if (_innerWhiteRingCached) return _innerWhiteRingCoordinates;

	cv::findNonZero(getRingPoly(INDEX_INNER_WHITE_SEMICIRCLE, size), _innerWhiteRingCoordinates);

	_innerWhiteRingCached = true;
	return _innerWhiteRingCoordinates;
}

const cv::Mat& PipelineGrid::getInnerBlackRingCoordinates(const cv::Size2i& size)
{
	if (_innerBlackRingCached) return _innerBlackRingCoordinates;

	cv::findNonZero(getRingPoly(INDEX_INNER_BLACK_SEMICIRCLE, size), _innerBlackRingCoordinates);

	_innerBlackRingCached = true;
	return _innerBlackRingCoordinates;
}

const std::vector<cv::Mat>& PipelineGrid::getGridCellCoordinates(const cv::Size2i& size)
{
	if (_gridCellsCached) return _gridCellCoordinates;

	cv::Mat img(size, CV_8UC1, cv::Scalar::all(0));
	for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
	{
		std::vector<std::vector<cv::Point>> points { _coordinates2D[i] };
		cv::fillPoly(img, points, whiteC1, 8, 0, _center);
		cv::findNonZero(img, _gridCellCoordinates[i - INDEX_MIDDLE_CELLS_BEGIN]);
		img.setTo(0);
	}

	_gridCellsCached = true;
	return _gridCellCoordinates;
}

const cv::Mat& PipelineGrid::getOuterRingCoordinates(const cv::Size2i& size)
{
	if (_outerRingCached) return _outerRingCoordinates;

	cv::Mat outerRing = getRingPoly(INDEX_OUTER_WHITE_RING, size);
	const std::vector<cv::Mat>& gridCellCoordinates = getGridCellCoordinates(size);
	const cv::Mat& blackSemicircle = getInnerBlackRingCoordinates(size);
	const cv::Mat& whiteSemicircle = getInnerWhiteRingCoordinates(size);

	auto removeCoords = [&](const cv::Mat& indices) {
		for (size_t idx = 0; idx < indices.total(); ++idx) {
			const cv::Point2i& pt = indices.at<cv::Point2i>(idx);
			outerRing.at<uint8_t>(pt.y, pt.x) = 0;
		}
	};

	// remove grid cells from polygon
	// TODO: could be implemented more efficently
	for (const cv::Mat& cell : gridCellCoordinates) {
		removeCoords(cell);
	}

    // remove inner semicircles from polygon
    removeCoords(blackSemicircle);
    removeCoords(whiteSemicircle);

    cv::findNonZero(outerRing, _outerRingCoordinates);

	_outerRingCached = true;
	return _outerRingCoordinates;
}

cv::Mat PipelineGrid::getRingPoly(const size_t ringIndex, const cv::Size2i& size)
{
	cv::Mat img(size, CV_8UC1, cv::Scalar::all(0));

	// TODO: unnecessary overhead
	typedef std::vector<std::vector<cv::Point>> pointvecvec_t;

	cv::fillPoly(img, pointvecvec_t{_coordinates2D[ringIndex]}, whiteC1, 8, 0, _center);

	return img;
}
