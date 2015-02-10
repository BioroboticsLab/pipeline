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

PipelineGrid::PipelineGrid(const PipelineGrid::gridconfig_t& config)
	: PipelineGrid(config.center, config.radius, config.angle_z,
				   config.angle_y, config.angle_x)
{}

PipelineGrid::gridconfig_t PipelineGrid::getConfig() const
{
	return {_center, _radius, _angle_z, _angle_y, _angle_x};
}

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
	cv::fillPoly(img, pointvecvec_t{_coordinates2D[INDEX_INNER_BLACK_SEMICIRCLE]}, gray, 8, 0, _center);
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
	CvHelper::drawPolyline(img, _coordinates2D, INDEX_OUTER_WHITE_RING, gray, false, _center);

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

    // determine bounding box of polygon
    int minx = std::numeric_limits<int>::max();
    int miny = std::numeric_limits<int>::max();
    int maxx = std::numeric_limits<int>::min();
    int maxy = std::numeric_limits<int>::min();
    for (cv::Point2i const& point : _coordinates2D[INDEX_INNER_WHITE_SEMICIRCLE]) {
            minx = std::min(minx, point.x);
            miny = std::min(miny, point.y);
            maxx = std::max(maxx, point.x);
            maxy = std::max(maxy, point.y);
    }
    const cv::Rect boundingBox(minx + _center.x, miny + _center.y, maxx - minx, maxy - miny);

    const cv::Mat img = getRingPoly(INDEX_INNER_WHITE_SEMICIRCLE, size);

    cv::Mat& outputArray = _innerWhiteRingCoordinates;

    // find non zero elements only inside of bounding box
    cv::Mat roi;
    roi = img(boundingBox);
    cv::findNonZero(roi, outputArray);

    // shift roi coordinates -> img coordinates
    for (size_t idx = 0; idx < outputArray.total(); ++idx) {
        outputArray.at<cv::Point2i>(idx) += boundingBox.tl();
    }

	_innerWhiteRingCached = true;
	return _innerWhiteRingCoordinates;
}

const cv::Mat& PipelineGrid::getInnerBlackRingCoordinates(const cv::Size2i& size)
{
	if (_innerBlackRingCached) return _innerBlackRingCoordinates;

    // determine bounding box of polygon
    int minx = std::numeric_limits<int>::max();
    int miny = std::numeric_limits<int>::max();
    int maxx = std::numeric_limits<int>::min();
    int maxy = std::numeric_limits<int>::min();
    for (cv::Point2i const& point : _coordinates2D[INDEX_INNER_BLACK_SEMICIRCLE]) {
        // TODO: debug points with invalid coordinates!
        if (point.x + _center.x >= 0 && point.y + _center.y >= 0 &&
                point.x + _center.x < size.width && point.y + _center.y < size.height) {
            minx = std::min(minx, point.x);
            miny = std::min(miny, point.y);
            maxx = std::max(maxx, point.x);
            maxy = std::max(maxy, point.y);
        }
    }
    const cv::Rect boundingBox(minx + _center.x, miny + _center.y, maxx - minx, maxy - miny);

    const cv::Mat img = getRingPoly(INDEX_INNER_BLACK_SEMICIRCLE, size);

    cv::Mat& outputArray = _innerBlackRingCoordinates;

    // find non zero elements only inside of bounding box
    cv::Mat roi;
    roi = img(boundingBox);
    cv::findNonZero(roi, outputArray);

    // shift roi coordinates -> img coordinates
    for (size_t idx = 0; idx < outputArray.total(); ++idx) {
        outputArray.at<cv::Point2i>(idx) += boundingBox.tl();
    }

	_innerBlackRingCached = true;
	return _innerBlackRingCoordinates;
}

const std::vector<cv::Mat>& PipelineGrid::getGridCellCoordinates(const cv::Size2i& size)
{
	if (_gridCellsCached) return _gridCellCoordinates;

	cv::Mat img(size, CV_8UC1, cv::Scalar::all(0));
	for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
	{
		// determine bounding box of polygon
		int minx = std::numeric_limits<int>::max();
		int miny = std::numeric_limits<int>::max();
		int maxx = std::numeric_limits<int>::min();
		int maxy = std::numeric_limits<int>::min();
		for (cv::Point2i const& point : _coordinates2D[i]) {
			// TODO: debug points with invalid coordinates!
			if (point.x + _center.x >= 0 && point.y + _center.y >= 0 &&
					point.x + _center.x < size.width && point.y + _center.y < size.height) {
				minx = std::min(minx, point.x);
				miny = std::min(miny, point.y);
				maxx = std::max(maxx, point.x);
				maxy = std::max(maxy, point.y);
			}
		}
		const cv::Rect boundingBox(minx + _center.x, miny + _center.y, maxx - minx, maxy - miny);

		// avoid copy
		const cv::Point* points[1] = { &_coordinates2D[i].front() };
		const int numpoints[1] = { static_cast<int>(_coordinates2D[i].size()) };
		cv::fillPoly(img, points, numpoints, 1, whiteC1, 8, 0, _center);

		cv::Mat& outputArray = _gridCellCoordinates[i - INDEX_MIDDLE_CELLS_BEGIN];

		// find non zero elements only inside of bounding box
		cv::Mat roi;
		roi = img(boundingBox);
		cv::findNonZero(roi, outputArray);

        // shift roi coordinates -> img coordinates
        for (size_t idx = 0; idx < outputArray.total(); ++idx) {
            outputArray.at<cv::Point2i>(idx) += boundingBox.tl();
        }

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

const std::vector<cv::Point2i> PipelineGrid::getOuterRingEdgeCoordinates()
{
	std::vector<cv::Point2i> coords(_coordinates2D[INDEX_OUTER_WHITE_RING]);
	for (cv::Point2i& point : coords) {
		point += _center;
	}

	return coords;
}

cv::Mat PipelineGrid::getRingPoly(const size_t ringIndex, const cv::Size2i& size)
{
	cv::Mat img(size, CV_8UC1, cv::Scalar::all(0));

    const cv::Point* points[1] = { &_coordinates2D[ringIndex].front() };
    const int numpoints[1] = { static_cast<int>(_coordinates2D[ringIndex].size()) };
    cv::fillPoly(img, points, numpoints, 1, whiteC1, 8, 0, _center);

    return img;
}

Grid::coordinates2D_t PipelineGrid::generate_3D_coordinates_from_parameters_and_project_to_2D()
{
	_innerWhiteRingCached = false;
	_innerBlackRingCached = false;
	_gridCellsCached      = false;
	_outerRingCached      = false;

	_innerWhiteRingCoordinates = cv::Mat();
	_innerBlackRingCoordinates = cv::Mat();
	_outerRingCoordinates = cv::Mat();
	_gridCellCoordinates = std::vector<cv::Mat>(NUM_MIDDLE_CELLS);

	return Grid::generate_3D_coordinates_from_parameters_and_project_to_2D();
}
