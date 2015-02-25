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
{
	resetCache();
}

PipelineGrid::PipelineGrid(const PipelineGrid::gridconfig_t& config)
	: PipelineGrid(config.center, config.radius, config.angle_z,
				   config.angle_y, config.angle_x)
{}

const PipelineGrid::coordinates_t& PipelineGrid::getInnerWhiteRingCoordinates()
{
	// outer ring coordinates always have to be calculated first
	if (!_outerRingCoordinates) getOuterRingCoordinates();

	return getCoordinates(_innerWhiteRingCoordinates, INDEX_INNER_WHITE_SEMICIRCLE);
}

const PipelineGrid::coordinates_t& PipelineGrid::getInnerBlackRingCoordinates()
{
	// outer ring coordinates always have to be calculated first
	if (!_outerRingCoordinates) getOuterRingCoordinates();

	return getCoordinates(_innerBlackRingCoordinates, INDEX_INNER_BLACK_SEMICIRCLE);
}

const PipelineGrid::coordinates_t& PipelineGrid::getGridCellCoordinates(const size_t idx)
{
	// outer ring coordinates always have to be calculated first
	if (!_outerRingCoordinates) getOuterRingCoordinates();

	return getCoordinates(_gridCellCoordinates[idx], idx + Grid::INDEX_MIDDLE_CELLS_BEGIN);
}

const PipelineGrid::coordinates_t& PipelineGrid::getOuterRingCoordinates()
{
	return getCoordinates(_outerRingCoordinates, INDEX_OUTER_WHITE_RING);
}

const PipelineGrid::coordinates_t& PipelineGrid::getCoordinates(PipelineGrid::cached_coordinates_t& coordinates, const size_t idx)
{
	// if already in cache, return previous results
	if (coordinates) return coordinates.get();

	coordinates = calculatePolygonCoordinates(idx);
	return coordinates.get();
}

cv::Mat PipelineGrid::getProjectedImage(const cv::Size2i size) const
{
    static const cv::Scalar white(255, 255, 255);
    static const cv::Scalar black(0, 0, 0);
    static const cv::Scalar gray(128, 128, 128);

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

void PipelineGrid::draw(cv::Mat& img, const double transparency)
{
	for (const cv::Point& point : getOuterRingCoordinates()) {
		uint8_t* ptr = (img.ptr<uint8_t>(point.y));
		ptr[point.x] = 200;
	}
	for (const cv::Point& point : getInnerWhiteRingCoordinates()) {
		uint8_t* ptr = (img.ptr<uint8_t>(point.y));
		ptr[point.x] = 255;
	}
	for (const cv::Point& point : getInnerBlackRingCoordinates()) {
		uint8_t* ptr = (img.ptr<uint8_t>(point.y));
		ptr[point.x] = 0;
	}
	for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++idx) {
		for (const cv::Point& point : getGridCellCoordinates(idx)) {
				uint8_t* ptr = (img.ptr<uint8_t>(point.y));
				ptr[point.x] = idx % 2 ? 220 : 60;
		}
	}
}

void PipelineGrid::drawContours(cv::Mat& img, const double transparency) const
{
    static const cv::Scalar white(255, 255, 255);
    static const cv::Scalar black(0, 0, 0);

    const int radius = static_cast<int>(std::ceil(_radius));
    const cv::Point subimage_origin( std::max(       0, _center.x - radius), std::max(       0, _center.y - radius) );
    const cv::Point subimage_end   ( std::min(img.cols, _center.x + radius), std::min(img.rows, _center.y + radius) );

    // draw only if subimage has a valid size (i.e. width & height > 0)
    if (subimage_origin.x < subimage_end.x && subimage_origin.y < subimage_end.y)
    {
        const cv::Point subimage_center( std::min(radius, _center.x), std::min(radius, _center.y) );
        cv::Mat subimage      = img( cv::Rect(subimage_origin, subimage_end) );
        cv::Mat subimage_copy = subimage.clone();

        for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
        {
                CvHelper::drawPolyline(subimage_copy, _coordinates2D, i, white, false, subimage_center);
        }

        CvHelper::drawPolyline(subimage_copy, _coordinates2D, INDEX_OUTER_WHITE_RING,       white, false, subimage_center);
        CvHelper::drawPolyline(subimage_copy, _coordinates2D, INDEX_INNER_WHITE_SEMICIRCLE, white,      false, subimage_center);
        CvHelper::drawPolyline(subimage_copy, _coordinates2D, INDEX_INNER_BLACK_SEMICIRCLE, black,      false, subimage_center);

        cv::addWeighted(subimage_copy, transparency, subimage, 1.0 - transparency, 0.0, subimage);
    }
}

void PipelineGrid::setCenter(cv::Point center)
{
    auto shiftCoordinates = [&](cached_coordinates_t& coordinates) {
        if (coordinates) {
            for (cv::Point& point : coordinates.get()) {
                point = point - _center + center;
            }
        }
    };

    shiftCoordinates(_outerRingCoordinates);
    shiftCoordinates(_innerBlackRingCoordinates);
    shiftCoordinates(_innerWhiteRingCoordinates);
    for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++idx) {
        shiftCoordinates(_gridCellCoordinates[idx]);
    }

    Grid::setCenter(center);
}

cv::Rect PipelineGrid::getPolygonBoundingBox(size_t idx)
{
    int minx = std::numeric_limits<int>::max();
    int miny = std::numeric_limits<int>::max();
    int maxx = std::numeric_limits<int>::min();
    int maxy = std::numeric_limits<int>::min();

    for (cv::Point2i const& point : _coordinates2D[idx]) {
        minx = std::min(minx, point.x);
        miny = std::min(miny, point.y);
        maxx = std::max(maxx, point.x);
        maxy = std::max(maxy, point.y);
    }

    return {minx - _boundingBox.tl().x, miny - _boundingBox.tl().y, maxx - minx, maxy - miny};
}

PipelineGrid::coordinates_t PipelineGrid::calculatePolygonCoordinates(const size_t idx)
{
	coordinates_t coordinates;

	cv::Rect box = getPolygonBoundingBox(idx);

    std::vector<cv::Point> shiftedPoints;
    shiftedPoints.reserve(_coordinates2D[idx].size());
    for (const cv::Point2i& origPoint : _coordinates2D[idx]) {
        shiftedPoints.push_back(origPoint - _boundingBox.tl() - box.tl());
    }

    cv::Mat roi = _idImage(box);
    cv::fillConvexPoly(roi, shiftedPoints, idx, 8);

    if (idx == INDEX_OUTER_WHITE_RING) {
        _innerWhiteRingCoordinates = calculatePolygonCoordinates(INDEX_INNER_WHITE_SEMICIRCLE);
        _innerBlackRingCoordinates = calculatePolygonCoordinates(INDEX_INNER_BLACK_SEMICIRCLE);
        for (size_t cellIdx = Grid::INDEX_MIDDLE_CELLS_BEGIN; cellIdx < Grid::INDEX_MIDDLE_CELLS_END; ++cellIdx) {
            _gridCellCoordinates[cellIdx - Grid::INDEX_MIDDLE_CELLS_BEGIN] = calculatePolygonCoordinates(cellIdx);
        }
    }

	const cv::Point offset = box.tl() + _boundingBox.tl() + _center;
	const int nRows = roi.rows;
	const int nCols = roi.cols;
	uint8_t* point;
	for (int i = 0; i < nRows; ++i) {
		point = roi.ptr<uint8_t>(i);
		for (int j = 0; j < nCols; ++j) {
			if (point[j] == idx) coordinates.push_back(cv::Point2i(j, i) + offset);
		}
	}

	return coordinates;
}

void PipelineGrid::resetCache()
{
	_innerWhiteRingCoordinates.reset();
	_innerBlackRingCoordinates.reset();
	_outerRingCoordinates.reset();
	for (cached_coordinates_t& coordinates : _gridCellCoordinates) {
		coordinates.reset();
	}

	_idImage = cv::Mat(_boundingBox.size(), CV_8UC1, cv::Scalar(255));
}

PipelineGrid::gridconfig_t PipelineGrid::getConfig() const
{
	return {_center, _radius / FOCAL_LENGTH, _angle_z, _angle_y, _angle_x};
}

const std::vector<cv::Point2i> PipelineGrid::getOuterRingEdgeCoordinates()
{
	std::vector<cv::Point2i> coords(_coordinates2D[INDEX_OUTER_WHITE_RING]);
	for (cv::Point2i& point : coords) {
		point += _center;
	}

	return coords;
}
