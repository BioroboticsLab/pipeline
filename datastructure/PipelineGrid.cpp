#include "PipelineGrid.h"

#include <algorithm>
#include <random>

#include "source/utility/CvHelper.h"

namespace {
static const cv::Scalar whiteC1(255);
static const cv::Scalar blackC1(0);

static uint8_t CONTOUR_OFFSET = Grid::INDEX_MIDDLE_CELLS_END + 10;
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
	assert(img.channels() == 1);

	for (const cv::Point& point : getOuterRingCoordinates().areaCoordinates) {
		uint8_t* ptr = (img.ptr<uint8_t>(point.y));
		ptr[point.x] = 200;
	}
	for (const cv::Point& point : getInnerWhiteRingCoordinates().areaCoordinates) {
		uint8_t* ptr = (img.ptr<uint8_t>(point.y));
		ptr[point.x] = 255;
	}
	for (const cv::Point& point : getInnerBlackRingCoordinates().areaCoordinates) {
		uint8_t* ptr = (img.ptr<uint8_t>(point.y));
		ptr[point.x] = 0;
	}
	for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++idx) {
		for (const cv::Point& point : getGridCellCoordinates(idx).areaCoordinates) {
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
            for (cv::Point& point : coordinates.get().areaCoordinates) {
                point = point - _center + center;
            }
            for (cv::Point& point : coordinates.get().edgeCoordinates) {
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

	// for each polygon, we speed up the calulation by iterating over the
	// bounding box of the polygon instead of the bounding box of the grid
	cv::Rect box = getPolygonBoundingBox(idx);

	// we actually have to draw the polygons in order to be able to extract the
	// coordinates. because the coordinates are internally centered at [0, 0],
	// we have to shift them initially.
	// this is unnecessary overhead and will be removed as soon as Tobi's
	// functions for polygon rasterization are finished.
    std::vector<cv::Point> shiftedPoints;
    shiftedPoints.reserve(_coordinates2D[idx].size());
    for (const cv::Point2i& origPoint : _coordinates2D[idx]) {
        shiftedPoints.push_back(origPoint - _boundingBox.tl() - box.tl());
    }

	// draw polygon and edges on internal id image representation (see comment
	// in header for _idImage
    cv::Mat roi = _idImage(box);
    cv::fillConvexPoly(roi, shiftedPoints, idx, 8);
    CvHelper::drawPolyline(roi, shiftedPoints, CONTOUR_OFFSET + idx, false, cv::Point(), 3);

	// iterate over roi and extract edge coordinates and store them in the
	// associated vector
	const cv::Point offset = box.tl() + _boundingBox.tl() + _center;
	const int nRows = roi.rows;
	const int nCols = roi.cols;
	uint8_t* point;
	for (int i = 0; i < nRows; ++i) {
		point = roi.ptr<uint8_t>(i);
		for (int j = 0; j < nCols; ++j) {
			if (point[j] == CONTOUR_OFFSET + idx) coordinates.edgeCoordinates.push_back(cv::Point2i(j, i) + offset);
		}
	}

	// when calculating the coordinates of the outer ring polygon, we have to
	// make sure that a) the outer ring is drawn first and b) the other area
	// are drawn before extracting the outer ring coordinates (because initially
	// the outer ring polygon also contains all or the coordinates that actually
	// belong to one of the other areas.
    if (idx == INDEX_OUTER_WHITE_RING) {
        _innerWhiteRingCoordinates = calculatePolygonCoordinates(INDEX_INNER_WHITE_SEMICIRCLE);
        _innerBlackRingCoordinates = calculatePolygonCoordinates(INDEX_INNER_BLACK_SEMICIRCLE);
        for (size_t cellIdx = Grid::INDEX_MIDDLE_CELLS_BEGIN; cellIdx < Grid::INDEX_MIDDLE_CELLS_END; ++cellIdx) {
            _gridCellCoordinates[cellIdx - Grid::INDEX_MIDDLE_CELLS_BEGIN] = calculatePolygonCoordinates(cellIdx);
        }
    }

	for (int i = 0; i < nRows; ++i) {
		point = roi.ptr<uint8_t>(i);
		for (int j = 0; j < nCols; ++j) {
			if (point[j] == idx) coordinates.areaCoordinates.push_back(cv::Point2i(j, i) + offset);
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

double PipelineGrid::compare(const PipelineGrid &to) const
{
    // distance of centers, inversed
    // max score is 1, min 0
    double d1 = 1 / ( 1 + norm( this->_center - to.getCenter() ));

    // get rotation matrices of both grids
    const auto R = CvHelper::rotationMatrix(this->_angle_z, this->_angle_y, this->_angle_x);
    const auto R2 = CvHelper::rotationMatrix(to.getZRotation(), to.getYRotation(), to.getXRotation());

    // multiply element-wise (dot-product) and sum up all elements
    // max should be 3 when all three base vectors point in the same directions
    // divide by 3, thus max is 1, min is -1
    double d2 = cv::sum( R.mul(R2) )[0] / 3;

    return d1 + d2;
}
