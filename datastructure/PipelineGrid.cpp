#include "PipelineGrid.h"

#include <algorithm>
#include <random>

#ifdef PipelineStandalone
// TODO: FIXME!
#include "CvHelper.h"
#endif

#include "PipelineGrid.impl.h"

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

void PipelineGrid::draw(cv::Mat& img)
{
	class area_fill_t {
	public:
		area_fill_t(cv::Mat& img, const uint8_t color)
		    : _img(img), _color(color)
		{}

		inline void operator()(const cv::Point coord) {
			_img.get().at<uint8_t>(coord) = _color;
		}

		void setColor(const uint8_t color) { _color = color; }

	private:
		std::reference_wrapper<cv::Mat> _img;
		uint8_t _color;
	};

	assert(img.channels() == 1);

	static const uint8_t COLOR_OUTER       = 200;
	static const uint8_t COLOR_INNER_WHITE = 255;
	static const uint8_t COLOR_INNER_BLACK = 0;
	static const uint8_t COLOR_CELL_WHITE  = 220;
	static const uint8_t COLOR_CELL_BLACK  = 60;

	area_fill_t fillFun(img, COLOR_INNER_WHITE);
	fillFun = processInnerWhiteRingCoordinates(std::move(fillFun));

	fillFun.setColor(COLOR_INNER_BLACK);
	fillFun = processInnerBlackRingCoordinates(std::move(fillFun));

	for (size_t idx = 0; idx < Grid::NUM_MIDDLE_CELLS; ++idx) {
		fillFun.setColor(idx % 2 ? COLOR_CELL_WHITE : COLOR_CELL_BLACK);
		fillFun = processGridCellCoordinates(idx, std::move(fillFun));
	}

	fillFun.setColor(COLOR_OUTER);
	fillFun = processOuterRingCoordinates(std::move(fillFun));
}

void PipelineGrid::drawContours(cv::Mat& img, const double transparency, const cv::Vec3b& color) const
{
	assert(img.channels() == 3);

	class edge_fill_t {
	public:
		edge_fill_t(cv::Mat& img, const cv::Vec3b color, const cv::Point offset) : _img(img), _color(color), _offset(offset) {}

		inline void operator()(const cv::Point coord) {
			_img.get().at<cv::Vec3b>(coord - _offset) = _color;
		}

		inline void setExpectedSobelGradient(const double, const double) const {}

	private:
		std::reference_wrapper<cv::Mat> _img;
		cv::Vec3b _color;
		cv::Point _offset;
	};

	const int radius = static_cast<int>(std::ceil(_radius));
	const cv::Point subimage_origin( std::max(       0, _center.x - radius), std::max(       0, _center.y - radius) );
	const cv::Point subimage_end   ( std::min(img.cols, _center.x + radius), std::min(img.rows, _center.y + radius) );

	// draw only if subimage has a valid size (i.e. width & height > 0)
	if (subimage_origin.x < subimage_end.x && subimage_origin.y < subimage_end.y)
	{
		cv::Mat subimage      = img( cv::Rect(subimage_origin, subimage_end) );
		cv::Mat subimage_copy = subimage.clone();

		const auto drawIfValid = [&](const size_t idx, cv::Vec3b const& color) {
			edge_fill_t fillFun(subimage_copy, color, subimage_origin);

			if (_coordinates2D[idx].size() >= 2) {
				fillFun = processEdgeCoordinates(idx, std::move(fillFun));
			}
		};

		for (size_t i = INDEX_MIDDLE_CELLS_BEGIN; i < INDEX_MIDDLE_CELLS_BEGIN + NUM_MIDDLE_CELLS; ++i)
		{
			drawIfValid(i, color);
		}

		drawIfValid(INDEX_OUTER_WHITE_RING, color);
		drawIfValid(INDEX_INNER_WHITE_SEMICIRCLE, color);
		drawIfValid(INDEX_INNER_BLACK_SEMICIRCLE, color);

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

void PipelineGrid::resetCache()
{
	_innerWhiteRingCoordinates.reset();
	_innerBlackRingCoordinates.reset();
	_outerRingCoordinates.reset();
	for (cached_coordinates_t& coordinates : _gridCellCoordinates) {
		coordinates.reset();
	}

	_idImage = cv::Mat(_boundingBox.size(), CV_8UC1, cv::Scalar(NOID));
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
	double d1 = 10 / ( 10 + norm( this->_center - to.getCenter() ));

	// get rotation matrices of both grids
	const auto R = CvHelper::rotationMatrix(this->_angle_z, this->_angle_y, this->_angle_x);
	const auto R2 = CvHelper::rotationMatrix(to.getZRotation(), to.getYRotation(), to.getXRotation());

	auto CP = R.mul(R2);

	double cp1 = cv::sum( CP.col(0) )[0];
	double cp2 = cv::sum( CP.col(1) )[0];
	double cp3 = cv::sum( CP.col(2) )[0];

	//    double cp1 = cv::sum( CP.row(0) )[0];
	//    double cp2 = cv::sum( CP.row(1) )[0];
	//    double cp3 = cv::sum( CP.row(2) )[0];

	return cv::min(d1, cp1*cp2*cp3*cp1*cp2*cp3*cp1*cp2*cp3);
}

