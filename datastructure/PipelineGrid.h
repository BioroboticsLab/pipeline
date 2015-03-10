#pragma once

#include <array>

#include <boost/optional.hpp>

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

class PipelineGrid : private Grid {
public:
	typedef struct {
		// rasterized coordinates of an grid area (outer ring etc.) for the
		// current grid configuration
		std::vector<cv::Point2i> areaCoordinates;
		// rasterized coordinates of the outer edges of the area
		std::vector<cv::Point2i> edgeCoordinates;
	} coordinates_t;

	typedef struct {
		cv::Point2i center;
		double radius;
		double angle_z;
		double angle_y;
		double angle_x;
	} gridconfig_t;

	explicit PipelineGrid(cv::Point2i center, double radius, double angle_z, double angle_y, double angle_x);
	explicit PipelineGrid(gridconfig_t const& config);
	virtual ~PipelineGrid() {}

	gridconfig_t getConfig() const;

	/**
	 * @brief getOuterRingCoordinates
	 * @return coordinates of outer (white) ring
	 */
	coordinates_t const& getOuterRingCoordinates();

	/**
	 * @brief getInnerWhiteRingCoordinates
	 * @return  coordinates of inner white semicircle
	 */
	coordinates_t const& getInnerWhiteRingCoordinates();

	/**
	 * @brief getInnerBlackRingCoordinates
	 * @return  coordinates of inner black semicircle
	 */
	coordinates_t const& getInnerBlackRingCoordinates();

	/**
	 * @brief getGridCellCoordinates
	 * @param idx index of the grid cells in [0, Grid::NUM_MIDDLE_CELLS]
	 * @return coordinates of the requested grid cell
	 */
	coordinates_t const& getGridCellCoordinates(const size_t idx);

	// legacy code
	const std::vector<cv::Point2i> getOuterRingEdgeCoordinates();

	// TODO: maybe merge different draw functions
	cv::Mat getProjectedImage(const cv::Size2i size) const;
    void draw(cv::Mat& img, const double transparency);
    void drawContours(cv::Mat& img, const double transparency, const cv::Scalar color = cv::Scalar(255, 255, 255)) const;

	void setXRotation(double angle) { Grid::setXRotation(angle); resetCache(); }
	double getXRotation() const { return Grid::getXRotation(); }

	void setYRotation(double angle) { Grid::setYRotation(angle); resetCache(); }
	double getYRotation() const { return Grid::getYRotation(); }

	void setZRotation(double angle) { Grid::setZRotation(angle); resetCache(); }
	double getZRotation() const { return Grid::getZRotation(); }

	void setCenter(cv::Point center);
	cv::Point getCenter() const { return Grid::getCenter(); }

	void setRadius(double radius) { Grid::setRadius(radius); resetCache(); }
	double getRadius() const { return Grid::getRadius(); }

	cv::Rect getBoundingBox() const { return Grid::getBoundingBox(); }

	idarray_t const& getIdArray() const { return Grid::getIdArray(); }

    double compare(const PipelineGrid &to) const;

private:
	// contains the cached coordinates of the current grid parameters
	// or is invalid.
    typedef boost::optional<coordinates_t> cached_coordinates_t;

	// image representation of the different areas and edges of the rasterized
	// grid. during the calculation of the coordinates, each area/edge is filled
	// with a scalar value equal to the the index of the area. for edges, a
	// constant (CONTOUR_OFFSET) is added to the scalar.
    cv::Mat _idImage;

    cached_coordinates_t _outerRingCoordinates;
    cached_coordinates_t _innerWhiteRingCoordinates;
    cached_coordinates_t _innerBlackRingCoordinates;
    std::array<cached_coordinates_t, NUM_MIDDLE_CELLS> _gridCellCoordinates;

	// returns the bounding box of a single polygon (identified by the index
	// of the polygon area [0, Grid::NUM_MIDDLE_CELLS)
    cv::Rect getPolygonBoundingBox(size_t idx);

	// convenience function to avoid code duplication, either return the already
	// cached coordinates or calculates the coordinates and then returns them
    const coordinates_t& getCoordinates(cached_coordinates_t& coordinates, const size_t idx);

	// calculates the rasterized coordinates of the (convex) polygon with the
	// given index
    coordinates_t calculatePolygonCoordinates(const size_t idx);

	// resets the coordinate caches. has to be called after a change of
	// orientation or scale but not after a position change.
    void resetCache();
};
