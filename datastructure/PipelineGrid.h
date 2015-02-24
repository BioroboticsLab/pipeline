#pragma once

#include <array>

#include <boost/optional.hpp>

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

class PipelineGrid : private Grid {
public:
	typedef std::vector<cv::Point2i> coordinates_t;

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

	coordinates_t const& getOuterRingCoordinates();
	coordinates_t const& getInnerWhiteRingCoordinates();
	coordinates_t const& getInnerBlackRingCoordinates();
	coordinates_t const& getGridCellCoordinates(const size_t idx);

	const std::vector<cv::Point2i> getOuterRingEdgeCoordinates();

	cv::Mat getProjectedImage(const cv::Size2i size) const;
	void draw(cv::Mat& img, const double transparency);
	void draw(cv::Mat& img, const double transparency) const;

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

private:
    typedef boost::optional<coordinates_t> cached_coordinates_t;
    cv::Mat _idImage;
    cached_coordinates_t _outerRingCoordinates;
    cached_coordinates_t _innerWhiteRingCoordinates;
    cached_coordinates_t _innerBlackRingCoordinates;
    std::array<cached_coordinates_t, NUM_MIDDLE_CELLS> _gridCellCoordinates;

    cv::Rect getPolygonBoundingBox(size_t idx);

    coordinates_t calculatePolygonCoordinates(const size_t idx);

    void resetCache();
};
