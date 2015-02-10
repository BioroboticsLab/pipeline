#pragma once

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

class PipelineGrid : private Grid {
public:
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

	void setXRotation(double angle) { Grid::setXRotation(angle); }
	double getXRotation() const { return Grid::getXRotation(); }

	void setYRotation(double angle) { Grid::setYRotation(angle); }
	double getYRotation() const { return Grid::getYRotation(); }

	void setZRotation(double angle) { Grid::setZRotation(angle); }
	double getZRotation() const { return Grid::getZRotation(); }

	// TODO: shift coordinates instead of recalculating projection + coordinates
	void setCenter(cv::Point c) { Grid::setCenter(c); prepare_visualization_data(); }
	cv::Point getCenter() const { return Grid::getCenter(); }

	void setRadius(double radius) { Grid::setRadius(radius); }
	double getRadius() const { return Grid::getRadius(); }

	cv::Rect getBoundingBox() const { return Grid::getBoundingBox(); }

	cv::Mat getProjectedImage(const cv::Size2i size) const;
	cv::Mat getInnerCircleMask(const cv::Size2i size) const;
	cv::Mat getOuterRingMask(const cv::Size2i size) const;

	cv::Point2i getOuterRingCentroid() const;

	const cv::Mat& getInnerWhiteRingCoordinates(const cv::Size2i& size);
	const cv::Mat& getInnerBlackRingCoordinates(const cv::Size2i& size);
	const std::vector<cv::Mat>& getGridCellCoordinates(const cv::Size2i& size);
	const cv::Mat& getOuterRingCoordinates(const cv::Size2i& size);
	const std::vector<cv::Point2i> getOuterRingEdgeCoordinates();

private:
	// TODO: invalidate cache on param change + efficient update when only
	// position changes
	bool _innerWhiteRingCached;
	bool _innerBlackRingCached;
	bool _gridCellsCached;
	bool _outerRingCached;

	cv::Mat _innerWhiteRingCoordinates;
	cv::Mat _innerBlackRingCoordinates;
	std::vector<cv::Mat> _gridCellCoordinates;
	cv::Mat _outerRingCoordinates;

	cv::Mat getRingPoly(const size_t ringIndex, const cv::Size2i& size);

	virtual coordinates2D_t generate_3D_coordinates_from_parameters_and_project_to_2D() override;
};
