#pragma once

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

class PipelineGrid : Grid {
public:
	explicit PipelineGrid(cv::Point2i center, double radius, double angle_z, double angle_y, double angle_x);
	virtual ~PipelineGrid() {}

	cv::Mat getProjectedImage(const cv::Size2i size) const;
	cv::Mat getInnerCircleMask(const cv::Size2i size) const;
	cv::Mat getOuterRingMask(const cv::Size2i size) const;

	cv::Point2i getOuterRingCentroid() const;

	const cv::Mat& getInnerWhiteRingCoordinates(const cv::Size2i& size);
	const cv::Mat& getInnerBlackRingCoordinates(const cv::Size2i& size);
	const std::vector<cv::Mat>& getGridCellCoordinates(const cv::Size2i& size);
	const cv::Mat& getOuterRingCoordinates(const cv::Size2i& size);

private:
	bool _innerWhiteRingCached;
	bool _innerBlackRingCached;
	bool _gridCellsCached;
	bool _outerRingCached;

	cv::Mat _innerWhiteRingCoordinates;
	cv::Mat _innerBlackRingCoordinates;
	std::vector<cv::Mat> _gridCellCoordinates;
	cv::Mat _outerRingCoordinates;

	cv::Mat getRingPoly(const size_t ringIndex, const cv::Size2i& size);
};
