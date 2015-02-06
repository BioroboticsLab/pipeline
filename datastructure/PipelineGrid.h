#pragma once

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

class PipelineGrid : Grid {
public:
	explicit PipelineGrid(cv::Point2i center, double radius, double angle_z, double angle_y, double angle_x);
	virtual ~PipelineGrid() {}

	cv::Mat getProjectedImage(const cv::Size2i size) const;
	cv::Mat getInnerCircleMask(const cv::Size2i size) const;

	cv::Point2i getOuterRingCentroid() const;
};
