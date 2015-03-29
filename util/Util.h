#pragma once

#include "../datastructure/Ellipse.h"

namespace Util {

typedef struct {
	cv::Point2i center;
	double radius;
	double angle_z;
	double angle_y;
	double angle_x;
} gridconfig_t;


std::array<gridconfig_t, 2> gridCandidatesFromEllipse(const pipeline::Ellipse& ellipse, const double rotation = 0);

inline bool pointInBounds(cv::Rect const& bounds, cv::Point const& point) {
	return (point.x >= bounds.tl().x &&
	        point.y >= bounds.tl().y &&
	        point.x <  bounds.br().x &&
	        point.y <  bounds.br().y);
}

inline bool pointInBounds(cv::Size const& size, cv::Point const& point) {
	return pointInBounds(cv::Rect(0, 0, size.width, size.height), point);
}

}
