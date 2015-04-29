#include "Util.h"

#include <opencv2/opencv.hpp>

#ifdef PipelineStandalone
// TODO: FIXME!
#include "../datastructure/CvHelper.h"
#include "../datastructure/util.h"
#endif

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

namespace Util {

std::array<gridconfig_t, 2> gridCandidatesFromEllipse(const pipeline::Ellipse& ellipse, const double rotation)
{
	const cv::Point2i& cen = ellipse.getCen();
	const cv::Size2i& axes = ellipse.getAxis();
	const double angle     = ellipse.getAngle();

	const double theta_rad    = angle * CV_PI / 180.;
	const double minor        = axes.height;
	const double major        = axes.width;
	assert(minor <= major);
	const double ellipse_roll = std::acos(minor / major);

	// TODO: quick and dirty hack to estimate actual grid radius
	const double radius = major - 0.15 * std::abs(std::sin(ellipse_roll)) * major;

	auto getGrid = [&](const double roll) {
		// TODO: optimize for speed
		// TODO: add comments
		const auto rotationMatrix = CvHelper::rotationMatrix_z(theta_rad) * CvHelper::rotationMatrix_x(roll) * CvHelper::rotationMatrix_z(rotation);

		const double angle_z = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
		const double angle_y = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)) );
		const double angle_x = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));

		// shift correction
		const double theta_orth = theta_rad + CV_PI / 2.;
		const cv::Vec2d base_x(1, 0);
		const cv::Matx<double, 2, 2> shiftRotMat(std::cos(theta_orth), -std::sin(theta_orth),
		                                         std::sin(theta_orth), std::cos(theta_orth));
		const cv::Vec2d base_proj = shiftRotMat * base_x;

		const cv::Point2i shiftedCen(static_cast<int>(cen.x + base_proj[0] * std::sin(roll) * major / Grid::FOCAL_LENGTH),
		        static_cast<int>(cen.y + base_proj[1] * std::sin(roll) * major / Grid::FOCAL_LENGTH));
		assert(shiftedCen.x > 0);
		assert(shiftedCen.y > 0);

		return gridconfig_t { shiftedCen, radius, angle_z, angle_y, angle_x };
	};

	return {getGrid(ellipse_roll), getGrid(-ellipse_roll)};
}

}
