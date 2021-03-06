#include "../../util/Util.h"

#include <opencv2/opencv.hpp>

#include "../../util/CvHelper.h"
#include "../../util/Quaternions.h"

#include "../../common/Grid.h"

namespace Util {

std::array<gridconfig_t, 2> gridCandidatesFromEllipse(const pipeline::Ellipse& ellipse, const double rotation,
													  const double focal_length)
{
	const cv::Point2i& cen = ellipse.getCen();
    const cv::Size2d& axes = ellipse.getAxis();
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
        const auto rotationQuat = Util::getIntrinsicRotationQuatZXZ(theta_rad, roll, rotation);

        const double pi = boost::math::constants::pi<double>();
        assert(!(std::abs(roll - pi / 2) < 0.000001 && std::abs(rotation - pi / 2) < 0.000001));
        const auto angles = Util::getEulerAngles(rotationQuat);

		// shift correction
		const double theta_orth = theta_rad + CV_PI / 2.;
		const cv::Vec2d base_x(1, 0);
		const cv::Matx<double, 2, 2> shiftRotMat(std::cos(theta_orth), -std::sin(theta_orth),
		                                         std::sin(theta_orth), std::cos(theta_orth));
		const cv::Vec2d base_proj = shiftRotMat * base_x;

		const cv::Point2i shiftedCen(static_cast<int>(cen.x + base_proj[0] * std::sin(roll) * major / focal_length),
		        static_cast<int>(cen.y + base_proj[1] * std::sin(roll) * major / focal_length));
		assert(shiftedCen.x > 0);
		assert(shiftedCen.y > 0);

        return gridconfig_t { shiftedCen, radius, angles[2], angles[1], angles[0] };
	};

	return {getGrid(ellipse_roll), getGrid(-ellipse_roll)};
}

}
