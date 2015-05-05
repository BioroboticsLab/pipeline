#pragma once

#include <opencv2/opencv.hpp>
#include <vector>

#include "datastructure/settings.h"

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

namespace pipeline {

class Tag;
class Ellipse;

class EllipseFitter {
private:
	settings::ellipsefitter_settings_t _settings;

	/**
	 * @brief detectXieEllipse
	 *
	 * see: Xie, Ji A New Efficient Ellipse Detection Method (2002) for details
	 * http://hci.iwr.uni-heidelberg.de/publications/dip/2002/ICPR2002/DATA/07_3_20.PDF
	 */
	void detectXieEllipse(Tag &tag);

	/**
	 * @brief detectEllipse
	 *
	 * see:
	 *  http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#findcontours
	 *  http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html#fitellipse
	 *  http://docs.opencv.org/doc/tutorials/imgproc/shapedescriptors/bounding_rotated_ellipses/bounding_rotated_ellipses.html
	 */
	void detectEllipse(Tag &tag);

#ifdef PipelineStandalone
	void visualizeEllipse(Ellipse const& ell, std::string const& title);
#endif

public:
	EllipseFitter() {}
	virtual ~EllipseFitter() {}

	void loadSettings(settings::ellipsefitter_settings_t &&settings);

	std::vector<Tag> process(std::vector<Tag> &&taglist);
	void visualizeEllipse(Tag const& tag , Ellipse const& ell, std::string const& title);
	int calcScore(Ellipse ell, cv::Mat canny);
	cv::Mat computeCannyEdgeMap(const cv::Mat &grayImage);
};
}
