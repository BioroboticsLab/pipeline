/*
 * helper.h
 *
 *  Created on: Feb 26, 2015
 *      Author: tobias
 */

#ifndef HELPER_H_
#define HELPER_H_

#include <opencv2/opencv.hpp> // cv::Point, CV_MAT_DEPTH, CV_8U, ...
#include <string>             // std::string, std::to_string

namespace heyho {

	/**
	 * converts an opencv image type to the name of the corresponding macro i.e. "CV_\d[UFS]C\d"
	 */
	std::string img_type_to_str(int type);

	/**
	 * compares cv::Points based on their y coordinate
	 */
	struct cv_point_less_y {
		bool operator()(const cv::Point &lhs, const cv::Point &rhs) const {
			return lhs.y < rhs.y;
		}
	};

}


#endif /* HELPER_H_ */
