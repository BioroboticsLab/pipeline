/*
 * line_iterator_cv.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#include <vector>         // std::vector
#include <iostream>       // std::cout
#include <algorithm>      // std::equal
#include <stdexcept>      // std::runtime_error
#include "line_iterator_cv.h"

namespace heyho {

	namespace tests {

		void line_iterator_tests() {

			const auto it_2_vec = [](line_iterator it) {
				std::vector<cv::Point> result;
				for (; !it.end(); ++it) {
					result.emplace_back(*it);
				}
				return result;
			};

			const auto compare = [&it_2_vec](line_iterator it, std::vector<cv::Point> expected) {
				const auto pts = it_2_vec(it);
				if (pts.size() != expected.size() || !std::equal(pts.cbegin(), pts.cend(), expected.cbegin())) {
					std::cout << "expected:";
					for (const auto p : expected) {
						std::cout << " " << p;
					}
					std::cout << "\n";
					std::cout << "got:";
					for (const auto p : pts) {
						std::cout << " " << p;
					}
					std::cout << "\n";
					throw std::runtime_error(":(");
				}
			};

			std::cout << "line iterator tests ... ";
			std::cout.flush();

			// empty iterator
			//===============
			// 0°
			compare(line_iterator{}, {});

			// no clipping (line inside image)
			//================================
			// 0°
			compare(line_iterator({2,2}, {5,2}, 8, cv::Size(10, 10)), {{2,2},{3,2},{4,2},{5,2}});
			// 1st octant
			compare(line_iterator({2,2}, {5,3}, 8, cv::Size(10, 10)), {{2,2},{3,2},{4,3},{5,3}});
			// 45°
			compare(line_iterator({2,2}, {5,5}, 8, cv::Size(10, 10)), {{2,2},{3,3},{4,4},{5,5}});
			// 2nd octant
			compare(line_iterator({2,2}, {3,5}, 8, cv::Size(10, 10)), {{2,2},{2,3},{3,4},{3,5}});
			// 90°
			compare(line_iterator({2,2}, {2,5}, 8, cv::Size(10, 10)), {{2,2},{2,3},{2,4},{2,5}});
			// 3rd octant
			compare(line_iterator({2,2}, {1,5}, 8, cv::Size(10, 10)), {{2,2},{2,3},{1,4},{1,5}});
			// 180°
			compare(line_iterator({2,2}, {0,2}, 8, cv::Size(10, 10)), {{2,2},{1,2},{0,2}});
			// 270°
			// 0°
			compare(line_iterator({2,2}, {2,0}, 8, cv::Size(10, 10)), {{2,2},{2,1},{2,0}});

			// clipping
			//==========
			// left
			compare(line_iterator({0,2}, {2,2}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{1,2},{2,2}});
			// right
			compare(line_iterator({1,2}, {3,2}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{1,2},{2,2}});
			// top
			compare(line_iterator({2,1}, {2,3}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{2,1},{2,2}});
			// bottom
			compare(line_iterator({2,0}, {2,2}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{2,1},{2,2}});

			// no boundariers
			//===============
			compare(line_iterator({-5,-5}, {-3,-3}, 8), {{-5,-5},{-4,-4},{-3,-3}});
			compare(line_iterator({-1,1000}, {-1,1000}, 8), {{-1,1000}});

			// single point
			//=============
			compare(line_iterator({-5,-5}, {-5,-5}, 8), {{-5,-5}});

			std::cout << "passed :)\n";
		}
	}

}
