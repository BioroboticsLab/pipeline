/*
 * line_iterator_test_helper.h
 *
 *  Created on: Mar 10, 2015
 *      Author: tobias
 */

#ifndef LINE_ITERATOR_TEST_HELPER_H_
#define LINE_ITERATOR_TEST_HELPER_H_

#include <vector>         // std::vector
#include <iostream>       // std::cout
#include <algorithm>      // std::equal
#include <stdexcept>      // std::runtime_error

namespace heyho {

	namespace tests {

		template<typename LINE_IT>
		std::vector<cv::Point> line_it_2_vec(LINE_IT it) {
			std::vector<cv::Point> result;
			for (; !it.end(); ++it) {
				result.emplace_back(*it);
			}
			return result;
		}

		template<typename LINE_IT>
		void line_iterator_test_helper()
		{
			const auto compare = [](LINE_IT it, std::vector<cv::Point> expected) {
				const auto pts = line_it_2_vec(it);
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

			// empty iterator
			//===============
			// 0°
			compare(LINE_IT{}, {});

			// no clipping (line inside image)
			//================================
			// 0°
			compare(LINE_IT({2,2}, {5,2}, 8, cv::Size(10, 10)), {{2,2},{3,2},{4,2},{5,2}});
			// 1st octant
			compare(LINE_IT({2,2}, {5,3}, 8, cv::Size(10, 10)), {{2,2},{3,2},{4,3},{5,3}});
			// 45°
			compare(LINE_IT({2,2}, {5,5}, 8, cv::Size(10, 10)), {{2,2},{3,3},{4,4},{5,5}});
			// 2nd octant
			compare(LINE_IT({2,2}, {3,5}, 8, cv::Size(10, 10)), {{2,2},{2,3},{3,4},{3,5}});
			// 90°
			compare(LINE_IT({2,2}, {2,5}, 8, cv::Size(10, 10)), {{2,2},{2,3},{2,4},{2,5}});
			// 3rd octant
			compare(LINE_IT({2,2}, {1,5}, 8, cv::Size(10, 10)), {{2,2},{2,3},{1,4},{1,5}});
			// 180°
			compare(LINE_IT({2,2}, {0,2}, 8, cv::Size(10, 10)), {{2,2},{1,2},{0,2}});
			// 270°
			compare(LINE_IT({2,2}, {2,0}, 8, cv::Size(10, 10)), {{2,2},{2,1},{2,0}});

			// clipping
			//==========
			// left
			compare(LINE_IT({0,2}, {2,2}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{1,2},{2,2}});
			// right
			compare(LINE_IT({1,2}, {3,2}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{1,2},{2,2}});
			// top
			compare(LINE_IT({2,1}, {2,3}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{2,1},{2,2}});
			// bottom
			compare(LINE_IT({2,0}, {2,2}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {{2,1},{2,2}});
			// line completely outside
			compare(LINE_IT({0,0}, {0,5}, 8, cv::Rect(cv::Point(1,1), cv::Point(3,3))), {});

			// no boundaries
			//===============
			compare(LINE_IT({-5,-5}, {-3,-3}, 8), {{-5,-5},{-4,-4},{-3,-3}});
			compare(LINE_IT({-1,1000}, {-1,1000}, 8), {{-1,1000}});

			// single point
			//=============
			compare(LINE_IT({-5,-5}, {-5,-5}, 8), {{-5,-5}});
		}
	}

}

#endif /* LINE_ITERATOR_TEST_HELPER_H_ */
