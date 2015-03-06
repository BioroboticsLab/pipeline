/*
 * poly_line_vertical_iterator.cpp
 *
 *  Created on: Mar 4, 2015
 *      Author: tobias
 */

#include "poly_line_vertical_iterator.h"
#include <vector>                        // std::vector
#include <iostream>                      // std::cout
#include <algorithm>                     // std::equal
#include <stdexcept>                     // std::runtime_error

namespace heyho {

	namespace tests {

		void poly_line_vertical_iterator_tests()  {

			/**
			 * // poly line points --> ring_iterator --> poly_line_iterator --> points
			 */
			const auto line_2_points = [](std::vector<cv::Point> poly_points, bool leftmost) {
				using BDIT = std::vector<cv::Point>::const_iterator;
				using ring = ring_iterator<BDIT>;
				using poly_v_line = poly_line_vertical_iterator<ring>;

				poly_v_line pl(ring(poly_points.cbegin(), poly_points.cend(), poly_points.cbegin(), poly_points.cend() - 1, false), 8);
				std::vector<cv::Point> pl_points;
				for (; !pl.end(); ++pl) {
					pl_points.emplace_back(leftmost ? pl->left() : pl->right());
				}
				return pl_points;
			};

			const auto compare = [&line_2_points](std::vector<cv::Point> poly_points, std::vector<cv::Point> expected, bool leftmost) {
				const auto pts = line_2_points(poly_points, leftmost);
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

			std::cout << "poly line vertical iterator tests ... ";
			std::cout.flush();

			// 1 point
			//=========
			compare({{4,5}}, {{4,5}}, true);
			compare({{4,5}}, {{4,5}}, false);

			// 2 points
			// =========

			// horizontal line left to right
			compare({{1,1},{5,1}}, {{1,1}}, true);
			compare({{1,1},{5,1}}, {{5,1}}, false);

			// horizontal line right to left
			compare({{5,1},{1,1}}, {{1,1}}, true);
			compare({{5,1},{1,1}}, {{5,1}}, false);

			// 0° < slope < 45°
			compare({{1,0},{4,1}}, {{1,0},{3,1}}, true);
			compare({{1,0},{4,1}}, {{2,0},{4,1}}, false);

			// -45° < slope < 0°
			compare({{4,0},{1,1}}, {{3,0},{1,1}}, true);
			compare({{4,0},{1,1}}, {{4,0},{2,1}}, false);

			// vertical line
			compare({{4,0},{4,3}}, {{4,0},{4,1},{4,2},{4,3}}, true);
			compare({{4,0},{4,3}}, {{4,0},{4,1},{4,2},{4,3}}, false);

			// random line
			compare({{4,0},{6,0},{4,2},{6,4},{4,4}}, {{4,0},{5,1},{4,2},{5,3},{4,4}}, true);
			compare({{4,0},{6,0},{4,2},{6,4},{4,4}}, {{6,0},{5,1},{4,2},{5,3},{6,4}}, false);

			// "invalid" line --> skip points
			compare({{4,0},{6,0},{8,-10},{10,-1},{12,-2},{16,2}}, {{ 4,0},{15,1},{16,2}}, true);
			compare({{4,0},{6,0},{8,-10},{10,-1},{12,-2},{16,2}}, {{14,0},{15,1},{16,2}}, false);

			std::cout << "passed :)\n";
		}
	}

}


