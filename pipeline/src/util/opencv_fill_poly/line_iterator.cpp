/*
 * line_iterator.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#include "../../../util/opencv_fill_poly/line_iterator_cv.h"
#include "../../../util/opencv_fill_poly/line_iterator.h"
#include "../../../util/opencv_fill_poly/line_iterator_test_helper.h"
#include "../../../util/opencv_fill_poly/test_helper.h"                // container_compare

namespace heyho {

	namespace tests {

		void line_iterator_tests()
		{
			const auto compare = [](line_iterator it, line_iterator_cv expected) {
				container_compare(
					line_it_2_vec(expected),
					line_it_2_vec(it)
				);
			};

			std::cout << "line iterator tests ... ";
			std::cout.flush();

			// compare line_iterator with line_iterator_cv : test every unclipped line
			for(int x = -10; x <= 10; ++x) {
				for (int y = -10; y <= 10; ++y) {
					const cv::Size size(50,50);
					const cv::Point p1(25, 25);
					const cv::Point vec(x,y);
					compare(
						line_iterator   (no_boundaries_tag{}, p1, p1 + vec, connectivity::eight_connected),
						line_iterator_cv(size,                p1, p1 + vec, connectivity::eight_connected)
					);

					compare(
						line_iterator   (no_boundaries_tag{}, p1, p1 + vec, connectivity::four_connected),
						line_iterator_cv(size,                p1, p1 + vec, connectivity::four_connected)
					);
				}
			}

			// line iterator tests : clipped lines etc
			line_iterator_test_helper<line_iterator>();

			std::cout << "passed :)\n";
		}

	}


}


