/*
 * line_iterator_cv.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#include "../../../util/opencv_fill_poly/line_iterator_cv.h"
#include "../../../util/opencv_fill_poly/line_iterator_test_helper.h"

namespace heyho {

	namespace tests {

		void line_iterator_cv_tests() {

			std::cout << "line iterator cv tests ... ";
			std::cout.flush();
			line_iterator_test_helper<line_iterator_cv>();
			std::cout << "passed :)\n";
		}
	}

}
