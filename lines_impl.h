/*
 * lines_impl.h
 *
 *  Created on: Mar 2, 2015
 *      Author: tobias
 */

#ifndef LINES_IMPL_H_
#define LINES_IMPL_H_

namespace heyho {

	template<typename F>
	inline F hline(cv::Size size, int x1, int x2, int y, F f)
	{
		if (y >= 0 && y < size.height) {
			x1 = std::max(x1, 0);
			x2 = std::min(x2, size.width - 1);
			cv::Point p(x1, y);
			for(; p.x <= x2; ++p.x) {
				f(p);
			}
		}
		return std::move(f);
	}


	template<typename F>
	inline F line(cv::Size size, cv::Point pt1, cv::Point pt2, F f, int connectivity, bool left_to_right)
	{
		if (left_to_right && pt1.x > pt2.x) {
			using std::swap;
			swap(pt1, pt2);
		}
		for (heyho::line_iterator it(pt1, pt2, connectivity, size); !it.end(); ++it) {
			f(*it);
		}
		return std::move(f);
	}

}


#endif /* LINES_IMPL_H_ */
