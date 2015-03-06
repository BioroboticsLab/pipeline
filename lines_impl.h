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
	inline F hline(int x_left, int x_right, int y, F f)
	{
		for(; x_left <= x_right; ++x_left) {
			f(cv::Point(x_left, y));
		}
		return std::move(f);
	}

	template<typename F>
	inline F hline(cv::Size size, int x_left, int x_right, int y, F f)
	{
		if (y >= 0 && y < size.height)
		{
			return hline(std::max(x_left, 0), std::min(x_right, size.width - 1), y, std::move(f));
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
		for (heyho::line_iterator_cv it(pt1, pt2, connectivity, size); !it.end(); ++it) {
			f(*it);
		}
		return std::move(f);
	}

}


#endif /* LINES_IMPL_H_ */
