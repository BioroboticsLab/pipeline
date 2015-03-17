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
	inline F hline(F f, no_boundaries_tag, int x_left, int x_right, int y)
	{
		for(; x_left <= x_right; ++x_left) {
			f(cv::Point(x_left, y));
		}
		return std::move(f);
	}

	template<typename F>
	inline F hline(F f, cv::Size boundaries, int x_left, int x_right, int y)
	{
		if (y >= 0 && y < boundaries.height)
		{
			return hline(
				std::move(f),
				no_boundaries_tag{},
				std::max(x_left,  0),
				std::min(x_right, boundaries.width - 1),
				y
			);
		}
		return std::move(f);
	}

	template<typename F>
	inline F hline(F f, cv::Rect boundaries, int x_left, int x_right, int y) {
		if (y >= boundaries.tl().y && y < boundaries.br().y)
		{
			return hline(
				std::move(f),
				no_boundaries_tag{},
				std::max(x_left,  boundaries.tl().x    ),
				std::min(x_right, boundaries.br().x - 1),
				y
			);
		}
		return std::move(f);
	}


	template<typename F, typename LINE_IT>
	inline F line(cv::Size size, cv::Point pt1, cv::Point pt2, F f, int connectivity, bool left_to_right)
	{
		if (left_to_right && pt1.x > pt2.x) {
			using std::swap;
			swap(pt1, pt2);
		}
		for (LINE_IT it(size, pt1, pt2, connectivity); !it.end(); ++it) {
			f(*it);
		}
		return std::move(f);
	}

}


#endif /* LINES_IMPL_H_ */
