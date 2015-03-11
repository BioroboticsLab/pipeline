/*
 * fill_convex_poly_impl.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_IMPL_H_
#define FILL_CONVEX_POLY_IMPL_H_

namespace heyho {

	template<typename IT, typename F, typename LINE_IT>
	F convex_poly(cv::Size size, IT begin, IT end, F f, int connectivity)
	{
		//using ring_it_t  = ring_iterator_bd<IT>;
		//using pixel_it_t = poly_line_vertical_iterator<ring_it_t>;

		using ring_fwd_it_t  = ring_iterator<IT, false>;
		using ring_rev_it_t  = ring_iterator<IT, true>;

		using pixel_fwd_it_t = poly_line_vertical_iterator<ring_fwd_it_t, LINE_IT>;
		using pixel_rev_it_t = poly_line_vertical_iterator<ring_rev_it_t, LINE_IT>;

		const auto min_max_y_it = std::minmax_element(begin, end, cv_point_less_y{});
		const auto min_y_it = min_max_y_it.first;
		const auto max_y_it = min_max_y_it.second;

		//pixel_it_t side_1(ring_it_t(begin, end, min_y_it, max_y_it, true),  connectivity);
		//pixel_it_t side_2(ring_it_t(begin, end, min_y_it, max_y_it, false), connectivity);

		pixel_fwd_it_t side_1(ring_fwd_it_t(begin, end, min_y_it, max_y_it), connectivity);
		pixel_rev_it_t side_2(ring_rev_it_t(begin, end, min_y_it, max_y_it), connectivity);

		for (; !side_1.end() && !side_2.end(); ++side_1, ++side_2)
		{
			f = hline(
				size,
				std::min(side_1->x_left,  side_2->x_left ),
				std::max(side_1->x_right, side_2->x_right),
				side_1->y,
				std::move(f)
			);
		}
		return std::move(f);
	}


	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::InputOutputArray _img, cv::InputArray _points, const cv::Scalar& color, int line_type) {
		cv::Mat img = _img.getMat();
		const cv::Mat points = _points.getMat();
		CV_Assert(points.checkVector(2, CV_32S) >= 0);
		const cv::Point *begin = reinterpret_cast<const cv::Point*>(points.data);
		const int size = points.rows * points.cols * points.channels() / 2;
		const cv::Point *end = begin + size;
		heyho::convex_poly<const cv::Point*, pixel_setter<pixel_t>, LINE_IT>(
			img.size(),
			begin,
			end,
			pixel_setter<pixel_t>{img, color},
			line_type
		);
	}

}

#endif /* FILL_CONVEX_POLY_IMPL_H_ */
