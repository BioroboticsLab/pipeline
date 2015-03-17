/*
 * fill_convex_poly.impl.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_IMPL_H_
#define FILL_CONVEX_POLY_IMPL_H_

namespace heyho {

	template<typename IT, typename F, typename LINE_IT>
	inline F convex_poly(cv::Rect boundaries, IT begin, IT end, F f, int connectivity)
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
				std::move(f),
				boundaries,
				std::min(side_1->x_left,  side_2->x_left ),
				std::max(side_1->x_right, side_2->x_right),
				side_1->y
			);
		}
		return std::move(f);
	}

	template<typename IT, typename F, typename LINE_IT>
	inline F convex_poly(cv::Size size, IT begin, IT end, F f, int connectivity)
	{
		// cv::Size --> cv::Rect; forward everything else
		return convex_poly<IT, F, LINE_IT>(
			cv::Rect(cv::Point(0,0), size),
			std::move(begin),
			std::move(end),
			std::move(f),
			connectivity
		);
	}

	template<typename F, typename LINE_IT>
	inline F convex_poly(cv::Rect boundaries, cv::InputArray points, F f, int connectivity)
	{
		// cv::InputArray --> begin & end iterator; forward everything else
		const auto ptr_size = cv_point_input_array_to_pointer(points);
		return convex_poly<const cv::Point*, F, LINE_IT>(
			boundaries,
			ptr_size.first,
			ptr_size.first + ptr_size.second,
			std::move(f),
			connectivity
		);
	}

	template<typename F, typename LINE_IT>
	inline F convex_poly(cv::Size size, cv::InputArray points, F f, int connectivity)
	{
		// cv::Size --> cv::Rect; forward everything else
		return convex_poly<F, LINE_IT>(
			cv::Rect(cv::Point(0,0), size),
			points,
			std::move(f),
			connectivity
		);
	}


	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::InputOutputArray img, cv::InputArray points, const cv::Scalar& color, int line_type)
	{
		// cv::InputOutputArray --> cv::Mat; forward everything else
		cv::Mat img_mat = img.getMat();
		heyho::fill_convex_poly<pixel_t, LINE_IT>(
			img_mat,
			points,
			color,
			line_type
		);
	}

	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::Mat& img, cv::InputArray points, const cv::Scalar& color, int line_type)
	{
		// cv::Scalar --> pixel_t; forward everything else
		heyho::fill_convex_poly<pixel_t, LINE_IT>(
			img,
			points,
			scalar2pixel<pixel_t>(color),
			line_type
		);
	}

	template<typename pixel_t, typename LINE_IT>
	void fill_convex_poly(cv::Mat& img, cv::InputArray points, const pixel_t &color, int line_type)
	{
		// cv::Mat & pixel_t --> pixel_setter; forward everything else
		heyho::convex_poly<pixel_setter<pixel_t>, LINE_IT>(
			img.size(),
			points,
			pixel_setter<pixel_t>{img, color},
			line_type
		);
	}

}

#endif /* FILL_CONVEX_POLY_IMPL_H_ */
