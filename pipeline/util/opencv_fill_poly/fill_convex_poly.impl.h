/*
 * fill_convex_poly.impl.h
 *
 *  Created on: Mar 6, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_IMPL_H_
#define FILL_CONVEX_POLY_IMPL_H_

namespace heyho {

	template<typename LINE_IT, typename F, typename B, typename IT>
	F convex_poly(F f, B boundaries, IT begin, IT end, connectivity line_type)
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

		pixel_fwd_it_t side_1(ring_fwd_it_t(begin, end, min_y_it, max_y_it), line_type);
		pixel_rev_it_t side_2(ring_rev_it_t(begin, end, min_y_it, max_y_it), line_type);

		for (; !side_1.end() && !side_2.end(); ++side_1, ++side_2)
		{
			f = hline<>(
				std::move(f),
				boundaries,
				std::min(side_1->x_left,  side_2->x_left ),
				std::max(side_1->x_right, side_2->x_right),
				side_1->y
			);
		}
		return std::move(f);
	}

	template<typename LINE_IT, typename F, typename B>
	inline F convex_poly(F f, B boundaries, cv::InputArray points, connectivity line_type)
	{
		// cv::InputArray --> begin & end iterator; forward everything else
		const auto ptr_size = cv_point_input_array_to_pointer(points);
		return convex_poly<LINE_IT>(
			std::move(f),
			boundaries,
			ptr_size.first,
			ptr_size.first + ptr_size.second,
			line_type
		);
	}


	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly(cv::InputOutputArray img, const cv::Scalar& color, cv::InputArray points, connectivity line_type)
	{
		// cv::InputOutputArray --> cv::Mat; forward everything else
		cv::Mat img_mat = img.getMat();
		heyho::fill_convex_poly<LINE_IT, pixel_t>(
			img_mat,
			color,
			points,
			line_type
		);
	}

	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly(cv::Mat& img, const cv::Scalar& color, cv::InputArray points, connectivity line_type)
	{
		// cv::Scalar --> pixel_t; forward everything else
		heyho::fill_convex_poly<LINE_IT>(
			img,
			scalar2pixel<pixel_t>(color),
			points,
			line_type
		);
	}

	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly(cv::Mat& img, const pixel_t &color, cv::InputArray points, connectivity line_type)
	{
		// cv::Mat & pixel_t --> pixel_setter; forward everything else
		heyho::convex_poly<LINE_IT>(
			pixel_setter<pixel_t>{img, color},
			img.size(),
			points,
			line_type
		);
	}

}

#endif /* FILL_CONVEX_POLY_IMPL_H_ */
