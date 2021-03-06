/*
 * fill_convex_poly_cv.impl.h
 *
 *  Created on: Feb 23, 2015
 *      Author: tobias
 */

#ifndef FILL_CONVEX_POLY_CV_IMPL_H_
#define FILL_CONVEX_POLY_CV_IMPL_H_

namespace heyho {

	template<typename LINE_IT, typename F>
	F convex_poly_cv(F f, const cv::Size size, cv::InputArray points, const connectivity line_type)
	{
		constexpr int XY_SHIFT = 16;
		constexpr int XY_ONE   = 1 << XY_SHIFT;
		constexpr int DELTA    = XY_ONE >> 1;

		const auto ptr_size = cv_point_input_array_to_pointer(points);
		const cv::Point* const pts = ptr_size.first;
		const int npts = ptr_size.second;

		if (pts == nullptr || npts <= 0) {
			return std::move(f);
		}

		struct {
			int idx, di;
			int x, dx, ye;
		}
		edge[2];

		// draw outline, calc min/max x/y coordinates
		int imin = 0;
		int ymin = pts[0].y;
		int ymax = pts[0].y;
		{
			int xmin = pts[0].x;
			int xmax = pts[0].x;
			cv::Point p0 = pts[npts - 1];
			for (int i = 0; i < npts; i++ )
			{
				cv::Point p = pts[i];
				if( p.y < ymin )
				{
					ymin = p.y;
					imin = i;
				}

				ymax = std::max( ymax, p.y );
				xmax = std::max( xmax, p.x );
				xmin = std::min( xmin, p.x );

				f = heyho::line<LINE_IT>(std::move(f), size, p0, p, line_type, true);

				p0 = p;
			}

			if ( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height ) {
				return std::move(f);
			}
		}

		ymax = std::min( ymax, size.height - 1 );

		int y = ymin;

		edge[0].idx = imin;
		edge[1].idx = imin;
		edge[0].ye = y;
		edge[1].ye = y;
		edge[0].di = 1;
		edge[1].di = npts - 1;

		{
			int edges = npts;
			int left  = 0;
			int right = 1;
			do
			{
				for (int i = 0; i < 2; ++i )
				{
					if( y >= edge[i].ye )
					{
						int idx = edge[i].idx;
						int di  = edge[i].di;
						int xs = 0;
						int ty = 0;

						for (;;)
						{
							ty = pts[idx].y;
							if( ty > y || edges == 0 ) {
								break;
							}
							xs = pts[idx].x;
							idx += di;
							idx -= ((idx < npts) - 1) & npts;   /* idx -= idx >= npts ? npts : 0 */
							--edges;
						}

						const int ye = ty;
						xs <<= XY_SHIFT;
						const int xe = pts[idx].x << XY_SHIFT;

						/* no more edges */
						if ( y >= ye ) {
							return std::move(f);
						}

						edge[i].ye = ye;
						edge[i].dx = ((xe - xs) * 2 + (ye - y)) / (2 * (ye - y));
						edge[i].x = xs;
						edge[i].idx = idx;
					}
				}

				if ( edge[left].x > edge[right].x )
				{
					left ^= 1;
					right ^= 1;
				}

				int x1 = edge[left].x;
				int x2 = edge[right].x;

				if ( y >= 0 )
				{
					const int xx1 = (x1 + DELTA) >> XY_SHIFT;
					const int xx2 = (x2 + DELTA) >> XY_SHIFT;

					if ( xx2 >= 0 && xx1 < size.width )
					{
						f = heyho::hline(
								std::move(f),
								no_boundaries_tag{},
								std::max(xx1, 0),
								std::min(xx2, size.width - 1),
								y
						);
					}
				}

				x1 += edge[left].dx;
				x2 += edge[right].dx;

				edge[left].x  = x1;
				edge[right].x = x2;
			}
			while ( ++y <= ymax );
		}
		return std::move(f);
	}


	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly_cv(cv::InputOutputArray img, const cv::Scalar& color, cv::InputArray points, connectivity line_type)
	{
		/*
		 * cv::InputOutputArray --> cv::Mat; forward everything else
		 */
		cv::Mat img_mat = img.getMat();
		heyho::fill_convex_poly_cv<LINE_IT, pixel_t>(
			img_mat,
			color,
			points,
			line_type
		);
	}


	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly_cv(cv::Mat& img, const cv::Scalar& color, cv::InputArray points, connectivity line_type)
	{
		/*
		 * cv::Scalar --> pixel_t; forward everything else
		 */
		heyho::fill_convex_poly_cv<LINE_IT>(
			img,
			scalar2pixel<pixel_t>(color),
			points,
			line_type
		);
	}


	template<typename LINE_IT, typename pixel_t>
	void fill_convex_poly_cv(cv::Mat& img, const pixel_t &color, cv::InputArray points, connectivity line_type)
	{
		/*
		 * cv::Mat + pixel_t --> pixel_setter; forward everything else
		 */
		heyho::convex_poly_cv<LINE_IT>(
			pixel_setter<pixel_t>{img, color},
			img.size(),
			points,
			line_type
		);
	}

}


#endif /* FILL_CONVEX_POLY_CV_IMPL_H_ */
