/*
 * blargs_impl.h
 *
 *  Created on: Feb 23, 2015
 *      Author: tobias
 */

#ifndef BLARGS_IMPL_H_
#define BLARGS_IMPL_H_

namespace heyho {

/**
 * converts cv::Scalar into pixel.
 *
 * @param color result of "cv::scalarToRawData"
 */
template<typename pixel_t>
inline pixel_t scalar2pixel(const cv::Scalar &color) {
	constexpr int img_type = cv::DataType<pixel_t>::type;
	double buf[4];
	cv::scalarToRawData(color, buf, img_type, 0);
	pixel_t result;
	uchar *pixel_p = reinterpret_cast<uchar*>(&result);
	for (size_t i = 0; i < sizeof result; ++i) {
		pixel_p[i] = reinterpret_cast<const uchar*>(buf)[i];
	}
	return result;
}


template<typename F>
inline F hline(cv::Mat&, int x1, int x2, int y, F f)
{
	for(; x1 <= x2; ++x1) {
		f(y, x1);
	}
	return std::move(f);
}


template<typename F>
inline F line(cv::Mat& img, cv::Point pt1, cv::Point pt2, F f, int connectivity)
{
	cv::LineIterator iterator(img, pt1, pt2, connectivity, true);
	const int count = iterator.count;
	for (int i = 0; i < count; ++i, ++iterator ) {
		f(iterator.pos());
	}
	return std::move(f);
}


template<typename F>
F convex_poly(cv::Mat& img, const cv::Point* pts, int npts, F f, int line_type)
{

	if (line_type != 4 && line_type != 8) {
		throw std::invalid_argument("invalid line type");
	}

	struct {
		int idx, di;
		int x, dx, ye;
	}
	edge[2];

	const cv::Size size = img.size();

	// draw outline, calc min/max x/y coordinates
	int imin = 0;
	int ymin = pts[0].y;
	int ymax = ymin;
	{
		int xmin = pts[0].x;
		int xmax = ymin;
		cv::Point p0 = pts[npts - 1];
		for(int i = 0; i < npts; i++ )
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

			f = heyho::line(img, p0, p, std::move(f), line_type);

			p0 = p;
		}

		if( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height ) {
			return f;
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
			if( line_type < CV_AA || y < ymax || y == ymin )
			{
				for(int i = 0; i < 2; ++i )
				{
					if( y >= edge[i].ye )
					{
						int idx = edge[i].idx;
						int di  = edge[i].di;
						int xs = 0;
						int ty = 0;

						for(;;)
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
						const int xe = pts[idx].x;

						/* no more edges */
						if( y >= ye )
							return f;

						edge[i].ye = ye;
						edge[i].dx = ((xe - xs) * 2 + (ye - y)) / (2 * (ye - y));
						edge[i].x = xs;
						edge[i].idx = idx;
					}
				}
			}

			if( edge[left].x > edge[right].x )
			{
				left ^= 1;
				right ^= 1;
			}

			int x1 = edge[left].x;
			int x2 = edge[right].x;

			if( y >= 0 )
			{
				int xx1 = x1;
				int xx2 = x2;

				if( xx2 >= 0 && xx1 < size.width )
				{
					if( xx1 < 0 )
						xx1 = 0;
					if( xx2 >= size.width )
						xx2 = size.width - 1;
					f = heyho::hline(img, xx1, xx2, y, std::move(f));
				}
			}

			x1 += edge[left].dx;
			x2 += edge[right].dx;

			edge[left].x = x1;
			edge[right].x = x2;
		}
		while( ++y <= ymax );
	}
	return std::move(f);
}


template<typename pixel_t>
void fill_convex_poly(cv::InputOutputArray _img, cv::InputArray _points, const cv::Scalar& color, int line_type) {
	cv::Mat img = _img.getMat(), points = _points.getMat();
	CV_Assert(points.checkVector(2, CV_32S) >= 0);
	heyho::fill_convex_poly<pixel_t>(img, reinterpret_cast<const cv::Point*>(points.data), points.rows*points.cols*points.channels()/2, color, line_type);
}


template<typename pixel_t>
void fill_convex_poly(cv::Mat& img, const cv::Point* pts, int npts, const cv::Scalar& color, int line_type)
{
	if( !pts || npts <= 0 )
		return;

	if( line_type == CV_AA && img.depth() != CV_8U )
		line_type = 8;

	heyho::fill_convex_poly(img, pts, npts, scalar2pixel<pixel_t>(color), line_type);
}


template<typename pixel_t>
void fill_convex_poly(cv::Mat& img, const cv::Point* v, int npts, const pixel_t &color, int line_type)
{
	constexpr int img_type = cv::DataType<pixel_t>::type;
	if (img.type() != img_type) {
		throw std::invalid_argument("invalid image pixel type");
	}

	static_assert(
			sizeof(typename cv::DataType<pixel_t>::value_type)
			==
			sizeof(typename cv::DataType<pixel_t>::channel_type) * cv::DataType<pixel_t>::channels
			, "unexpected pixel size"
	);

	constexpr int pix_size = sizeof(typename cv::DataType<pixel_t>::value_type);
	if (pix_size != static_cast<int>(img.elemSize())) {
		throw std::invalid_argument("invalid image pixel size");
	}

	heyho::convex_poly(img, v, npts, pixel_setter<pixel_t>{img, color}, line_type);

	/**
	 * size_t Mat::elemSize()  const --> num_chans * sizeof(T)
	 * size_t Mat::elemSize1() const -->             sizeof(T)
	 *
	 * int Mat::channels() const --> num_chans
	 *
	 * int Mat::type() const --> CV_16SC3
	 *
	 * int Mat::depth() const --> MACRO / enum fuer {signed, unsigned}{char, short, int} ...
	 *
	 *
	 * CV_MAT_CN(CV_16SC3) --> num_chans
	 *
	 */
}

}



#endif /* BLARGS_IMPL_H_ */
