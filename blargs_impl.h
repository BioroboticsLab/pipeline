/*
 * blargs_impl.h
 *
 *  Created on: Feb 23, 2015
 *      Author: tobias
 */

#ifndef BLARGS_IMPL_H_
#define BLARGS_IMPL_H_

namespace heyho {

template<typename F>
inline F Line(cv::Mat& img, cv::Point pt1, cv::Point pt2, F f, int connectivity)
{
	cv::LineIterator iterator(img, pt1, pt2, connectivity, true);
	const int count = iterator.count;
	for (int i = 0; i < count; ++i, ++iterator ) {
		f(iterator.pos());
	}
	return std::move(f);
}

template<typename pixel_t>
inline void DrawLine(cv::Mat& img, cv::Point pt1, cv::Point pt2, const pixel_t &color, int connectivity)
{
	heyho::Line(img, pt1, pt2, pixel_setter<pixel_t>{img, color}, connectivity);
}

template<typename F>
inline F hline(cv::Mat&, int x1, int x2, int y, F f)
{
	for(; x1 <= x2; ++x1) {
		f(y, x1);
	}
	return std::move(f);
}

template<typename pixel_t>
inline void drawhline(cv::Mat &img, int x1, int x2, int y, const pixel_t &color)
{
	heyho::hline(img, x1, x2, y, pixel_setter<pixel_t>{img, color});
}

template<typename pixel_t>
void fillConvexPoly(cv::InputOutputArray _img, cv::InputArray _points, const cv::Scalar& color, int line_type) {
	cv::Mat img = _img.getMat(), points = _points.getMat();
	CV_Assert(points.checkVector(2, CV_32S) >= 0);
	heyho::fillConvexPoly<pixel_t>(img, reinterpret_cast<const cv::Point*>(points.data), points.rows*points.cols*points.channels()/2, color, line_type);
}

template<typename pixel_t>
void fillConvexPoly(cv::Mat& img, const cv::Point* pts, int npts, const cv::Scalar& color, int line_type)
{
	if( !pts || npts <= 0 )
		return;

	if( line_type == CV_AA && img.depth() != CV_8U )
		line_type = 8;

	double buf[4];
	cv::scalarToRawData(color, buf, img.type(), 0);
	heyho::FillConvexPoly<pixel_t>(img, pts, npts, buf, line_type);
}

/**
 * helper function: filling horizontal row
 *
 * @param ptr         pointer to first byte of row
 * @param xl          index of first pixel
 * @param xr          index of last pixel
 * @param color       pointer to color buffer
 * @param pix_size    pixel size
 */
void inline ICV_HLINE(uchar* ptr, int xl, int xr, const void *color, int pix_size)
{
	const uchar* hline_max_ptr = ptr +  xr * pix_size;

	for(uchar *hline_ptr = ptr + xl * pix_size; hline_ptr <= hline_max_ptr; hline_ptr += pix_size)
	{
		for(int hline_j = 0; hline_j < pix_size; hline_j++ )
		{
			hline_ptr[hline_j] = static_cast<const uchar*>(color)[hline_j];
		}
	}
}

/**
 * converts raw pixel data into pixel.
 *
 * @param color result of "cv::scalarToRawData"
 */
template<typename pixel_t>
inline pixel_t color2pixel(const void *color) {
	pixel_t result;
	uchar *pixel_p = reinterpret_cast<uchar*>(&result);
	for (size_t i = 0; i < sizeof result; ++i) {
		pixel_p[i] = static_cast<const uchar*>(color)[i];
	}
	return result;
}



template<typename pixel_t>
void FillConvexPoly(cv::Mat& img, const cv::Point* v, int npts, const void* color, int line_type)
{

	if (line_type != 4 && line_type != 8) {
		throw std::invalid_argument("invalid line type");
	}

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

	struct {
		int idx, di;
		int x, dx, ye;
	}
	edge[2];

	const cv::Size size = img.size();
	const pixel_t color_pixel = color2pixel<pixel_t>(color);

	// draw outline, calc min/max x/y coordinates
	int imin = 0;
	int ymin = v[0].y;
	int ymax = ymin;
	{
		int xmin = v[0].x;
		int xmax = ymin;
		cv::Point p0 = v[npts - 1];
		for(int i = 0; i < npts; i++ )
		{
			cv::Point p = v[i];
			if( p.y < ymin )
			{
				ymin = p.y;
				imin = i;
			}

			ymax = std::max( ymax, p.y );
			xmax = std::max( xmax, p.x );
			xmin = std::min( xmin, p.x );

			//heyho::Line( img, p0, p, color, line_type );
			heyho::DrawLine(img, p0, p, color_pixel, line_type);

			p0 = p;
		}

		if( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height ) {
			return;
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
		uchar* ptr = img.data + img.step * y;
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
							ty = v[idx].y;
							if( ty > y || edges == 0 ) {
								break;
							}
							xs = v[idx].x;
							idx += di;
							idx -= ((idx < npts) - 1) & npts;   /* idx -= idx >= npts ? npts : 0 */
							--edges;
						}

						const int ye = ty;
						const int xe = v[idx].x;

						/* no more edges */
						if( y >= ye )
							return;

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
					heyho::drawhline(img, xx1, xx2, y, color_pixel);
				}
			}

			x1 += edge[left].dx;
			x2 += edge[right].dx;

			edge[left].x = x1;
			edge[right].x = x2;
			ptr += img.step;
		}
		while( ++y <= ymax );
	}
}

template<typename F>
F ConvexPoly(cv::Mat& img, const cv::Point* v, int npts, F f, int line_type)
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
	int ymin = v[0].y;
	int ymax = ymin;
	{
		int xmin = v[0].x;
		int xmax = ymin;
		cv::Point p0 = v[npts - 1];
		for(int i = 0; i < npts; i++ )
		{
			cv::Point p = v[i];
			if( p.y < ymin )
			{
				ymin = p.y;
				imin = i;
			}

			ymax = std::max( ymax, p.y );
			xmax = std::max( xmax, p.x );
			xmin = std::min( xmin, p.x );

			f = heyho::DrawLine(img, p0, p, std::move(f), line_type);

			p0 = p;
		}

		if( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height ) {
			return;
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
		uchar* ptr = img.data + img.step * y;
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
							ty = v[idx].y;
							if( ty > y || edges == 0 ) {
								break;
							}
							xs = v[idx].x;
							idx += di;
							idx -= ((idx < npts) - 1) & npts;   /* idx -= idx >= npts ? npts : 0 */
							--edges;
						}

						const int ye = ty;
						const int xe = v[idx].x;

						/* no more edges */
						if( y >= ye )
							return;

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
					f = heyho::drawhline(img, xx1, xx2, y, std::move(f));
				}
			}

			x1 += edge[left].dx;
			x2 += edge[right].dx;

			edge[left].x = x1;
			edge[right].x = x2;
			ptr += img.step;
		}
		while( ++y <= ymax );
	}
	return std::move(f);
}


}



#endif /* BLARGS_IMPL_H_ */
