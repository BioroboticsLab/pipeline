/*
 * blargs.cpp
 *
 *  Created on: Feb 20, 2015
 *      Author: tobias
 */

#include "blargs.h"
#include <cstring>

namespace heyho {

constexpr int XY_SHIFT = 8;
constexpr int XY_ONE   = 1 << XY_SHIFT;

void Line(cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* _color, int connectivity)
{
    if( connectivity == 0 )
        connectivity = 8;
    if( connectivity == 1 )
        connectivity = 4;

    cv::LineIterator iterator(img, pt1, pt2, connectivity, true);
    const int count = iterator.count;
    const int pix_size = static_cast<int>(img.elemSize());
    const uchar* const color = static_cast<const uchar*>(_color);

    for(int i = 0; i < count; i++, ++iterator )
    {
        uchar* const ptr = *iterator;
        if( pix_size == 1 )
            ptr[0] = color[0];
        else if( pix_size == 3 )
        {
            ptr[0] = color[0];
            ptr[1] = color[1];
            ptr[2] = color[2];
        }
        else
            std::memcpy( *iterator, color, pix_size );
    }
}

void fillConvexPoly(cv::InputOutputArray _img, cv::InputArray _points, const cv::Scalar& color, int line_type) {
	cv::Mat img = _img.getMat(), points = _points.getMat();
	CV_Assert(points.checkVector(2, CV_32S) >= 0);
	heyho::fillConvexPoly(img, reinterpret_cast<const cv::Point*>(points.data), points.rows*points.cols*points.channels()/2, color, line_type);
}

void fillConvexPoly(cv::Mat& img, const cv::Point* pts, int npts, const cv::Scalar& color, int line_type)
{
    if( !pts || npts <= 0 )
        return;

//    constexpr int line_type = 8;  // lineType – Type of the polygon boundaries. See the line() description.
//                                  //    8 (or omitted) - 8-connected line.
//                                  //    4 - 4-connected line.
//                                  //    CV_AA - antialiased line.
//
    if( line_type == CV_AA && img.depth() != CV_8U )
        line_type = 8;

    double buf[4];
    cv::scalarToRawData(color, buf, img.type(), 0);
    heyho::FillConvexPoly(img, pts, npts, buf, line_type);
}

/* helper function: filling horizontal row */
static void inline ICV_HLINE(uchar* ptr, int xl, int xr, const void *color, int pix_size)
{
	const uchar* hline_max_ptr = ptr +  xr * pix_size;

	for(uchar *hline_ptr = ptr + xl * pix_size; hline_ptr <= hline_max_ptr; hline_ptr += pix_size)
	{
		for(int hline_j = 0; hline_j < (pix_size); hline_j++ )
		{
			hline_ptr[hline_j] = static_cast<const uchar*>(color)[hline_j];
		}
	}
}

void FillConvexPoly(cv::Mat& img, const cv::Point* v, int npts, const void* color, int line_type)
{
	if (line_type != 4 && line_type != 8) {
		throw std::invalid_argument("invalid line type");
	}

    struct {
        int idx, di;
        int x, dx, ye;
    }
    edge[2];

    int imin = 0, left = 0, right = 1;
    int edges = npts;
    int xmin, xmax, ymin, ymax;
    uchar* ptr = img.data;
    const cv::Size size = img.size();
    const int pix_size = static_cast<int>(img.elemSize());

    constexpr int delta1 = XY_ONE >> 1;
    constexpr int delta2 = delta1;

    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    {
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

			heyho::Line( img, p0, p, color, line_type );

			p0 = p;
		}
    }

    if( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height )
        return;

    ymax = std::min( ymax, size.height - 1 );
    edge[0].idx = edge[1].idx = imin;

    int y = ymin;
    edge[0].ye = edge[1].ye = y;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    ptr += img.step*y;

    do
    {
        if( line_type < CV_AA || y < ymax || y == ymin )
        {
            for(int i = 0; i < 2; i++ )
            {
                if( y >= edge[i].ye )
                {
                    int idx = edge[i].idx, di = edge[i].di;
                    int xs = 0, xe, ye, ty = 0;

                    for(;;)
                    {
                        ty = v[idx].y;
                        if( ty > y || edges == 0 )
                            break;
                        xs = v[idx].x;
                        idx += di;
                        idx -= ((idx < npts) - 1) & npts;   /* idx -= idx >= npts ? npts : 0 */
                        edges--;
                    }

                    ye = ty;
                    xs <<= XY_SHIFT;
                    xe = v[idx].x << (XY_SHIFT);

                    /* no more edges */
                    if( y >= ye )
                        return;

                    edge[i].ye = ye;
                    edge[i].dx = ((xe - xs)*2 + (ye - y)) / (2 * (ye - y));
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
            int xx1 = (x1 + delta1) >> XY_SHIFT;
            int xx2 = (x2 + delta2) >> XY_SHIFT;

            if( xx2 >= 0 && xx1 < size.width )
            {
                if( xx1 < 0 )
                    xx1 = 0;
                if( xx2 >= size.width )
                    xx2 = size.width - 1;
                ICV_HLINE( ptr, xx1, xx2, color, pix_size );
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
