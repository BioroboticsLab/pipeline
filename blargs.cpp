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
        uchar* ptr = *iterator;
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

void Line2(cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* color)
{
    int ecount;
    int x, y;
    int x_step, y_step;
    int cb = static_cast<const uchar*>(color)[0];
    int cg = static_cast<const uchar*>(color)[1];
    int cr = static_cast<const uchar*>(color)[2];
    const int pix_size = static_cast<int>(img.elemSize());
    uchar *const ptr = img.data;
    uchar *tptr;
    const size_t step = img.step;
    const cv::Size size = img.size();
    const cv::Size sizeScaled(size.width*XY_ONE, size.height*XY_ONE);

    //assert( img && (nch == 1 || nch == 3) && img.depth() == CV_8U );

    if( !cv::clipLine( sizeScaled, pt1, pt2 ))
        return;

    int dx = pt2.x - pt1.x;
    int dy = pt2.y - pt1.y;

    const int j = dx < 0 ? -1 : 0;
    const int ax = (dx ^ j) - j;
    const int i = dy < 0 ? -1 : 0;
    const int ay = (dy ^ i) - i;

    if( ax > ay )
    {
        dx = ax;
        dy = (dy ^ j) - j;
        pt1.x ^= pt2.x & j;
        pt2.x ^= pt1.x & j;
        pt1.x ^= pt2.x & j;
        pt1.y ^= pt2.y & j;
        pt2.y ^= pt1.y & j;
        pt1.y ^= pt2.y & j;

        x_step = XY_ONE;
        y_step = static_cast<int>((static_cast<int64>(dy) << XY_SHIFT) / (ax | 1));
        ecount = (pt2.x - pt1.x) >> XY_SHIFT;
    }
    else
    {
        dy = ay;
        dx = (dx ^ i) - i;
        pt1.x ^= pt2.x & i;
        pt2.x ^= pt1.x & i;
        pt1.x ^= pt2.x & i;
        pt1.y ^= pt2.y & i;
        pt2.y ^= pt1.y & i;
        pt1.y ^= pt2.y & i;

        x_step = static_cast<int>((static_cast<int64>(dx) << XY_SHIFT) / (ay | 1));
        y_step = XY_ONE;
        ecount = (pt2.y - pt1.y) >> XY_SHIFT;
    }

    pt1.x += (XY_ONE >> 1);
    pt1.y += (XY_ONE >> 1);

    if( pix_size == 3 )
    {
        #define  ICV_PUT_POINT(_x,_y)   \
        x = (_x); y = (_y);             \
        if( 0 <= x && x < size.width && \
            0 <= y && y < size.height ) \
        {                               \
            tptr = ptr + y*step + x*3;  \
            tptr[0] = static_cast<uchar>(cb);        \
            tptr[1] = static_cast<uchar>(cg);        \
            tptr[2] = static_cast<uchar>(cr);        \
        }

        ICV_PUT_POINT((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT,
                      (pt2.y + (XY_ONE >> 1)) >> XY_SHIFT);

        if( ax > ay )
        {
            pt1.x >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT(pt1.x, pt1.y >> XY_SHIFT);
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT(pt1.x >> XY_SHIFT, pt1.y);
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

        #undef ICV_PUT_POINT
    }
    else if( pix_size == 1 )
    {
        #define  ICV_PUT_POINT(_x,_y) \
        x = (_x); y = (_y);           \
        if( 0 <= x && x < size.width && \
            0 <= y && y < size.height ) \
        {                           \
            tptr = ptr + y*step + x;\
            tptr[0] = static_cast<uchar>(cb);    \
        }

        ICV_PUT_POINT((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT,
                      (pt2.y + (XY_ONE >> 1)) >> XY_SHIFT);

        if( ax > ay )
        {
            pt1.x >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT(pt1.x, pt1.y >> XY_SHIFT);
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT(pt1.x >> XY_SHIFT, pt1.y);
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

        #undef ICV_PUT_POINT
    }
    else
    {
        #define  ICV_PUT_POINT(_x,_y)   \
        x = (_x); y = (_y);             \
        if( 0 <= x && x < size.width && \
            0 <= y && y < size.height ) \
        {                               \
            tptr = ptr + y*step + x*pix_size;\
            for(int j = 0; j < pix_size; j++ ) \
                tptr[j] = static_cast<const uchar*>(color)[j]; \
        }

        ICV_PUT_POINT((pt2.x + (XY_ONE >> 1)) >> XY_SHIFT,
                      (pt2.y + (XY_ONE >> 1)) >> XY_SHIFT);

        if( ax > ay )
        {
            pt1.x >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT(pt1.x, pt1.y >> XY_SHIFT);
                pt1.x++;
                pt1.y += y_step;
                ecount--;
            }
        }
        else
        {
            pt1.y >>= XY_SHIFT;

            while( ecount >= 0 )
            {
                ICV_PUT_POINT(pt1.x >> XY_SHIFT, pt1.y);
                pt1.x += x_step;
                pt1.y++;
                ecount--;
            }
        }

        #undef ICV_PUT_POINT
    }
}

/* Correction table depent on the slope */
static const uchar SlopeCorrTable[] = {
    181, 181, 181, 182, 182, 183, 184, 185, 187, 188, 190, 192, 194, 196, 198, 201,
    203, 206, 209, 211, 214, 218, 221, 224, 227, 231, 235, 238, 242, 246, 250, 254
};

/* Gaussian for antialiasing filter */
static const int FilterTable[] = {
    168, 177, 185, 194, 202, 210, 218, 224, 231, 236, 241, 246, 249, 252, 254, 254,
    254, 254, 252, 249, 246, 241, 236, 231, 224, 218, 210, 202, 194, 185, 177, 168,
    158, 149, 140, 131, 122, 114, 105, 97, 89, 82, 75, 68, 62, 56, 50, 45,
    40, 36, 32, 28, 25, 22, 19, 16, 14, 12, 11, 9, 8, 7, 5, 5
};

void LineAA(cv::Mat& img, cv::Point pt1, cv::Point pt2, const void* color)
{
    int ecount, scount = 0;
    int slope;
    int x_step, y_step;
    int ep_table[9];
    const int cb = static_cast<const uchar*>(color)[0], cg = static_cast<const uchar*>(color)[1], cr = static_cast<const uchar*>(color)[2];
    int _cb, _cg, _cr;
    const int nch = img.channels();
    uchar* ptr = img.data;
    size_t const step = img.step;
    cv::Size size = img.size();

    if( !((nch == 1 || nch == 3) && img.depth() == CV_8U) )
    {
    	heyho::Line(img, pt1, pt2, color);
        return;
    }

    pt1.x -= XY_ONE*2;
    pt1.y -= XY_ONE*2;
    pt2.x -= XY_ONE*2;
    pt2.y -= XY_ONE*2;
    ptr += img.step*2 + 2*nch;

    size.width = ((size.width - 5) << XY_SHIFT) + 1;
    size.height = ((size.height - 5) << XY_SHIFT) + 1;

    if( !clipLine( size, pt1, pt2 ))
        return;

    int dx = pt2.x - pt1.x;
    int dy = pt2.y - pt1.y;

    int j = dx < 0 ? -1 : 0;
    const int ax = (dx ^ j) - j;
    int i = dy < 0 ? -1 : 0;
    const int ay = (dy ^ i) - i;

    if( ax > ay )
    {
        dx = ax;
        dy = (dy ^ j) - j;
        pt1.x ^= pt2.x & j;
        pt2.x ^= pt1.x & j;
        pt1.x ^= pt2.x & j;
        pt1.y ^= pt2.y & j;
        pt2.y ^= pt1.y & j;
        pt1.y ^= pt2.y & j;

        x_step = XY_ONE;
        y_step = static_cast<int>((static_cast<int64>(dy) << XY_SHIFT) / (ax | 1));
        pt2.x += XY_ONE;
        ecount = (pt2.x >> XY_SHIFT) - (pt1.x >> XY_SHIFT);
        j = -(pt1.x & (XY_ONE - 1));
        pt1.y += static_cast<int>((static_cast<int64>(y_step) * j) >> XY_SHIFT) + (XY_ONE >> 1);
        slope = (y_step >> (XY_SHIFT - 5)) & 0x3f;
        slope ^= (y_step < 0 ? 0x3f : 0);

        /* Get 4-bit fractions for end-point adjustments */
        i = (pt1.x >> (XY_SHIFT - 7)) & 0x78;
        j = (pt2.x >> (XY_SHIFT - 7)) & 0x78;
    }
    else
    {
        dy = ay;
        dx = (dx ^ i) - i;
        pt1.x ^= pt2.x & i;
        pt2.x ^= pt1.x & i;
        pt1.x ^= pt2.x & i;
        pt1.y ^= pt2.y & i;
        pt2.y ^= pt1.y & i;
        pt1.y ^= pt2.y & i;

        x_step = static_cast<int>((static_cast<int64>(dx) << XY_SHIFT) / (ay | 1));
        y_step = XY_ONE;
        pt2.y += XY_ONE;
        ecount = (pt2.y >> XY_SHIFT) - (pt1.y >> XY_SHIFT);
        j = -(pt1.y & (XY_ONE - 1));
        pt1.x += static_cast<int>((static_cast<int64>(x_step) * j) >> XY_SHIFT) + (XY_ONE >> 1);
        slope = (x_step >> (XY_SHIFT - 5)) & 0x3f;
        slope ^= (x_step < 0 ? 0x3f : 0);

        /* Get 4-bit fractions for end-point adjustments */
        i = (pt1.y >> (XY_SHIFT - 7)) & 0x78;
        j = (pt2.y >> (XY_SHIFT - 7)) & 0x78;
    }

    slope = (slope & 0x20) ? 0x100 : SlopeCorrTable[slope];

    /* Calc end point correction table */
    {
        int t0 = slope << 7;
        int t1 = ((0x78 - i) | 4) * slope;
        int t2 = (j | 4) * slope;

        ep_table[0] = 0;
        ep_table[8] = slope;
        ep_table[1] = ep_table[3] = ((((j - i) & 0x78) | 4) * slope >> 8) & 0x1ff;
        ep_table[2] = (t1 >> 8) & 0x1ff;
        ep_table[4] = ((((j - i) + 0x80) | 4) * slope >> 8) & 0x1ff;
        ep_table[5] = ((t1 + t0) >> 8) & 0x1ff;
        ep_table[6] = (t2 >> 8) & 0x1ff;
        ep_table[7] = ((t2 + t0) >> 8) & 0x1ff;
    }

    if( nch == 3 )
    {
        #define  ICV_PUT_POINT()            \
        {                                   \
            _cb = tptr[0];                  \
            _cb += ((cb - _cb)*a + 127)>> 8;\
            _cg = tptr[1];                  \
            _cg += ((cg - _cg)*a + 127)>> 8;\
            _cr = tptr[2];                  \
            _cr += ((cr - _cr)*a + 127)>> 8;\
            tptr[0] = static_cast<uchar>(_cb);           \
            tptr[1] = static_cast<uchar>(_cg);           \
            tptr[2] = static_cast<uchar>(_cr);           \
        }
        if( ax > ay )
        {
            ptr += (pt1.x >> XY_SHIFT) * 3;

            while( ecount >= 0 )
            {
                uchar *tptr = ptr + ((pt1.y >> XY_SHIFT) - 1) * step;

                int ep_corr = ep_table[(((scount >= 2) + 1) & (scount | 2)) * 3 +
                                       (((ecount >= 2) + 1) & (ecount | 2))];
                int a, dist = (pt1.y >> (XY_SHIFT - 5)) & 31;

                a = (ep_corr * FilterTable[dist + 32] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr += step;
                a = (ep_corr * FilterTable[dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr += step;
                a = (ep_corr * FilterTable[63 - dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                pt1.y += y_step;
                ptr += 3;
                scount++;
                ecount--;
            }
        }
        else
        {
            ptr += (pt1.y >> XY_SHIFT) * step;

            while( ecount >= 0 )
            {
                uchar *tptr = ptr + ((pt1.x >> XY_SHIFT) - 1) * 3;

                int ep_corr = ep_table[(((scount >= 2) + 1) & (scount | 2)) * 3 +
                                       (((ecount >= 2) + 1) & (ecount | 2))];
                int a, dist = (pt1.x >> (XY_SHIFT - 5)) & 31;

                a = (ep_corr * FilterTable[dist + 32] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr += 3;
                a = (ep_corr * FilterTable[dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr += 3;
                a = (ep_corr * FilterTable[63 - dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                pt1.x += x_step;
                ptr += step;
                scount++;
                ecount--;
            }
        }
        #undef ICV_PUT_POINT
    }
    else
    {
        #define  ICV_PUT_POINT()            \
        {                                   \
            _cb = tptr[0];                  \
            _cb += ((cb - _cb)*a + 127)>> 8;\
            tptr[0] = static_cast<uchar>(_cb);           \
        }

        if( ax > ay )
        {
            ptr += (pt1.x >> XY_SHIFT);

            while( ecount >= 0 )
            {
                uchar *tptr = ptr + ((pt1.y >> XY_SHIFT) - 1) * step;

                int ep_corr = ep_table[(((scount >= 2) + 1) & (scount | 2)) * 3 +
                                       (((ecount >= 2) + 1) & (ecount | 2))];
                int a, dist = (pt1.y >> (XY_SHIFT - 5)) & 31;

                a = (ep_corr * FilterTable[dist + 32] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr += step;
                a = (ep_corr * FilterTable[dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr += step;
                a = (ep_corr * FilterTable[63 - dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                pt1.y += y_step;
                ptr++;
                scount++;
                ecount--;
            }
        }
        else
        {
            ptr += (pt1.y >> XY_SHIFT) * step;

            while( ecount >= 0 )
            {
                uchar *tptr = ptr + ((pt1.x >> XY_SHIFT) - 1);

                int ep_corr = ep_table[(((scount >= 2) + 1) & (scount | 2)) * 3 +
                                       (((ecount >= 2) + 1) & (ecount | 2))];
                int a, dist = (pt1.x >> (XY_SHIFT - 5)) & 31;

                a = (ep_corr * FilterTable[dist + 32] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr++;
                a = (ep_corr * FilterTable[dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                tptr++;
                a = (ep_corr * FilterTable[63 - dist] >> 8) & 0xff;
                ICV_PUT_POINT();
                ICV_PUT_POINT();

                pt1.x += x_step;
                ptr += step;
                scount++;
                ecount--;
            }
        }
        #undef ICV_PUT_POINT
    }
}

void fillConvexPoly(cv::InputOutputArray _img, cv::InputArray _points, const cv::Scalar& color, int line_type, int shift) {
	cv::Mat img = _img.getMat(), points = _points.getMat();
	CV_Assert(points.checkVector(2, CV_32S) >= 0);
	heyho::fillConvexPoly(img, reinterpret_cast<const cv::Point*>(points.data), points.rows*points.cols*points.channels()/2, color, line_type, shift);
}

void fillConvexPoly(cv::Mat& img, const cv::Point* pts, int npts, const cv::Scalar& color, int line_type, int shift)
{
    if( !pts || npts <= 0 )
        return;

//    constexpr int line_type = 8;  // lineType – Type of the polygon boundaries. See the line() description.
//                                  //    8 (or omitted) - 8-connected line.
//                                  //    4 - 4-connected line.
//                                  //    CV_AA - antialiased line.
//
//    constexpr int shift = 0;      // shift – Number of fractional bits in the vertex coordinates.
//                                  // shift – Number of fractional bits in the point  coordinates.

    if( line_type == CV_AA && img.depth() != CV_8U )
        line_type = 8;

    double buf[4];
    CV_Assert( 0 <= shift && shift <=  XY_SHIFT );
    cv::scalarToRawData(color, buf, img.type(), 0);
    heyho::FillConvexPoly(img, pts, npts, buf, line_type, shift);
}

/* helper macros: filling horizontal row */
#define ICV_HLINE( ptr, xl, xr, color, pix_size )            \
{                                                            \
    uchar* hline_ptr = static_cast<uchar*>(ptr) + (xl)*(pix_size);      \
    uchar* hline_max_ptr = static_cast<uchar*>(ptr) + (xr)*(pix_size);  \
                                                             \
    for( ; hline_ptr <= hline_max_ptr; hline_ptr += (pix_size))\
    {                                                        \
        int hline_j;                                         \
        for( hline_j = 0; hline_j < (pix_size); hline_j++ )  \
        {                                                    \
            hline_ptr[hline_j] = static_cast<const uchar*>(color)[hline_j];   \
        }                                                    \
    }                                                        \
}

void FillConvexPoly(cv::Mat& img, const cv::Point* v, int npts, const void* color, int line_type, int shift)
{
//    constexpr int line_type = 8;
//    constexpr int shift = 0;

    struct {
        int idx, di;
        int x, dx, ye;
    }
    edge[2];

    int delta = shift ? 1 << (shift - 1) : 0;
    int i, y, imin = 0, left = 0, right = 1, x1, x2;
    int edges = npts;
    int xmin, xmax, ymin, ymax;
    uchar* ptr = img.data;
    cv::Size size = img.size();
    int pix_size = static_cast<int>(img.elemSize());
    cv::Point p0;
    int delta1, delta2;

    if( line_type < CV_AA )
        delta1 = delta2 = XY_ONE >> 1;
    else
        delta1 = XY_ONE - 1, delta2 = 0;

    p0 = v[npts - 1];
    p0.x <<= XY_SHIFT - shift;
    p0.y <<= XY_SHIFT - shift;

    assert( 0 <= shift && shift <= XY_SHIFT );
    xmin = xmax = v[0].x;
    ymin = ymax = v[0].y;

    for( i = 0; i < npts; i++ )
    {
        cv::Point p = v[i];
        if( p.y < ymin )
        {
            ymin = p.y;
            imin = i;
        }

        ymax = std::max( ymax, p.y );
        xmax = std::max( xmax, p.x );
        xmin = MIN( xmin, p.x );

        p.x <<= XY_SHIFT - shift;
        p.y <<= XY_SHIFT - shift;

        if( line_type <= 8 )
        {
            if( shift == 0 )
            {
            	cv::Point pt0, pt1;
                pt0.x = p0.x >> XY_SHIFT;
                pt0.y = p0.y >> XY_SHIFT;
                pt1.x = p.x >> XY_SHIFT;
                pt1.y = p.y >> XY_SHIFT;
                heyho::Line( img, pt0, pt1, color, line_type );
            }
            else
            	heyho::Line2( img, p0, p, color );
        }
        else
        	heyho::LineAA( img, p0, p, color );
        p0 = p;
    }

    xmin = (xmin + delta) >> shift;
    xmax = (xmax + delta) >> shift;
    ymin = (ymin + delta) >> shift;
    ymax = (ymax + delta) >> shift;

    if( npts < 3 || xmax < 0 || ymax < 0 || xmin >= size.width || ymin >= size.height )
        return;

    ymax = MIN( ymax, size.height - 1 );
    edge[0].idx = edge[1].idx = imin;

    edge[0].ye = edge[1].ye = y = ymin;
    edge[0].di = 1;
    edge[1].di = npts - 1;

    ptr += img.step*y;

    do
    {
        if( line_type < CV_AA || y < ymax || y == ymin )
        {
            for( i = 0; i < 2; i++ )
            {
                if( y >= edge[i].ye )
                {
                    int idx = edge[i].idx, di = edge[i].di;
                    int xs = 0, xe, ye, ty = 0;

                    for(;;)
                    {
                        ty = (v[idx].y + delta) >> shift;
                        if( ty > y || edges == 0 )
                            break;
                        xs = v[idx].x;
                        idx += di;
                        idx -= ((idx < npts) - 1) & npts;   /* idx -= idx >= npts ? npts : 0 */
                        edges--;
                    }

                    ye = ty;
                    xs <<= XY_SHIFT - shift;
                    xe = v[idx].x << (XY_SHIFT - shift);

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

        x1 = edge[left].x;
        x2 = edge[right].x;

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
