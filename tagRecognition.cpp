#include "tagRecognition.h"

using namespace cv;
using namespace std;


//forward declarations
vector <Ellipse> detectXieEllipse(Mat &ec,  int minmajor,  int maxmajor,  int minminor,  int maxminor,  int threshold);
void ellipseTransform(Mat &roi, Ellipse &ell);
double getOtsuThreshold(Mat &srcMat, Mat &mask);

/*
	Detect and mark the ellipse in the given ROI
	returns whether an ellipse was detected
*/
bool tagRecognition( Mat & cannyEdgeMap, Rect &boundingBox, Mat &image, Ellipse &currentEllipse, Mat &subroi, Mat &ellcall, Mat &ellcallt , vector<Ellipse> & ellipseCandidate) {
	Mat roi = Mat( cannyEdgeMap, boundingBox );
//cout << "\tDetect Ellipse ... ";
	ellipseCandidate = detectXieEllipse( roi, 42, 54, 30, 54, 25 );

//cout << " Done!" << endl;

	if ( ellipseCandidate.size( ) == 0 ) {
		#ifdef _DEBUG
		Mat output;
		cannyEdgeMap.copyTo( output );
		rectangle( output, boundingBox, Scalar( 255, 0, 0 ), 1 );
		line( output, Point( boundingBox.x +1, boundingBox.y +1 ), Point( boundingBox.x + boundingBox.width -2, boundingBox.y + boundingBox.height -2 ), Scalar( 0, 0, 255 ));
		#endif
		DEBUG_IMSHOW("output", output);
		return false;
	}
	else {
		currentEllipse = ellipseCandidate[ellipseCandidate.size() -1 ];
		// refine the boundingBox (coords absolute in image) and the subroi according to the size of the fitted ellipse (coords relative to the boundingBox)
		// we make the roi a bit bigger to be sure not to cut off any important details
		int newWidth = currentEllipse.getAxis().width;  // note: this value is only the "radius", hence half of the actual axis size
		int roiPadding = 10; // size of the border around the actual ellipse
		Point2i oldTopLeft = boundingBox.tl();
		// compute top left corner of the rect from ellipse.cen and the size; Note that the center of the ellipse is only relative to the old boundingBox, hence the adding of the old top left corner
		// to achieve absolute coordinates for the boundingBox
		Point2i newTopLeft = Point( currentEllipse.cen.x - newWidth - roiPadding, currentEllipse.cen.y - newWidth - roiPadding ) + oldTopLeft;
		Point2i newBottomRight = newTopLeft + Point( newWidth *2 + roiPadding *2, newWidth *2 + roiPadding *2 );
		boundingBox = Rect( newTopLeft, newBottomRight ); // new BoundingBox' size is quadratic
		subroi = Mat( image, boundingBox );  // assign subroi using the refined boundingBox
		// adapt the ellipse centrum to the new (smaller) roi; the former ellipse centrum was relative to oldTopLeft, now it must be relative to newTopLeft
		currentEllipse.cen = Point( newWidth + roiPadding, newWidth + roiPadding );

	#ifdef _DEBUG
		Mat subroiTest;
		subroi.copyTo( subroiTest );
		ellipse(subroiTest, currentEllipse.cen, currentEllipse.axis, currentEllipse.angle, 0, 360, Scalar(0,0,255));
		namedWindow("refinedROI / subroi", WINDOW_NORMAL);
		imshow( "refinedROI / subroi", subroiTest );
		waitKey();


		subroi.copyTo( ellcall );
		subroi.copyTo( ellcallt );
		cv::ellipse( ellcallt, currentEllipse.cen, currentEllipse.axis, currentEllipse.angle, 0, 360, Scalar( 0, 0, 255 ));
	#endif
		DEBUG_NAMED_WINDOW("Ellipse");
		//DEBUG_SET_MOUSE_CALLBACK("Ellipse", EllipseCallback, 0);
		DEBUG_IMSHOW("Ellipse", ellcallt);
		return (true);
	}
}

void ellipseTransformation( Mat &subroi, Ellipse &ellipse, Mat &elMask ) {
		subroi.copyTo( elMask );
		ellipseTransform( elMask, ellipse );
}


/**
 * Calculate the euclidean distance between two points
 *
 * \param p 2D Point
 * \param q 2D Point
 */
double pointDistance(Point2d p, Point2d q) {
	return (sqrt((q.y - p.y) * (q.y - p.y) + (q.x - p.x) * (q.x - p.x)));
}

/**
 * Xie's ellipse detection
 * Publication: http://csce.uark.edu/~jgauch/library/Segmentation/Xie.2002.pdf
 *
 * The proposed algorithm is changed to be exhaustive and return only the best ellipse.
 *
 * \param ec the region of interest's (binary) edge map
 * \param minmajor major axis' minimum length
 * \param maxmajor major axis' maximum length
 * \param minminor minor axis' minimum length
 * \param threshold minimum number of edge pixels required to support an ellipse
 */
vector <Ellipse> detectXieEllipse(Mat &ec,  int minmajor, int maxmajor,  int minminor,  int maxminor, int threshold) {
	//std::cout << "Start ..." << std::endl;

	//edge_pixel array, all edge pixels are stored in this array
	vector<Point2i> ep;
	ep.reserve(cv::countNonZero(ec)+1);

	vector<Ellipse>::iterator cand;
	 int max_ind;
	//Ellipse elli;
	double a, b, tau, d, f, alpha, costau, sintau;
	// a - half length of the ellipse' major axis
	// b - half length of the ellipse' minor axis
	// alpha - ellipse' orientation

//	vector<vector<Point2i> > support;
//	for ( i = 0; i < maxminor / 2 + 1; i++) {
//		support.push_back(vector<Point2i>());
//	}

	Point2d p, p1, p2, center;
	vector< Ellipse > candidates;

	// (1) all white (being edge) pixels are written into the ep array
	MatIterator_< char> mit, end;
	for (mit = ec.begin< char>(), end = ec.end< char>();
			mit != end; mit++) {
		if (*mit.ptr == 255) {
			ep.push_back(mit.pos());
		}
	}

	// (2) initiate the accumulator array
	std::vector< int> accu((maxminor) / 2 + 1);
	 int vote_minor;

	// (3) loop through each edge pixel in ep
	for (std::vector<Point2i>::iterator it = ep.begin(); it != ep.end(); it++) {
		p1.x = (*it).x;
		p1.y = (*it).y;

		// (4) loop through each other edge pixel in ep and propose a major axis between p1 and p2
		for (std::vector<Point2i>::iterator it2 = ep.begin(); it2 != ep.end();
				it2++) {
			if ((it2 > it) && (*it) != (*it2)) {
				p2.x = (*it2).x;
				p2.y = (*it2).y;
				// the proposed ellipse' length of the major axis lies between minmajor and maxmajor
				if (pointDistance(p1, p2) > minmajor
						&& pointDistance(p1, p2) < maxmajor) {
					// (5) calculate the ellipse' center, half length of the major axis (a) and orientation (alpha) based on p1 and p2
					center.x = (p1.x + p2.x) / 2;
					center.y = (p1.y + p2.y) / 2;
					a = pointDistance(p1, p2) / 2;
					alpha = atan2((p2.y - p1.y) , (p2.x - p1.x));
					// (6) loop through each third edge pixel eventually lying on the ellipse
					for (std::vector<Point2i>::iterator it3 = ep.begin();
							it3 != ep.end(); it3++) {

						if ((*it3) != (*it2) && (*it3) != (*it)) {
							p.x = (*it3).x;
							p.y = (*it3).y;
							d = pointDistance(p, center);

							if (d > minminor / 2.0 && d <= a) {
								// (7) estimate the half length of the minor axis (b)
								f = pointDistance(p, p2);
								costau = (a * a + d * d - f * f) / (2 * a * d);
								tau = acos(costau);
								sintau = sin(tau);
								b = sqrt(
										(a * a * d * d * sintau * sintau )
												/ ((a * a)
														- (d * d * costau
																* costau)));

								// (8) increment the accumulator for the minor axis' half length (b) just estimated
								if (b <= maxminor / 2.0
										&& b >= minminor / 2.0) {
									accu[cvRound(b) - 1] += 1;
									//support[cvRound(b) - 1].push_back((*it3));
								}
							}
						}
					}

					// (10) find the maximum within the accumulator, is it above the threshold?
					max_ind = std::max_element(accu.begin(), accu.end())
							- accu.begin() + 1;
					vote_minor = (*(max_ind + accu.begin() - 1));

					if (vote_minor >= threshold) {

						// (11) save ellipse parameters
						Point2i cen = Point2i(cvRound(center.x), cvRound(center.y));

						Size axis = Size(cvRound(a), max_ind);

						double angle = (alpha * 180) / CV_PI;

						//scoring:
						float jm = minmajor + (maxmajor - minmajor)/2.0;
						float nm = minminor + (maxminor - minminor)/2.0;
						float j = cvRound(a);
						float n = max_ind;

						// more "circular" ellipses are weighted more than very thin ellipses
						//std::cout << n/j << std::endl;
						vote_minor = vote_minor*(50*n/j);

						if (candidates.size() == 0) {
							candidates.push_back(Ellipse(vote_minor, cen, axis, angle));
						}
						for (int el = 0; el < candidates.size(); el++) {
							if (abs(candidates[el].cen.x - cen.x) < 8 &&
									abs(candidates[el].cen.y - cen.y) < 8 &&
									abs(candidates[el].axis.width - j)  < 8 &&
									abs(candidates[el].axis.height - n)  < 8 &&

									//check angle in relation to minor/major axis
									abs(candidates[el].angle - angle)  < (180.0*candidates[el].axis.height)/candidates[el].axis.width) {

								if (candidates[el].vote < vote_minor) {
									candidates[el].cen.x = cen.x;
									candidates[el].cen.y = cen.y;
									candidates[el].axis.width = j;
									candidates[el].axis.height = n;
									candidates[el].angle = angle;
									candidates[el].vote = vote_minor;
								}
								break;
							}
							if (el == candidates.size() - 1) {
								candidates.push_back(Ellipse(vote_minor, cen, axis, angle));
							}
						}

						// (12) remove associated edge pixels from ep
						//not done currently to get the best ellipse possible
						//if (it2 != ep.end()) ep.erase(it2);
					}

					// (13) clear accumulator anyway
					for (std::vector< int>::iterator ini = accu.begin();
							ini != accu.end(); ini++) {
						(*ini) = 0;
					}
				}
			}
		}
		//if (it != ep.end()) ep.erase(it);
	}

	// sort the candidates list according to their vote
	std::sort(candidates.begin(), candidates.end());
	//std::cout << "Found " << candidates.size() << " ellipse candidates." << std::endl;

//	Mat roi;
//	for (int i = 0; i < candidates.size(); i++) {
//		cv::cvtColor(ec, roi, CV_GRAY2BGR);
//		cv::ellipse(roi, Point2i(candidates[i].cen.x, candidates[i].cen.y), candidates[i].axis, candidates[i].angle, 0, 360, Scalar(0,255,0), 1);
//		wimshow("ellipse", roi);
//		waitKey(0);
//	}

	return (candidates);
}

/**
 * Reverse a perspektive transformation circle --> ellipse to ellipse --> circle on the basis of an ellipse.
 *
 * \param roi gray scale region of intereset where an ellipse (ell) is known
 * \param ell Ellipse ellipse from which the transformation parameters are drawn
 */
void ellipseTransform(Mat &roi, Ellipse &ell) {

	Mat rot = Mat(2, 3, CV_64F);

	// rotation angle is the ellipse' orientation
	double a = (ell.angle*CV_PI)/180.0;

	// scale factor in y-direction
	double s = ((double)ell.axis.width)/ ((double) ell.axis.height);

	//center of the transformation
	float x0 = ell.cen.x;
	float y0 = ell.cen.y;

	// create the following rotation matrix: http://goo.gl/H3kZDj
	rot.at<double>(0,0) = cos(a)*cos(a) + s*sin(a)*sin(a);
	rot.at<double>(0,1) = cos(a)*sin(a) - s*cos(a)*sin(a);
	rot.at<double>(0,2) = -(cos(a)*cos(a) + s*sin(a)*sin(a))*x0 + x0 - y0*(cos(a)*sin(a) - s*cos(a)*sin(a));
	rot.at<double>(1,0) = cos(a)*sin(a) - s*cos(a)*sin(a);
	rot.at<double>(1,1) = s*cos(a)*cos(a) + sin(a)*sin(a);
	rot.at<double>(1,2) = -(s*cos(a)*cos(a) + sin(a)*sin(a))*y0 + y0 - x0*(cos(a)*sin(a) - s*cos(a)*sin(a));

	//apply transformation described by the matrix rot
	cv::warpAffine(roi, roi, rot, roi.size());
}

/**
 *
 */
vector<Point2f> getOrientationVector(Mat &roi, Point3f &circle) {


	vector<Point2f> res(2, Point());

	// create circular cutout
	Mat circMask = Mat(roi.rows, roi.cols, CV_8UC1, Scalar(0));
	cv::circle(circMask, Point(circle.x,circle.y), circle.z, Scalar(1), CV_FILLED);

	// apply otsu binarization to this cutout
	double otsut = getOtsuThreshold(roi, circMask);

	Mat circ = roi.mul(circMask);
	Mat hc_white;
	Mat hc_black;

	cv::threshold(circ,hc_white,otsut, 255, CV_THRESH_BINARY);
	hc_black = circ.mul(255-hc_white);

//	std::cout << "Otsu threshold: " << otsut << std::endl;
//	wimshow("white", hc_white);
//	wimshow("black", hc_black);
//	waitKey();

	cv::Moments momw = cv::moments(hc_white,true);
	cv::Moments momb = cv::moments(hc_black,true);

	res[0].x = momb.m10 / momb.m00;
	res[0].y = momb.m01 / momb.m00;

	res[1].x = momw.m10 / momw.m00;
	res[1].y = momw.m01 / momw.m00;


	///////////////////////////////////////////////////////////
	/// ****************************************************///
	///////////////////////////////////////////////////////////

//	vector<Point2f> res(2, Point());
//
//	vector < Point > contour;
//	cv::ellipse2Poly(Point(circle.x, circle.y), Size(circle.z, circle.z), 0, 0, 360, 1, contour);
//
//	//cout << "Circle size: " << contour.size() << endl;
//
//
//	float mu = 0;
//
//	for (int i = 0; i < contour.size(); i++) {
//		mu += roi.at<unsigned char>(contour[i].x,contour[i].y);
//	}
//	mu /= contour.size();
//
//	double black = -1;
//	double white = -1;
//
//	Mat mask = Mat(roi.rows, roi.cols, CV_8UC1, Scalar(0));
//	vector < vector <Point > > contours;
//	contours.push_back(contour);
//	drawContours(mask,contours,0,Scalar(255),CV_FILLED);
//
//	cv::minMaxIdx(roi, &black, &white, 0, 0, mask);
//
//	float blackf = (float) black;
//	float whitef = (float) white;
//
//	for (int i = 0; i < contour.size(); i++) {
//
//		res[0].x += (contour[i].x * abs(roi.at<unsigned char>(contour[i].x,contour[i].y)-whitef));
//		res[0].y += (contour[i].y * abs(roi.at<unsigned char>(contour[i].x,contour[i].y)-whitef));
//
//		res[1].x += (contour[i].x * abs(roi.at<unsigned char>(contour[i].x,contour[i].y)-blackf));
//		res[1].y += (contour[i].y * abs(roi.at<unsigned char>(contour[i].x,contour[i].y)-blackf));
//	}
//
//
//
//	res[0].x = res[0].x / (contour.size() *abs(mu-whitef));
//	res[0].y = res[0].y / (contour.size() *abs(mu-whitef));
//
//	res[1].x = res[1].x / (contour.size() *abs(mu-blackf));
//	res[1].y = res[1].y / (contour.size() *abs(mu-blackf));
//
//	Mat draw;
//	roi.copyTo(draw);
//
//	if (roi.type() == CV_8UC1) {
//		// in case we get a gray scale image as input, we convert it
//		cv::cvtColor(draw,draw,CV_GRAY2BGR);
//	}
//
//	vector < vector <Point > > conts;
//	conts.push_back(contour);
//	drawContours(draw,conts,0,Scalar(255,0,0),1);
//	line(draw, res[0], res[1], Scalar(0,255,0));
//	wimshow("orientcircle",draw);
//	waitKey(0);
//	//cout <<  " x1:"<<res[0].x << ", y1:"<< res[0].y << "; x2:"<< res[1].x << ", y2:" << res[1].y << endl;


	return (res);

}

/** Otsu Binarization
 * \param srcMat grayscale source image
 * \param mask mask which is applied before computing the Otsu binarization threshold
 * \return Otsu-threshold of the masked input image
 */
double getOtsuThreshold(Mat &srcMat, Mat &mask) {
	//Code Snippet from
	//http://stackoverflow.com/questions/12953993/otsu-thresholding-for-depth-image
	cv::Mat copyImg;
	srcMat.copyTo(copyImg);
	uchar* ptr = copyImg.datastart;
	uchar* ptr_end = copyImg.dataend;
	while (ptr < ptr_end) {
		if (*ptr == 0) { // swap if zero
			uchar tmp = *ptr_end;
			*ptr_end = *ptr;
			*ptr = tmp;
			ptr_end--; // make array smaller
		} else {
			ptr++;
		}
	}

	// make a new matrix with only valid data
	cv::Mat nz = Mat(vector<uchar>(copyImg.datastart, ptr_end), true);

	// compute  Otsu threshold
	double thresh = cv::threshold(nz, nz, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

	return (thresh);
}

