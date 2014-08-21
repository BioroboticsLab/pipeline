#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef ELLIPSE_H_
#define ELLIPSE_H_

using namespace cv;
using namespace std;

namespace decoder {
class Ellipse {
public:

	/**************************************
	 *
	 * 			members
	 *
	 **************************************/

	// number of edge pixels supporting this ellipse
	int vote;
	// this ellipse' center pixel
	Point2i cen;
	// tuple of axis' length (major_axes_length, minor_axes_length)
	Size axis;
	// ellipse' orientation in degrees
	double angle;

	/**
	 * subimage of the cannyEdgeMap for the bounding box. transformed, so that ellipse is now a circle with no angle.
	 */
	Mat transformedImage;
	Mat binarizedImage;

	/**************************************
	 *
	 * 			constructor/ destructor
	 *
	 **************************************/
	Ellipse() {
		this->vote = 0;

	}

	Ellipse(int vote, Point2i center, Size axis_length, double angle) {
		this->vote = vote;
		this->angle = angle;
		cen = center;
		axis = axis_length;
	}

	/**************************************
	 *
	 * 			getter/setter
	 *
	 **************************************/

	double getAngle() const {
		return (angle);
	}

	void setAngle(double angle) {
		this->angle = angle;
	}

	Size getAxis() const {
		return (axis);
	}

	void setAxis(Size axis) {
		this->axis = axis;
	}

	Point2i getCen() const {
		return (cen);
	}

	void setCen(Point2i cen) {
		this->cen = cen;
	}

	int getVote() const {
		return (vote);
	}

	void setVote(int vote) {
		this->vote = vote;
	}

	bool operator<(const Ellipse & elli2) const {
		return (vote < elli2.vote);
	}

	const Mat& getTransformedImage() const {
		return transformedImage;
	}

	void setTransformedImage(const Mat& transformedImage) {
		this->transformedImage = transformedImage;
	}
};
}
#endif /* ELLIPSE_H_ */
