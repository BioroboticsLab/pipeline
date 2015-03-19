#ifndef ELLIPSE_H_
#define ELLIPSE_H_

#include <opencv2/core/core.hpp> // cv::Size, cv::Point, cv::Mat

namespace pipeline {

class Ellipse {
private:
	// number of edge pixels supporting this ellipse
	int _vote;
	// this ellipse' center pixel
	cv::Point2i _cen;
	// tuple of axis' length (major_axes_length, minor_axes_length)
	cv::Size _axis;
	// ellipse' orientation in degrees
	double _angle;
	// dimensions of ROI
	cv::Size _roiSize;

public:
	double getAngle() const       { return _angle; }
	void   setAngle(double angle) { _angle = angle; }

	cv::Size getAxis() const        { return _axis; }
	void     setAxis(cv::Size axis) { _axis = axis; }

	cv::Point2i getCen() const          { return _cen; }
	void        setCen(cv::Point2i cen) { _cen = cen; }

	int  getVote() const   { return _vote; }
	void setVote(int vote) {  _vote = vote; }

	// TODO: add caching
	const cv::Mat getMask(const cv::Size axisBorder = cv::Size(0, 0)) const;

	explicit Ellipse();
	explicit Ellipse(int vote, cv::Point2i center, cv::Size axis_length, double angle, cv::Size roiSize);
};

inline bool operator<(const Ellipse &lhs, const Ellipse &rhs) {
	return lhs.getVote() < rhs.getVote();
}

}
#endif /* ELLIPSE_H_ */
