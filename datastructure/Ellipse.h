#ifndef ELLIPSE_H_
#define ELLIPSE_H_

#include <opencv2/core/core.hpp> // cv::Size, cv::Point, cv::Mat

namespace decoder {
class Ellipse {

private:

    // number of edge pixels supporting this ellipse
    int vote;
    // this ellipse' center pixel
    cv::Point2i cen;
    // tuple of axis' length (major_axes_length, minor_axes_length)
    cv::Size axis;
    // ellipse' orientation in degrees
    double angle;
    /**
     * subimage of the cannyEdgeMap fpr the bounding box. transformed, so tht ellipse is now a circle with no angle.
     */
    cv::Mat transformedImage;
    mutable cv::Mat binarizedImage;

public:

    double getAngle() const       { return angle; }
    void   setAngle(double angle) {this->angle = angle; }

    cv::Size getAxis() const        { return axis; }
    void     setAxis(cv::Size axis) { this->axis = axis; }

    cv::Point2i getCen() const          { return cen; }
    void        setCen(cv::Point2i cen) { this->cen = cen; }

    int  getVote() const   { return vote; }
    void setVote(int vote) { this->vote = vote; }

    const cv::Mat& getTransformedImage() const                          { return transformedImage; }
    void           setTransformedImage(const cv::Mat& transformedImage);

    const cv::Mat& getBinarizedImage() const;

    explicit Ellipse();

    explicit Ellipse(int vote, cv::Point2i center, cv::Size axis_length, double angle);



};

inline bool operator<(const Ellipse &lhs, const Ellipse &rhs) {
	return lhs.getVote() < rhs.getVote();
}

}
#endif /* ELLIPSE_H_ */
