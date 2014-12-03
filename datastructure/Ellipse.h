#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#ifndef ELLIPSE_H_
#define ELLIPSE_H_


namespace decoder {
class Ellipse {
public:
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
    cv::Mat binarizedImage;

    double getAngle() const;
    void setAngle(double angle);
    cv::Size getAxis() const;
    void setAxis(cv::Size axis);
    cv::Point2i getCen() const;
    void setCen(cv::Point2i cen);
    int getVote() const;
    void setVote(int vote);

    Ellipse();

    Ellipse(int vote, cv::Point2i center, cv::Size axis_length, double angle);

    bool operator<(const Ellipse & elli2) const;

    const cv::Mat& getTransformedImage() const {
        return transformedImage;
    }

    void setTransformedImage(const cv::Mat& transformedImage) {
        this->transformedImage = transformedImage;
    }
};
}
#endif /* ELLIPSE_H_ */
