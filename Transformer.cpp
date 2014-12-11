/*
 * Transformer.cpp
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#include "Transformer.h"

namespace decoder {
std::vector<Tag> Transformer::process(std::vector<Tag>&& taglist){
    // remove invalid tags
    taglist.erase(std::remove_if(taglist.begin(), taglist.end(), [](Tag& tag) { return !tag.isValid(); }), taglist.end());
    for (Tag& tag : taglist) {
        transformImages(tag);
    }
    return std::move(taglist);
}

void Transformer::transformImages(Tag &tag){
    cv::Mat originalImage = tag.getOrigSubImage();
    std::vector<TagCandidate> candidates = tag.getCandidates();

    for (TagCandidate& candidate : candidates) {
        cv::Mat transformedImage = ellipseTransform(candidate.getEllipse(), originalImage);
        candidate.setTransformedImage(transformedImage);
    }

    tag.setCandidates(std::move(candidates));
}

cv::Mat Transformer::ellipseTransform(Ellipse ell, cv::Mat originalImage) {

    // rotation angle is the ellipse' orientation
    const double a = (ell.getAngle() * CV_PI) / 180.0;

    // scale factor in y-direction
    const double s = (static_cast<double>(ell.getAxis().width)) / (static_cast<double>(ell.getAxis().height));

    //center of the transformation
	const double x0 = ell.getCen().x;
	const double y0 = ell.getCen().y;

    // create the following rotation matrix: http://goo.gl/H3kZDj
    cv::Mat rot(2, 3, CV_64F);
    rot.at<double>(0,0) = cos(a) * cos(a) + s * sin(a) * sin(a);
    rot.at<double>(0,1) = cos(a) * sin(a) - s * cos(a) * sin(a);
    rot.at<double>(0,2) = -(cos(a) * cos(a) + s * sin(a) * sin(a)) * x0 + x0 - y0 * (cos(a) * sin(a) - s * cos(a) * sin(a));
    rot.at<double>(1,0) = cos(a) * sin(a) - s * cos(a) * sin(a);
    rot.at<double>(1,1) = s * cos(a) * cos(a) + sin(a) * sin(a);
    rot.at<double>(1,2) = -(s * cos(a) * cos(a) + sin(a) * sin(a)) * y0 + y0 - x0 * (cos(a) * sin(a) - s * cos(a) * sin(a));

    //apply transformation described by the matrix rot

    cv::Mat transformedImage = originalImage.clone();

    cv::warpAffine(originalImage, transformedImage, rot, transformedImage.size());

    return transformedImage;
}
} /* namespace decoder */
