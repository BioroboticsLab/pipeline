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
    return taglist;
}

void Transformer::transformImages(Tag &tag){
    Mat originalImage = tag.getOrigSubImage();
    std::vector<TagCandidate> candidates = tag.getCandidates();

    for (TagCandidate& candidate : candidates) {
        Mat transformedImage = ellipseTransform(candidate.getEllipse(), originalImage);
        candidate.setTransformedImage(transformedImage);
    }

    tag.setCandidates(std::move(candidates));
}

Mat Transformer::ellipseTransform(Ellipse ell, Mat originalImage) {
    Mat rot = Mat(2, 3, CV_64F);

    // rotation angle is the ellipse' orientation
    double a = (ell.angle * CV_PI) / 180.0;

    // scale factor in y-direction
    double s = (static_cast<double>(ell.axis.width)) / (static_cast<double>(ell.axis.height));

    //center of the transformation
    float x0 = ell.cen.x;
    float y0 = ell.cen.y;

    // create the following rotation matrix: http://goo.gl/H3kZDj
    rot.at<double>(0,0) = cos(a) * cos(a) + s * sin(a) * sin(a);
    rot.at<double>(0,1) = cos(a) * sin(a) - s * cos(a) * sin(a);
    rot.at<double>(0,2) = -(cos(a) * cos(a) + s * sin(a) * sin(a)) * x0 + x0 - y0 * (cos(a) * sin(a) - s * cos(a) * sin(a));
    rot.at<double>(1,0) = cos(a) * sin(a) - s * cos(a) * sin(a);
    rot.at<double>(1,1) = s * cos(a) * cos(a) + sin(a) * sin(a);
    rot.at<double>(1,2) = -(s * cos(a) * cos(a) + sin(a) * sin(a)) * y0 + y0 - x0 * (cos(a) * sin(a) - s * cos(a) * sin(a));

    //apply transformation described by the matrix rot

    Mat transformedImage;
    originalImage.copyTo(transformedImage);

    cv::warpAffine(originalImage, transformedImage, rot, transformedImage.size());

    return transformedImage;
}
} /* namespace decoder */
