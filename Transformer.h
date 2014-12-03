/*
 * Transformer.h
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#ifndef TRANSFORMER_H_
#define TRANSFORMER_H_

#include "./datastructure/Tag.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#endif

using namespace cv;

namespace decoder {
class Transformer {
private:
    Mat ellipseTransform(Ellipse ell, Mat originalImage);
    void transformImages(Tag &t);

public:
    Transformer() {}
    virtual ~Transformer() {}

    std::vector<Tag> process(std::vector<Tag> &&taglist);
};
} /* namespace decoder */

#endif /* TRANSFORMER_H_ */
