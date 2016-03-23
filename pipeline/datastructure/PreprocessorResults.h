#pragma once

#include <opencv2/core/core.hpp>

#include "serialization.hpp"

namespace pipeline {

struct PreprocessorResult {
    cv::Mat originalImage;
    cv::Mat preprocessedImage;
    cv::Mat claheImage;
};

}
