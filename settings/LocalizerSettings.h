
#pragma once

#include "Settings.h"

namespace pipeline {
namespace settings {

namespace Localizer {
namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.LOCALIZER.";
static const std::string BASE_STANDALONE = "LOCALIZER.";
static const std::string BINARY_THRESHOLD = "BINARY_THRESHOLD";
static const std::string FIRST_DILATION_NUM_ITERATIONS =
        "FIRST_DILATION_NUM_ITERATIONS";
static const std::string FIRST_DILATION_SIZE = "FIRST_DILATION_SIZE";
static const std::string EROSION_SIZE = "EROSION_SIZE";
static const std::string SECOND_DILATION_SIZE = "SECOND_DILATION_SIZE";
static const std::string TAG_SIZE = "TAG_SIZE";
}

namespace Defaults {
static const int BINARY_THRESHOLD = 29;
static const unsigned int FIRST_DILATION_NUM_ITERATIONS = 4;
static const unsigned int FIRST_DILATION_SIZE = 2;
static const unsigned int EROSION_SIZE = 25;
static const unsigned int SECOND_DILATION_SIZE = 2;
static const unsigned int TAG_SIZE = 60;
}
}

class localizer_settings_t: public settings_abs {
public:
    int get_binary_threshold();
    unsigned int get_first_dilation_num_iterations();
    unsigned int get_first_dilation_size();
    unsigned int get_erosion_size();
    unsigned int get_second_dilation_size();
    unsigned int get_tag_size();

    localizer_settings_t();
};
}
}
