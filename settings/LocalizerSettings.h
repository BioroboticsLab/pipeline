#pragma once

#include <string>

#include "Settings.h"

namespace pipeline {
namespace settings {

namespace Localizer {
namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.LOCALIZER.";
static const std::string BASE_STANDALONE = "LOCALIZER.";
}

PARAMETER(BINARY_THRESHOLD, int, 29)
PARAMETER(FIRST_DILATION_NUM_ITERATIONS, unsigned int, 4)
PARAMETER(FIRST_DILATION_SIZE, unsigned int, 2)
PARAMETER(EROSION_SIZE, unsigned int, 25)
PARAMETER(SECOND_DILATION_SIZE, unsigned int, 2)
PARAMETER(TAG_SIZE, unsigned int, 60)

#ifdef USE_DEEPLOCALIZER
PARAMETER(DEEPLOCALIZER_FILTER, bool, false)
PARAMETER(DEEPLOCALIZER_MODEL_FILE, std::string, "")
PARAMETER(DEEPLOCALIZER_PARAM_FILE, std::string, "")
PARAMETER(DEEPLOCALIZER_PROBABILITY_THRESHOLD, double, 0.5f)
#endif

}

class localizer_settings_t: public settings_abs {
public:
    int get_binary_threshold();
    unsigned int get_first_dilation_num_iterations();
    unsigned int get_first_dilation_size();
    unsigned int get_erosion_size();
    unsigned int get_second_dilation_size();
    unsigned int get_tag_size();
    bool get_deeplocalizer_filter();
    std::string get_deeplocalizer_model_file();
    std::string get_deeplocalizer_param_file();
    double get_deeplocalizer_probability_threshold();

    localizer_settings_t();
};
}
}
