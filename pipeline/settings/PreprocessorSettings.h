#pragma once

#include "Settings.h"

namespace pipeline {
namespace settings {

namespace Preprocessor {
namespace Defaults {

static const bool OPT_USE_CONTRAST_STRETCHING = true;
static const bool OPT_USE_EQUALIZE_HISTOGRAM = false;
static const unsigned int OPT_FRAME_SIZE = 200;
static const double OPT_AVERAGE_CONTRAST_VALUE = 120;

static const bool COMB_ENABLED = true;
static const unsigned int COMB_MIN_SIZE = 65;
static const unsigned int COMB_MAX_SIZE = 80;
static const double COMB_THRESHOLD = 27;
static const unsigned int COMB_DIFF_SIZE = 15;
static const unsigned int COMB_LINE_WIDTH = 9;
static const unsigned int COMB_LINE_COLOR = 0;

static const bool HONEY_ENABLED = true;
static const double HONEY_STD_DEV = 0;
static const unsigned int HONEY_FRAME_SIZE = 30;
static const double HONEY_AVERAGE_VALUE = 80;
}

namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.PREPROCESSOR.";
static const std::string BASE_STANDALONE = "PREPROCESSOR.";
static const std::string OPT_USE_CONTRAST_STRETCHING =
        "OPT_USE_CONTRAST_STRETCHING";
static const std::string OPT_USE_EQUALIZE_HISTOGRAM =
        "OPT_USE_EQUALIZE_HISTOGRAM";
static const std::string OPT_FRAME_SIZE = "OPT_FRAME_SIZE";
static const std::string OPT_AVERAGE_CONTRAST_VALUE =
        "OPT_AVERAGE_CONTRAST_VALUE";

static const std::string COMB_ENABLED = "COMB_ENABLED";
static const std::string COMB_MIN_SIZE = "COMB_MIN_SIZE";
static const std::string COMB_MAX_SIZE = "COMB_MAX_SIZE";
static const std::string COMB_THRESHOLD = "COMB_THRESHOLD";
static const std::string COMB_DIFF_SIZE = "COMB_DIFF_SIZE";
static const std::string COMB_LINE_WIDTH = "COMB_LINE_WIDTH";
static const std::string COMB_LINE_COLOR = "COMB_LINE_COLOR";

static const std::string HONEY_ENABLED = "HONEY_ENABLED";
static const std::string HONEY_STD_DEV = "HONEY_STD_DEV";
static const std::string HONEY_FRAME_SIZE = "HONEY_FRAME_SIZE";
static const std::string HONEY_AVERAGE_VALUE = "HONEY_AVERAGE_VALUE";
}
}

class preprocessor_settings_t: public settings_abs {
public:
    /*
     * general
     */
    bool get_opt_use_contrast_streching();
    bool get_opt_use_equalize_histogram();
    unsigned int get_opt_frame_size();
    double get_opt_average_contrast_value();

    /*
     * comb detection
     */
    bool get_comb_enabled();
    unsigned int get_comb_min_size();
    unsigned int get_comb_max_size();
    double get_comb_threshold();
    unsigned int get_comb_diff_size();
    unsigned int get_comb_line_width();
    unsigned int get_comb_line_color();

    /*
     * honey detection
     */
    bool get_honey_enabled();
    double get_honey_std_dev();
    unsigned int get_honey_frame_size();
    double get_honey_average_value();

    preprocessor_settings_t();
};
}
}
