#pragma once

#include "Settings.h"

namespace pipeline {
namespace settings {

namespace EllipseFitter {
namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.ELLIPSEFITTER.";
static const std::string BASE_STANDALONE = "ELLIPSEFITTER.";

/**
 * canny edge detection
 */
static const std::string CANNY_INITIAL_HIGH = "CANNY_INITIAL_HIGH";
static const std::string CANNY_VALUES_DISTANCE = "CANNY_VALUES_DISTANCE";
static const std::string CANNY_MEAN_MIN = "CANNY_MEAN_MIN";
static const std::string CANNY_MEAN_MAX = "CANNY_MEAN_MAX";

/**
 * size constraints
 */
static const std::string MIN_MAJOR_AXIS = "MIN_MAJOR_AXIS";
static const std::string MAX_MAJOR_AXIS = "MAX_MAJOR_AXIS";
static const std::string MIN_MINOR_AXIS = "MIN_MINOR_AXIS";
static const std::string MAX_MINOR_AXIS = "MAX_MINOR_AXIS";

static const std::string ELLIPSE_REGULARISATION = "ELLIPSE_REGULARISATION";

/**
 * voting
 */
static const std::string THRESHOLD_EDGE_PIXELS = "THRESHOLD_EDGE_PIXELS";
static const std::string THRESHOLD_VOTE = "THRESHOLD_VOTE";
static const std::string THRESHOLD_BEST_VOTE = "THRESHOLD_BEST_VOTE";

static const std::string USE_XIE_AS_FALLBACK = "USE_XIE_AS_FALLBACK";
}

namespace Defaults {
static const int CANNY_INITIAL_HIGH = 90;
static const int CANNY_VALUES_DISTANCE = 20;
static const int CANNY_MEAN_MIN = 12;
static const int CANNY_MEAN_MAX = 15;

static const int MIN_MAJOR_AXIS = 42;
static const int MAX_MAJOR_AXIS = 54;
static const int MIN_MINOR_AXIS = 30;
static const int MAX_MINOR_AXIS = 54;

static const double ELLIPSE_REGULARISATION = 50;

static const int THRESHOLD_EDGE_PIXELS = 25;
static const int THRESHOLD_VOTE = 1000;
static const int THRESHOLD_BEST_VOTE = 3000;
static const bool USE_XIE_AS_FALLBACK = true;
}
}

class ellipsefitter_settings_t: public settings_abs {
public:
    int get_canny_initial_high();
    int get_canny_values_distance();
    int get_canny_mean_min();
    int get_canny_mean_max();
    int get_min_major_axis();
    int get_max_major_axis();
    int get_min_minor_axis();
    int get_max_minor_axis();
    double get_ellipse_regularisation();
    int get_threshold_edge_pixels();
    int get_threshold_vote();
    int get_threshold_best_vote();
    bool get_use_xie_as_fallback();

    ellipsefitter_settings_t();
};
}
}
