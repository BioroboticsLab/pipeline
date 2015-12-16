
#pragma once

#include "Settings.h"

namespace pipeline {
namespace settings {

namespace Gridfitter {
namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.GRIDFITTER.";
static const std::string BASE_STANDALONE = "GRIDFITTER.";
/*
 * error function weights
 */
static const std::string ERR_FUNC_ALPHA_INNER = "ERR_FUNC_ALPHA_INNER";
static const std::string ERR_FUNC_ALPHA_OUTER = "ERR_FUNC_ALPHA_OUTER";
static const std::string ERR_FUNC_ALPHA_VARIANCE = "ERR_FUNC_ALPHA_VARIANCE";
static const std::string ERR_FUNC_ALPHA_OUTER_EDGE = "ERR_FUNC_ALPHA_OUTER_EDGE";
static const std::string ERR_FUNC_ALPHA_INNER_EDGE = "ERR_FUNC_ALPHA_INNER_EDGE";
/*
 * mininum value of sobel derivatives in error function
 */
static const std::string SOBEL_THRESHOLD = "SOBEL_THRESHOLD";
/*
 * size of neighbourhood area
 */
static const std::string ADAPTIVE_BLOCK_SIZE = "ADAPTIVE_BLOCK_SIZE";
/*
 * constant which is substracted from mean of neighborhood area
 */
static const std::string ADAPTIVE_C = "ADAPTIVE_C";
/*
 *gradient descent parameters
 */
static const std::string GRADIENT_NUM_INITIAL = "GRADIENT_NUM_INITIAL";
static const std::string GRADIENT_NUM_RESULTS = "GRADIENT_NUM_RESULTS";
static const std::string GRADIENT_ERROR_THRESHOLD = "GRADIENT_ERROR_THRESHOLD";
static const std::string GRADIENT_MAX_ITERATIONS = "GRADIENT_MAX_ITERATIONS";

static const std::string EPS_ANGLE = "EPS_ANGLE";
static const std::string EPS_POS = "EPS_POS";
static const std::string EPS_SCALE = "EPS_SCALE";
static const std::string ALPHA = "ALPHA";
}

namespace Defaults {
static const double ERR_FUNC_ALPHA_INNER = 1.0;
static const double ERR_FUNC_ALPHA_OUTER = 2.0;
static const double ERR_FUNC_ALPHA_VARIANCE = 2.0;
static const double ERR_FUNC_ALPHA_OUTER_EDGE = 1.0;
static const double ERR_FUNC_ALPHA_INNER_EDGE = 4.0;

static const double SOBEL_THRESHOLD = 0.1;

static const int ADAPTIVE_BLOCK_SIZE = 23;
static const double ADAPTIVE_C = 3;

static const size_t GRADIENT_NUM_INITIAL = 6;
static const size_t GRADIENT_NUM_RESULTS = 3;

static const double GRADIENT_ERROR_THRESHOLD = 0.01;
static const size_t GRADIENT_MAX_ITERATIONS = 100;

static const double EPS_ANGLE = 0.01;
static const int EPS_POS = 1;
static const double EPS_SCALE = 0.01;
static const double ALPHA = 1;
}
}

class gridfitter_settings_t: public settings_abs {
public:
    /*
     * error function
     */
    double get_err_func_alpha_inner();
    double get_err_func_alpha_outer();
    double get_err_func_alpha_variance();
    double get_err_func_alpha_inner_edge();
    double get_err_func_alpha_outer_edge();

    double get_sobel_threshold();

    /*
     * adaptive
     */
    int get_adaptive_block_size();
    double get_adaptive_c();

    /*
     * gradient
     */
    size_t get_gradient_num_initial();
    size_t get_gradient_num_results();
    double get_gradient_error_threshold();
    size_t get_gradient_max_iterations();

    /*
     * eps
     */
    double get_eps_angle();
    int get_eps_pos();
    double get_eps_scale();
    double get_alpha();

    gridfitter_settings_t();

};
}
}
