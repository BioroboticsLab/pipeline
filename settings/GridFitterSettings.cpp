#include "GridFitterSettings.h"

namespace pipeline {
namespace settings {

double gridfitter_settings_t::get_err_func_alpha_inner() {
    return this->getValue<double>(Gridfitter::Params::ERR_FUNC_ALPHA_INNER);
}

double gridfitter_settings_t::get_err_func_alpha_outer() {
    return this->getValue<double>(Gridfitter::Params::ERR_FUNC_ALPHA_OUTER);
}

double gridfitter_settings_t::get_err_func_alpha_variance() {
    return this->getValue<double>(
                Gridfitter::Params::ERR_FUNC_ALPHA_VARIANCE);
}

double gridfitter_settings_t::get_err_func_alpha_inner_edge() {
    return this->getValue<double>(
                Gridfitter::Params::ERR_FUNC_ALPHA_INNER_EDGE);
}

double gridfitter_settings_t::get_err_func_alpha_outer_edge() {
    return this->getValue<double>(
                Gridfitter::Params::ERR_FUNC_ALPHA_OUTER_EDGE);
}

int gridfitter_settings_t::get_adaptive_block_size() {
    return this->getValue<int>(Gridfitter::Params::ADAPTIVE_BLOCK_SIZE);
}

double gridfitter_settings_t::get_adaptive_c() {
    return this->getValue<double>(Gridfitter::Params::ADAPTIVE_C);
}

size_t gridfitter_settings_t::get_gradient_num_initial() {
    return this->getValue<size_t>(Gridfitter::Params::GRADIENT_NUM_INITIAL);
}

size_t gridfitter_settings_t::get_gradient_num_results() {
    return this->getValue<size_t>(Gridfitter::Params::GRADIENT_NUM_RESULTS);
}

double gridfitter_settings_t::get_gradient_error_threshold() {
    return this->getValue<double>(
                Gridfitter::Params::GRADIENT_ERROR_THRESHOLD);
}

size_t gridfitter_settings_t::get_gradient_max_iterations() {
    return this->getValue<size_t>(
                Gridfitter::Params::GRADIENT_MAX_ITERATIONS);
}

double gridfitter_settings_t::get_eps_angle() {
    return this->getValue<double>(Gridfitter::Params::EPS_ANGLE);
}

int gridfitter_settings_t::get_eps_pos() {
    return this->getValue<int>(Gridfitter::Params::EPS_POS);
}

double gridfitter_settings_t::get_eps_scale() {
    return this->getValue<double>(Gridfitter::Params::EPS_SCALE);
}

double gridfitter_settings_t::get_alpha() {
    return this->getValue<double>(Gridfitter::Params::ALPHA);
}

gridfitter_settings_t::gridfitter_settings_t() {
    using namespace Gridfitter;
    _base = Params::BASE_STANDALONE;

    /*
         * error function weights
         */
    addEntry(
                setting_entry(Params::ERR_FUNC_ALPHA_INNER,
                              Defaults::ERR_FUNC_ALPHA_INNER));
    addEntry(
                setting_entry(Params::ERR_FUNC_ALPHA_INNER_EDGE,
                              Defaults::ERR_FUNC_ALPHA_INNER_EDGE));
    addEntry(
                setting_entry(Params::ERR_FUNC_ALPHA_OUTER,
                              Defaults::ERR_FUNC_ALPHA_OUTER));
    addEntry(
                setting_entry(Params::ERR_FUNC_ALPHA_OUTER_EDGE,
                              Defaults::ERR_FUNC_ALPHA_OUTER_EDGE));
    addEntry(
                setting_entry(Params::ERR_FUNC_ALPHA_VARIANCE,
                              Defaults::ERR_FUNC_ALPHA_VARIANCE));
    /*
         * adaptive
         */
    addEntry(
                setting_entry(Params::ADAPTIVE_BLOCK_SIZE,
                              Defaults::ADAPTIVE_BLOCK_SIZE));
    addEntry(setting_entry(Params::ADAPTIVE_C, Defaults::ADAPTIVE_C));

    /*
         * gradient
         */
    addEntry(
                setting_entry(Params::GRADIENT_ERROR_THRESHOLD,
                              Defaults::GRADIENT_ERROR_THRESHOLD));
    addEntry(
                setting_entry(Params::GRADIENT_MAX_ITERATIONS,
                              Defaults::GRADIENT_MAX_ITERATIONS));
    addEntry(
                setting_entry(Params::GRADIENT_NUM_INITIAL,
                              Defaults::GRADIENT_NUM_INITIAL));
    addEntry(
                setting_entry(Params::GRADIENT_NUM_RESULTS,
                              Defaults::GRADIENT_NUM_RESULTS));

    /*
         * eps
         */

    addEntry(setting_entry(Params::EPS_ANGLE, Defaults::EPS_ANGLE));
    addEntry(setting_entry(Params::EPS_POS, Defaults::EPS_POS));
    addEntry(setting_entry(Params::EPS_SCALE, Defaults::EPS_SCALE));
    addEntry(setting_entry(Params::ALPHA, Defaults::ALPHA));
}
}
}
