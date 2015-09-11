#include "../../settings//PreprocessorSettings.h"

namespace pipeline {
namespace settings {

bool preprocessor_settings_t::get_opt_use_contrast_streching() {
    return this->getValue<bool>(
                Preprocessor::Params::OPT_USE_CONTRAST_STRETCHING);
}

bool preprocessor_settings_t::get_opt_use_equalize_histogram() {
    return this->getValue<bool>(
                Preprocessor::Params::OPT_USE_EQUALIZE_HISTOGRAM);
}

unsigned int preprocessor_settings_t::get_opt_frame_size() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::OPT_FRAME_SIZE);
}

double preprocessor_settings_t::get_opt_average_contrast_value() {
    return this->getValue<double>(
                Preprocessor::Params::OPT_AVERAGE_CONTRAST_VALUE);
}

bool preprocessor_settings_t::get_comb_enabled() {
    return this->getValue<bool>(Preprocessor::Params::COMB_ENABLED);
}

unsigned int preprocessor_settings_t::get_comb_min_size() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::COMB_MIN_SIZE);
}

unsigned int preprocessor_settings_t::get_comb_max_size() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::COMB_MAX_SIZE);
}

double preprocessor_settings_t::get_comb_threshold() {
    return this->getValue<double>(Preprocessor::Params::COMB_THRESHOLD);
}

unsigned int preprocessor_settings_t::get_comb_diff_size() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::COMB_DIFF_SIZE);
}

unsigned int preprocessor_settings_t::get_comb_line_width() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::COMB_LINE_WIDTH);
}

unsigned int preprocessor_settings_t::get_comb_line_color() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::COMB_LINE_COLOR);
}

bool preprocessor_settings_t::get_honey_enabled() {
    return this->getValue<bool>(Preprocessor::Params::HONEY_ENABLED);
}

double preprocessor_settings_t::get_honey_std_dev() {
    return this->getValue<double>(Preprocessor::Params::HONEY_STD_DEV);
}

unsigned int preprocessor_settings_t::get_honey_frame_size() {
    return this->getValue<unsigned int>(
                Preprocessor::Params::HONEY_FRAME_SIZE);
}

double preprocessor_settings_t::get_honey_average_value() {
    return this->getValue<double>(
                Preprocessor::Params::HONEY_AVERAGE_VALUE);
}

preprocessor_settings_t::preprocessor_settings_t() {

    _base = Preprocessor::Params::BASE_STANDALONE;
    /*
         * general optimizations
         */
    addEntry(
                setting_entry(Preprocessor::Params::OPT_USE_CONTRAST_STRETCHING,
                              Preprocessor::Defaults::OPT_USE_CONTRAST_STRETCHING));
    addEntry(
                setting_entry(Preprocessor::Params::OPT_USE_EQUALIZE_HISTOGRAM,
                              Preprocessor::Defaults::OPT_USE_EQUALIZE_HISTOGRAM));
    addEntry(
                setting_entry(Preprocessor::Params::OPT_FRAME_SIZE,
                              Preprocessor::Defaults::OPT_FRAME_SIZE));
    addEntry(
                setting_entry(Preprocessor::Params::OPT_AVERAGE_CONTRAST_VALUE,
                              Preprocessor::Defaults::OPT_AVERAGE_CONTRAST_VALUE));
    /*
         * comb detection
         */

    addEntry(
                setting_entry(Preprocessor::Params::COMB_ENABLED,
                              Preprocessor::Defaults::COMB_ENABLED));
    addEntry(
                setting_entry(Preprocessor::Params::COMB_MIN_SIZE,
                              Preprocessor::Defaults::COMB_MIN_SIZE));

    addEntry(
                setting_entry(Preprocessor::Params::COMB_MAX_SIZE,
                              Preprocessor::Defaults::COMB_MAX_SIZE));

    addEntry(
                setting_entry(Preprocessor::Params::COMB_THRESHOLD,
                              Preprocessor::Defaults::COMB_THRESHOLD));

    addEntry(
                setting_entry(Preprocessor::Params::COMB_DIFF_SIZE,
                              Preprocessor::Defaults::COMB_DIFF_SIZE));

    addEntry(
                setting_entry(Preprocessor::Params::COMB_LINE_WIDTH,
                              Preprocessor::Defaults::COMB_LINE_WIDTH));

    addEntry(
                setting_entry(Preprocessor::Params::COMB_LINE_COLOR,
                              Preprocessor::Defaults::COMB_LINE_COLOR));

    /*
         * honey detection
         */

    addEntry(
                setting_entry(Preprocessor::Params::HONEY_ENABLED,
                              Preprocessor::Defaults::HONEY_ENABLED));
    addEntry(
                setting_entry(Preprocessor::Params::HONEY_STD_DEV,
                              Preprocessor::Defaults::HONEY_STD_DEV));
    addEntry(
                setting_entry(Preprocessor::Params::HONEY_FRAME_SIZE,
                              Preprocessor::Defaults::HONEY_FRAME_SIZE));
    addEntry(
                setting_entry(Preprocessor::Params::HONEY_AVERAGE_VALUE,
                              Preprocessor::Defaults::HONEY_AVERAGE_VALUE));

}

}
}
