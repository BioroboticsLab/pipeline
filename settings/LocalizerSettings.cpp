#include "LocalizerSettings.h"

namespace pipeline {
namespace settings {

int localizer_settings_t::get_binary_threshold() {
    return this->getValue<int>(Localizer::Params::BINARY_THRESHOLD);
}

unsigned int localizer_settings_t::get_first_dilation_num_iterations() {
    return this->getValue<unsigned int>(
                Localizer::Params::FIRST_DILATION_NUM_ITERATIONS);
}

unsigned int localizer_settings_t::get_first_dilation_size() {
    return this->getValue<unsigned int>(
                Localizer::Params::FIRST_DILATION_SIZE);
}

unsigned int localizer_settings_t::get_erosion_size() {
    return this->getValue<unsigned int>(Localizer::Params::EROSION_SIZE);
}

unsigned int localizer_settings_t::get_second_dilation_size() {
    return this->getValue<unsigned int>(
                Localizer::Params::SECOND_DILATION_SIZE);
}

unsigned int localizer_settings_t::get_tag_size() {
    return this->getValue<unsigned int>(Localizer::Params::TAG_SIZE);
}

#ifdef USE_DEEPLOCALIZER
bool localizer_settings_t::get_deeplocalizer_filter()
{
    return getValue<bool>(Localizer::Params::DEEPLOCALIZER_FILTER);
}

std::string localizer_settings_t::get_deeplocalizer_model_file()
{
    return getValue<std::string>(Localizer::Params::DEEPLOCALIZER_MODEL_FILE);
}

std::string localizer_settings_t::get_deeplocalizer_param_file()
{
    return getValue<std::string>(Localizer::Params::DEEPLOCALIZER_PARAM_FILE);
}

double localizer_settings_t::get_deeplocalizer_probability_threshold()
{
    return getValue<double>(Localizer::Params::DEEPLOCALIZER_PROBABILITY_THRESHOLD);
}
#endif

unsigned int localizer_settings_t::get_min_num_pixels()
{
    return getValue<unsigned int>(Localizer::Params::MIN_NUM_PIXELS);
}

unsigned int localizer_settings_t::get_max_num_pixels()
{
    return getValue<unsigned int>(Localizer::Params::MAX_NUM_PIXELS);
}

localizer_settings_t::localizer_settings_t() {

    _base = Localizer::Params::BASE_STANDALONE;

    addEntry(
                setting_entry(Localizer::Params::BINARY_THRESHOLD,
                              Localizer::Defaults::BINARY_THRESHOLD));
    addEntry(
                setting_entry(Localizer::Params::FIRST_DILATION_NUM_ITERATIONS,
                              Localizer::Defaults::FIRST_DILATION_NUM_ITERATIONS));
    addEntry(
                setting_entry(Localizer::Params::FIRST_DILATION_SIZE,
                              Localizer::Defaults::FIRST_DILATION_SIZE));
    addEntry(
                setting_entry(Localizer::Params::EROSION_SIZE,
                              Localizer::Defaults::EROSION_SIZE));
    addEntry(
                setting_entry(Localizer::Params::SECOND_DILATION_SIZE,
                              Localizer::Defaults::SECOND_DILATION_SIZE));
    addEntry(
                setting_entry(Localizer::Params::TAG_SIZE,
                              Localizer::Defaults::TAG_SIZE));
#ifdef USE_DEEPLOCALIZER
    addEntry(
                setting_entry(Localizer::Params::MIN_NUM_PIXELS,
                              Localizer::Defaults::MIN_NUM_PIXELS));
    addEntry(
                setting_entry(Localizer::Params::MAX_NUM_PIXELS,
                              Localizer::Defaults::MAX_NUM_PIXELS));
    addEntry(
                setting_entry(Localizer::Params::DEEPLOCALIZER_FILTER,
                              Localizer::Defaults::DEEPLOCALIZER_FILTER));
    addEntry(
                setting_entry(Localizer::Params::DEEPLOCALIZER_MODEL_FILE,
                              Localizer::Defaults::DEEPLOCALIZER_MODEL_FILE));
    addEntry(
                setting_entry(Localizer::Params::DEEPLOCALIZER_PARAM_FILE,
                              Localizer::Defaults::DEEPLOCALIZER_PARAM_FILE));
    addEntry(
                setting_entry(Localizer::Params::DEEPLOCALIZER_PROBABILITY_THRESHOLD,
                              Localizer::Defaults::DEEPLOCALIZER_PROBABILITY_THRESHOLD));
#endif
}

}
}
