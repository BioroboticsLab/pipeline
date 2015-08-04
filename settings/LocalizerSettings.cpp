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
}

}
}
