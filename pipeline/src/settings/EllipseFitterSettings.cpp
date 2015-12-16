#include "../../settings/EllipseFitterSettings.h"

namespace pipeline {
namespace settings {

int ellipsefitter_settings_t::get_canny_initial_high() {
    return this->getValue<int>(EllipseFitter::Params::CANNY_INITIAL_HIGH);
}

int ellipsefitter_settings_t::get_canny_values_distance() {
    return this->getValue<int>(EllipseFitter::Params::CANNY_VALUES_DISTANCE);
}

int ellipsefitter_settings_t::get_canny_mean_min() {
    return this->getValue<int>(EllipseFitter::Params::CANNY_MEAN_MIN);
}

int ellipsefitter_settings_t::get_canny_mean_max() {
    return this->getValue<int>(EllipseFitter::Params::CANNY_MEAN_MAX);
}

int ellipsefitter_settings_t::get_min_major_axis() {
    return this->getValue<int>(EllipseFitter::Params::MIN_MAJOR_AXIS);
}

int ellipsefitter_settings_t::get_max_major_axis() {
    return this->getValue<int>(EllipseFitter::Params::MAX_MAJOR_AXIS);
}

int ellipsefitter_settings_t::get_min_minor_axis() {
    return this->getValue<int>(EllipseFitter::Params::MIN_MAJOR_AXIS);
}

int ellipsefitter_settings_t::get_max_minor_axis() {
    return this->getValue<int>(EllipseFitter::Params::MAX_MAJOR_AXIS);
}

double ellipsefitter_settings_t::get_ellipse_regularisation()
{
    return this->getValue<double>(EllipseFitter::Params::ELLIPSE_REGULARISATION);
}

int ellipsefitter_settings_t::get_threshold_edge_pixels() {
    return this->getValue<int>(EllipseFitter::Params::THRESHOLD_EDGE_PIXELS);
}

int ellipsefitter_settings_t::get_threshold_vote() {
    return this->getValue<int>(EllipseFitter::Params::THRESHOLD_VOTE);
}

int ellipsefitter_settings_t::get_threshold_best_vote() {
    return this->getValue<int>(EllipseFitter::Params::THRESHOLD_BEST_VOTE);
}

bool ellipsefitter_settings_t::get_use_xie_as_fallback(){
    return this->getValue<bool>(EllipseFitter::Params::USE_XIE_AS_FALLBACK);
}

ellipsefitter_settings_t::ellipsefitter_settings_t() {

    _base = EllipseFitter::Params::BASE_STANDALONE;

    addEntry(

                setting_entry(EllipseFitter::Params::CANNY_INITIAL_HIGH,
                              EllipseFitter::Defaults::CANNY_INITIAL_HIGH));

    addEntry(
                setting_entry(EllipseFitter::Params::CANNY_VALUES_DISTANCE,
                              EllipseFitter::Defaults::CANNY_VALUES_DISTANCE));


    addEntry(
                setting_entry(EllipseFitter::Params::CANNY_MEAN_MIN,
                              EllipseFitter::Defaults::CANNY_MEAN_MIN));

    addEntry(
                setting_entry(EllipseFitter::Params::CANNY_MEAN_MAX,
                              EllipseFitter::Defaults::CANNY_MEAN_MAX));


    addEntry(
                setting_entry(EllipseFitter::Params::MIN_MAJOR_AXIS,
                              EllipseFitter::Defaults::MIN_MAJOR_AXIS));

    addEntry(
                setting_entry(EllipseFitter::Params::MAX_MAJOR_AXIS,
                              EllipseFitter::Defaults::MAX_MAJOR_AXIS));

    addEntry(
                setting_entry(EllipseFitter::Params::MIN_MINOR_AXIS,
                              EllipseFitter::Defaults::MIN_MINOR_AXIS));

    addEntry(
                setting_entry(EllipseFitter::Params::MAX_MINOR_AXIS,
                              EllipseFitter::Defaults::MAX_MINOR_AXIS));

    addEntry(
                setting_entry(EllipseFitter::Params::ELLIPSE_REGULARISATION,
                              EllipseFitter::Defaults::ELLIPSE_REGULARISATION));

    addEntry(
                setting_entry(EllipseFitter::Params::THRESHOLD_EDGE_PIXELS,
                              EllipseFitter::Defaults::THRESHOLD_EDGE_PIXELS));
    addEntry(
                setting_entry(EllipseFitter::Params::THRESHOLD_VOTE,
                              EllipseFitter::Defaults::THRESHOLD_VOTE));
    addEntry(
                setting_entry(EllipseFitter::Params::THRESHOLD_BEST_VOTE,
                              EllipseFitter::Defaults::THRESHOLD_BEST_VOTE));

    addEntry(
                setting_entry(EllipseFitter::Params::USE_XIE_AS_FALLBACK,
                              EllipseFitter::Defaults::USE_XIE_AS_FALLBACK));

}

}
}
