/*
 * Options.h
 *
 *  Created on: 31.07.2014
 *      Author: mareikeziese
 */

#ifndef DECODER_OPTIONS_H_
#define DECODER_OPTIONS_H_

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/variant.hpp>
#include <map>

#ifndef piplineStandalone
class Settings;
#endif

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

static const bool HONEY_ENABLED = false;
static const double HONEY_STD_DEV = 0;
static const unsigned int HONEY_FRAME_SIZE = 30;
static const double HONEY_AVERAGE_VALUE = 80;
}

namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.PREPROCESSOR.";

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

/**
 * @var prefix for the default-values in the config files
 */
static const std::string DEFAULT_PREFIX = "DEFAULT.";

enum setting_entry_type {
	INT = 1, DOUBLE = 2, BOOL = 3, U_INT = 4
};

class setting_entry {
public:
	std::string setting_name;
	boost::variant<int, double, bool, unsigned int> field;
	setting_entry_type type;

	setting_entry() {
	}

	setting_entry(std::string name, int member) :
			setting_name(name), field(member) {
		type = setting_entry_type::INT;

	}

	setting_entry(std::string name, unsigned int member) :
			setting_name(name), field(member) {
		type = setting_entry_type::U_INT;

	}

	setting_entry(std::string name, double member) :
			setting_name(name), field(member) {
		type = setting_entry_type::DOUBLE;

	}
	setting_entry(std::string name, bool member) :
			setting_name(name), field(member) {
		type = setting_entry_type::BOOL;

	}

	void setValue(boost::property_tree::ptree ptree, std::string base) {
		/*switch (type) {
		 case setting_entry_type::INT: {
		 int i = ptree.get<int>(base + configName);
		 int& valref = boost::get<int&>(field);
		 valref = i;
		 break;
		 }

		 case setting_entry_type::DOUBLE: {
		 double d = ptree.get<double>(base + configName);
		 double& valref = boost::get<double&>(field);
		 valref = d;
		 break;
		 }
		 case setting_entry_type::BOOL: {
		 bool d = ptree.get<bool>(base + configName);
		 bool& valref = boost::get<bool&>(field);
		 valref = d;
		 break;
		 }
		 }*/

	}
};

class settings_abs {

protected:
	std::map<std::string, setting_entry> _settings;
	std::string _base;

	void _addEntry(setting_entry setting) {
		_settings.insert(
				std::pair<std::string, setting_entry>(setting.setting_name,
						setting));
	}
	template<typename T>
	T _getValue(std::string setting_name) {
		if (_settings.count(setting_name)) {
			setting_entry entry = _settings[setting_name];
			try {
				return boost::get<T>(entry.field);
			} catch (std::exception & e) {
				std::cerr << "wrong value-type " << entry.field << std::endl;
				throw  std::runtime_error("could not get setting " + setting_name);

			}
		} else {
			throw  std::runtime_error(
					"setting_entry  doesn't exist " + setting_name);
		}
	}

	template<typename T>
	T _setValue(std::string setting_name, T value) {
		setting_entry entry = _settings[setting_name];
		entry.field = value;
		_settings[setting_name] = entry;
	}

	setting_entry _getEnty(std::string setting_name) {
		return _settings[setting_name];
	}

	void _setEntry(setting_entry setting) {

	}

public:

	settings_abs() {

	}

	void loadValues(boost::property_tree::ptree ptree) {

		/*for (setting_entry& entry : _settings) {
		 entry.setValue(ptree, _base);
		 }*/
	}

	void loadDefaults(boost::property_tree::ptree ptree) {
		/*for (setting_entry& entry : _settings) {
		 entry.setValue(ptree, _base + DEFAULT_PREFIX);
		 }*/
	}
#ifndef piplineStandalone
	void loadValues(Settings &settings, std::string base);
#endif

};

class preprocessor_settings_t: public settings_abs {

public:

	/*
	 * general
	 */
	bool get_opt_use_contrast_streching() {
		return this->_getValue<bool>(
				Preprocessor::Params::OPT_USE_CONTRAST_STRETCHING);
	}

	bool get_opt_use_equalize_histogram() {
		return this->_getValue<bool>(
				Preprocessor::Params::OPT_USE_EQUALIZE_HISTOGRAM);
	}
	unsigned int get_opt_frame_size() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::OPT_FRAME_SIZE);
	}

	double get_opt_average_contrast_value() {
		return this->_getValue<double>(
				Preprocessor::Params::OPT_AVERAGE_CONTRAST_VALUE);
	}

	/*
	 * comb detection
	 */

	bool get_comb_enabled() {
		return this->_getValue<bool>(Preprocessor::Params::COMB_ENABLED);
	}

	unsigned int get_comb_min_size() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::COMB_MIN_SIZE);
	}

	unsigned int get_comb_max_size() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::COMB_MAX_SIZE);
	}

	double get_comb_threshold() {
		return this->_getValue<double>(Preprocessor::Params::COMB_THRESHOLD);
	}

	unsigned int get_comb_diff_size() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::COMB_DIFF_SIZE);
	}

	unsigned int get_comb_line_width() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::COMB_LINE_WIDTH);
	}

	unsigned int get_comb_line_color() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::COMB_LINE_COLOR);
	}

	/*
	 * honey detection
	 */

	bool get_honey_enabled() {
		return this->_getValue<bool>(Preprocessor::Params::HONEY_ENABLED);
	}

	double get_honey_std_dev() {
		return this->_getValue<double>(Preprocessor::Params::HONEY_STD_DEV);
	}

	unsigned int get_honey_frame_size() {
		return this->_getValue<unsigned int>(
				Preprocessor::Params::HONEY_FRAME_SIZE);
	}

	double get_honey_average_value() {
		return this->_getValue<double>(Preprocessor::Params::HONEY_AVERAGE_VALUE);
	}


	preprocessor_settings_t() {

		/*
		 * general optimizations
		 */
		_addEntry(
				setting_entry(Preprocessor::Params::OPT_USE_CONTRAST_STRETCHING,
						Preprocessor::Defaults::OPT_USE_CONTRAST_STRETCHING));
		_addEntry(
				setting_entry(Preprocessor::Params::OPT_USE_EQUALIZE_HISTOGRAM,
						Preprocessor::Defaults::OPT_USE_EQUALIZE_HISTOGRAM));
		_addEntry(
				setting_entry(Preprocessor::Params::OPT_FRAME_SIZE,
						Preprocessor::Defaults::OPT_FRAME_SIZE));
		_addEntry(
				setting_entry(Preprocessor::Params::OPT_AVERAGE_CONTRAST_VALUE,
						Preprocessor::Defaults::OPT_AVERAGE_CONTRAST_VALUE));
		/*
		 * comb detection
		 */

		_addEntry(
				setting_entry(Preprocessor::Params::COMB_ENABLED,
						Preprocessor::Defaults::COMB_ENABLED));
		_addEntry(
				setting_entry(Preprocessor::Params::COMB_MIN_SIZE,
						Preprocessor::Defaults::COMB_MIN_SIZE));

		_addEntry(
				setting_entry(Preprocessor::Params::COMB_MAX_SIZE,
						Preprocessor::Defaults::COMB_MAX_SIZE));

		_addEntry(
				setting_entry(Preprocessor::Params::COMB_THRESHOLD,
						Preprocessor::Defaults::COMB_THRESHOLD));

		_addEntry(
				setting_entry(Preprocessor::Params::COMB_DIFF_SIZE,
						Preprocessor::Defaults::COMB_DIFF_SIZE));

		_addEntry(
				setting_entry(Preprocessor::Params::COMB_LINE_WIDTH,
						Preprocessor::Defaults::COMB_LINE_WIDTH));

		_addEntry(
				setting_entry(Preprocessor::Params::COMB_LINE_COLOR,
						Preprocessor::Defaults::COMB_LINE_COLOR));

		/*
		 * honey detection
		 */

		_addEntry(
				setting_entry(Preprocessor::Params::HONEY_ENABLED,
						Preprocessor::Defaults::HONEY_ENABLED));
		_addEntry(
				setting_entry(Preprocessor::Params::HONEY_STD_DEV,
						Preprocessor::Defaults::HONEY_STD_DEV));
		_addEntry(
				setting_entry(Preprocessor::Params::HONEY_FRAME_SIZE,
						Preprocessor::Defaults::HONEY_FRAME_SIZE));
		_addEntry(
				setting_entry(Preprocessor::Params::HONEY_AVERAGE_VALUE,
						Preprocessor::Defaults::HONEY_AVERAGE_VALUE));


	}

	void loadFromIni(std::string filename, std::string section) {

		boost::property_tree::ptree pt;

	}

};
}
}

#endif
