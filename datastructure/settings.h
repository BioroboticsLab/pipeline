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
#include <iostream>

#ifndef piplineStandalone
class Settings;
#endif

namespace pipeline {

namespace settings {

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
				throw std::runtime_error(
						"could not get setting " + setting_name);

			}
		} else {
			throw std::runtime_error(
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

public:

	settings_abs() {

	}
	settings_abs(std::string section) :
			_base(section) {
	}

	const std::string& getBase() const {
		return _base;
	}

	void setBase(const std::string& base) {
		_base = base;
	}
	/**
	 * add settings to existing ptree
	 * @param pt propertytree with existing settings
	 */
	void addToPTree(boost::property_tree::ptree& pt) {

		typedef std::map<std::string, setting_entry>::iterator it_type;
		for (it_type it = _settings.begin(); it != _settings.end(); it++) {
			setting_entry& entry = it->second;
			pt.put(_base + entry.setting_name, entry.field);

		}

	}
	/**
	 * get ptTree with all settings
	 * @return ptree
	 */
	boost::property_tree::ptree getPTree() {
		boost::property_tree::ptree pt;
		addToPTree(pt);
		return pt;
	}

	/**
	 *  writes all values found in the ptree in the depending entries
	 * @param ptree with nodes for the settings
	 */
	void loadValues(boost::property_tree::ptree ptree) {

		typedef std::map<std::string, setting_entry>::iterator it_type;
		for (it_type it = _settings.begin(); it != _settings.end(); it++) {
			setting_entry& entry = it->second;

			switch (entry.type) {
			case (setting_entry_type::INT): {
				boost::optional<int> param = ptree.get_optional<int>(
						_base + entry.setting_name);

				if (param)
					entry.field = boost::get<int>(param);

				break;
			}
			case (setting_entry_type::DOUBLE): {
				boost::optional<double> param = ptree.get_optional<double>(
						_base + entry.setting_name);
				if (param)
					entry.field = boost::get<double>(param);
				break;
			}
			case (setting_entry_type::BOOL): {
				boost::optional<bool> param = ptree.get_optional<bool>(
						_base + entry.setting_name);
				if (param)
					entry.field = boost::get<bool>(param);

				break;
			}
			case (setting_entry_type::U_INT): {
				const boost::optional<unsigned int> param = ptree.get_optional<
						unsigned int>(_base + entry.setting_name);
				if (param)
					entry.field = boost::get<unsigned int>(param);

				break;
			}
			}
		}
	}
	/**
	 * load all setting values form the json-file (concerning to the base)
	 * @param filename absolute path to the file
	 */
	void loadFromJson(std::string filename) {
		boost::property_tree::ptree pt;
		boost::property_tree::read_json(filename, pt);
		loadValues(pt);
	}
	/**
	 * writes all setting values form the json-file (concerning to the base)
	 * @param filename absolute path to the file
	 */
	bool writeToJson(std::string filename) {
		try {

			boost::property_tree::ptree pt = getPTree();

			boost::property_tree::write_json(filename, pt);
			return true;
		} catch (std::exception& e) {
			std::cerr << "Error on writing config in " << filename << std::endl;
			return false;
		}
	}

	bool writeToJson(std::string filename, boost::property_tree::ptree pt) {
		try {
			boost::property_tree::write_json(filename, pt);
			return true;
		} catch (std::exception& e) {
			std::cerr << "Error on writing config in " << filename << std::endl;
			return false;
		}
	}

#ifndef piplineStandalone
	void loadValues(Settings &settings, std::string base);
#endif

#ifdef piplineStandalone

#endif

};

/**************************************
 *
 *         Preprocessor Settings
 *
 **************************************/

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
		return this->_getValue<double>(
				Preprocessor::Params::HONEY_AVERAGE_VALUE);
	}

	preprocessor_settings_t() {

		_base = Preprocessor::Params::BASE_STANDALONE;
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
};

/**************************************
 *
 *           Localizer settings
 *
 **************************************/

namespace Localizer {
namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.LOCALIZER.";
static const std::string BASE_STANDALONE = "LOCALIZER.";
static const std::string BINARY_THRESHOLD = "BINARY_THRESHOLD";
static const std::string FIRST_DILATION_NUM_ITERATIONS =
		"FIRST_DILATION_NUM_ITERATIONS";
static const std::string FIRST_DILATION_SIZE = "FIRST_DILATION_SIZE";
static const std::string EROSION_SIZE = "EROSION_SIZE";
static const std::string SECOND_DILATION_SIZE = "SECOND_DILATION_SIZE";
static const std::string MAX_TAG_SIZE = "MAX_TAG_SIZE";
static const std::string MIN_BOUNDING_BOX_SIZE = "MIN_BOUNDING_BOX_SIZE";
}

namespace Defaults {
static const int BINARY_THRESHOLD = 29;
static const int FIRST_DILATION_NUM_ITERATIONS = 4;
static const int FIRST_DILATION_SIZE = 2;
static const int EROSION_SIZE = 25;
static const int SECOND_DILATION_SIZE = 2;
static const int MAX_TAG_SIZE = 250;
static const int MIN_BOUNDING_BOX_SIZE = 100;
}
}

class localizer_settings_t: public settings_abs {

public:
	int get_binary_threshold() {
		return this->_getValue<int>(Localizer::Params::BINARY_THRESHOLD);
	}

	int get_first_dilation_num_iterations() {
		return this->_getValue<int>(
				Localizer::Params::FIRST_DILATION_NUM_ITERATIONS);
	}

	int get_first_dilation_size() {
		return this->_getValue<int>(Localizer::Params::FIRST_DILATION_SIZE);
	}

	int get_erosion_size() {
		return this->_getValue<int>(Localizer::Params::EROSION_SIZE);
	}

	int get_second_dilation_size() {
		return this->_getValue<int>(Localizer::Params::SECOND_DILATION_SIZE);
	}

	int get_max_tag_size() {
		return this->_getValue<int>(Localizer::Params::MAX_TAG_SIZE);
	}
	int get_min_bounding_box_size() {
		return this->_getValue<int>(Localizer::Params::MIN_BOUNDING_BOX_SIZE);
	}

	localizer_settings_t() {

		_base = Localizer::Params::BASE_STANDALONE;

		_addEntry(
				setting_entry(Localizer::Params::BINARY_THRESHOLD,
						Localizer::Defaults::BINARY_THRESHOLD));
		_addEntry(
				setting_entry(Localizer::Params::FIRST_DILATION_NUM_ITERATIONS,
						Localizer::Defaults::FIRST_DILATION_NUM_ITERATIONS));
		_addEntry(
				setting_entry(Localizer::Params::FIRST_DILATION_SIZE,
						Localizer::Defaults::FIRST_DILATION_SIZE));
		_addEntry(
				setting_entry(Localizer::Params::EROSION_SIZE,
						Localizer::Defaults::EROSION_SIZE));
		_addEntry(
				setting_entry(Localizer::Params::SECOND_DILATION_SIZE,
						Localizer::Defaults::SECOND_DILATION_SIZE));
		_addEntry(
				setting_entry(Localizer::Params::MAX_TAG_SIZE,
						Localizer::Defaults::MAX_TAG_SIZE));
		_addEntry(
				setting_entry(Localizer::Params::MIN_BOUNDING_BOX_SIZE,
						Localizer::Defaults::MIN_BOUNDING_BOX_SIZE));
	}
};
}
}

#endif
