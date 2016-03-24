#pragma once

#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/variant.hpp>
#include <map>
#include <iostream>

#ifndef pipelineStandalone
namespace BioTracker {
namespace Core {
class Settings;
}
}
#endif

#define PARAMETER(NAME, TYPE, DEFAULT_VALUE) \
    namespace Params { \
    static const std::string NAME = #NAME; \
    } \
    namespace Defaults { \
    static const TYPE NAME = DEFAULT_VALUE; \
    }

namespace pipeline {
namespace settings {

/**
 * @var prefix for the default-values in the config files
 */
static const std::string DEFAULT_PREFIX = "DEFAULT.";

enum setting_entry_type {
    INT = 0,
    DOUBLE,
    BOOL,
    U_INT,
    SIZE_T,
    STRING
};

class setting_entry {
public:
    std::string setting_name;
    boost::variant<int, double, bool, unsigned int, size_t, std::string> field;
    setting_entry_type type;

    setting_entry() {}
    setting_entry(std::string name, int member);
    setting_entry(std::string name, unsigned int member);
    setting_entry(std::string name, double member);
    setting_entry(std::string name, bool member);
    setting_entry(std::string name, size_t member);
    setting_entry(std::string name, std::string member);
};

class settings_abs {

private:
    void print(boost::property_tree::ptree const& pt);

protected:
    std::map<std::string, setting_entry> _settings;
    std::string _base;

    void addEntry(setting_entry setting);

public:
    template<typename T>
    T getValue(std::string setting_name) {
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
    void setValue(std::string setting_name, T value) {
        setting_entry entry = _settings[setting_name];
        entry.field = value;
        _settings[setting_name] = entry;
    }

    setting_entry getEntry(std::string setting_name);

    settings_abs() {}
    settings_abs(std::string section);

    const std::string& getBase() const;
    void setBase(const std::string& base);

    /**
     * add settings to existing ptree
     * @param pt propertytree with existing settings
     */
    void addToPTree(boost::property_tree::ptree& pt);

    /**
     * get ptTree with all settings
     * @return ptree
     */
    boost::property_tree::ptree getPTree();

    void print();

    /**
     *  writes all values found in the ptree in the depending entries
     * @param ptree with nodes for the settings
     */
    void loadValues(boost::property_tree::ptree ptree);
    /**
     * load all setting values form the json-file (concerning to the base)
     * @param filename absolute path to the file
     */

    void loadFromJson(std::string filename);
    /**
     * writes all setting values form the json-file (concerning to the base)
     * @param filename absolute path to the file
     */

    bool writeToJson(std::string filename);
    bool writeToJson(std::string filename, boost::property_tree::ptree pt);

#ifndef pipelineStandalone
    void loadValues(BioTracker::Core::Settings &settings, std::string base);
#endif
};


}
}
