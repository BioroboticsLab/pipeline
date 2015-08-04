#include "Settings.h"

namespace pipeline {
namespace settings {

setting_entry::setting_entry(std::string name, int member) :
    setting_name(name), field(member) {
    type = setting_entry_type::INT;

}

setting_entry::setting_entry(std::string name, unsigned int member) :
    setting_name(name), field(member) {
    type = setting_entry_type::U_INT;

}

setting_entry::setting_entry(std::string name, double member) :
    setting_name(name), field(member) {
    type = setting_entry_type::DOUBLE;

}

setting_entry::setting_entry(std::string name, bool member) :
    setting_name(name), field(member) {
    type = setting_entry_type::BOOL;

}

setting_entry::setting_entry(std::string name, size_t member) :
    setting_name(name), field(member) {
    type = setting_entry_type::SIZE_T;
}

setting_entry::setting_entry(std::string name, std::string member) :
    setting_name(name), field(member) {
    type = setting_entry_type::STRING;
}

void settings_abs::print(const boost::property_tree::ptree &pt)
{
    using boost::property_tree::ptree;
    ptree::const_iterator end = pt.end();
    for (ptree::const_iterator it = pt.begin(); it != end; ++it) {
        std::cout << it->first << ": " << it->second.get_value<std::string>() << std::endl;
        print(it->second);
    }
}

void settings_abs::addEntry(setting_entry setting) {
    _settings.insert(
                std::pair<std::string, setting_entry>(setting.setting_name,
                                                      setting));
}

setting_entry settings_abs::getEntry(std::string setting_name) {
    return _settings[setting_name];
}

settings_abs::settings_abs(std::string section) :
    _base(section) {
}

const std::string &settings_abs::getBase() const {
    return _base;
}

void settings_abs::setBase(const std::string &base) {
    _base = base;
}

void settings_abs::addToPTree(boost::property_tree::ptree &pt) {

    typedef std::map<std::string, setting_entry>::iterator it_type;
    for (it_type it = _settings.begin(); it != _settings.end(); it++) {
        setting_entry& entry = it->second;
        pt.put(_base + entry.setting_name, entry.field);

    }

}

boost::property_tree::ptree settings_abs::getPTree() {
    boost::property_tree::ptree pt;
    addToPTree(pt);
    return pt;
}

void settings_abs::print()
{
    print(getPTree());
}

void settings_abs::loadValues(boost::property_tree::ptree ptree) {
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
        case (setting_entry_type::SIZE_T): {
            const boost::optional<size_t> param =
                    ptree.get_optional<size_t>(_base + entry.setting_name);
            if (param)
                entry.field = boost::get<size_t>(param);

            break;
        }
        case (setting_entry_type::STRING): {
            boost::optional<std::string> param = ptree.get_optional<std::string>(
                        _base + entry.setting_name);

            if (param)
                entry.field = boost::get<std::string>(param);

            break;
        }
        }
    }
}

void settings_abs::loadFromJson(std::string filename) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_json(filename, pt);
    loadValues(pt);
}

bool settings_abs::writeToJson(std::string filename) {
    try {

        boost::property_tree::ptree pt = getPTree();

        boost::property_tree::write_json(filename, pt);
        return true;
    } catch (std::exception& e) {
        std::cerr << "Error on writing config in " << filename << std::endl;
        return false;
    }
}

bool settings_abs::writeToJson(std::string filename, boost::property_tree::ptree pt) {
    try {
        boost::property_tree::write_json(filename, pt);
        return true;
    } catch (std::exception& e) {
        std::cerr << "Error on writing config in " << filename << std::endl;
        return false;
    }
}

}
}
