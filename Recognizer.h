#pragma once

#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include "datastructure/Tag.h"
#include "datastructure/Ellipse.h"
#include "datastructure/settings.h"

#ifdef PipelineStandalone
#include "../config.h"
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#endif


namespace pipeline {


class Recognizer {
private:
	settings::recognizer_settings_t _settings;

    void detectXieEllipse(Tag &tag);

#ifdef PipelineStandalone
    void loadConfigVars(std::string filename);
    void visualizeEllipse(Ellipse const& ell, std::string const& title);
#endif

public:
    Recognizer();
#ifdef PipelineStandalone
    Recognizer(std::string configFile);
#endif
    virtual ~Recognizer() {}

	void loadSettings(settings::recognizer_settings_t &&settings);

    std::vector<Tag> process(std::vector<Tag> &&taglist);
    void visualizeEllipse(Tag const& tag , Ellipse const& ell, std::string const& title);

	cv::Mat computeCannyEdgeMap(const cv::Mat &grayImage);
};
}
