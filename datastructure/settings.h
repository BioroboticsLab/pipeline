/*
 * Options.h
 *
 *  Created on: 31.07.2014
 *      Author: mareikeziese
 */

#ifndef DECODER_OPTIONS_H_
#define DECODER_OPTIONS_H_

#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace Preprocessor {
namespace Defaults {
static const int USE_EQUALIZE_HISTOGRAM= 1;
static const int USE_COMB_DETECTION = 0;
static const int COMB_THRESHOLD = 60;
static const int MIN_COMB_SIZE = 65;
static const int MAX_COMB_SIZE = 75;
static const int DIFF_COMB_SIZE = 15;
static const int COMB_LINE_WIDTH = 9;
static const int COMB_LINE_COLOR = 0;
}

namespace Params {
static const std::string BASE = "BEESBOOKPIPELINE.PREPROCESSOR.";
static const std::string USE_EQUALIZE_HISTOGRAM =   "USE_EQUALIZE_HISTOGRAM";
static const std::string USE_COMB_DETECTION =  "USE_COMB_DETECTION";
static const std::string MIN_COMB_SIZE =   "MIN_COMB_SIZE";
static const std::string MAX_COMB_SIZE =   "MAX_COMB_SIZE";
static const std::string COMB_THRESHOLD =  "COMB_THRESHOLD";
static const std::string DIFF_COMB_SIZE =   "DIFF_COMB_SIZE";
static const std::string COMB_LINE_WIDTH =   "COMB_LINE_WIDTH";
static const std::string COMB_LINE_COLOR =   "COMB_LINE_COLOR";
}
}


namespace decoder {


 struct preprocessor_settings_t{
	///use histogramm-equalisation, make sense on dark images without too many combs
	unsigned int use_equalize_histogram;

	///try to filter combs, to hide them in a binarized image. You can use this for images with combs with a high contrast
	unsigned int use_comb_detection;

	///Threshold for the comb-detection, must be high enough, that all disturbing combs are in the binarized image,
	/// but low enough, that not much more is visible in the binarized image
	double comb_threshold;

	///minimal size for the contours, that may be combs
	unsigned int min_size_comb;

	///maximal size for the contours, that may be combs
	unsigned int max_size_comb;

	/// difference between height and width for the contours, that may be combs
	double diff_size_combs;

	/// width of the line, that is drawed over the comb contour
	int line_width_combs;

	/// value between 0 and 255. It's the color of the line, that is drawed over the comb contour
	unsigned int line_color_combs;
	preprocessor_settings_t(){};
	preprocessor_settings_t(bool useDefault){
		using namespace Preprocessor;

		if(useDefault){
			this->comb_threshold = Defaults::COMB_THRESHOLD;
			this->min_size_comb = Defaults::MIN_COMB_SIZE;
			this->diff_size_combs = Defaults::DIFF_COMB_SIZE;
			this->line_width_combs = Defaults::COMB_LINE_WIDTH;
			this->line_color_combs = Defaults::COMB_LINE_COLOR;
			this->use_comb_detection = Defaults::USE_COMB_DETECTION;
			this->use_equalize_histogram = Defaults::USE_EQUALIZE_HISTOGRAM;
			this->max_size_comb = Defaults::MAX_COMB_SIZE;
		}
	}
	void loadFromIni(std::string filename, std::string section) {

		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini(filename, pt);

		this->use_equalize_histogram = pt.get<unsigned int>(
				section + ".use_equalize_histogramm");
		this->use_comb_detection = pt.get<int>(section + ".use_comb_detection");
		this->comb_threshold = pt.get<double>(section + ".comb_threshold");
		this->min_size_comb = pt.get<unsigned int>(section + ".min_size_comb");
		this->diff_size_combs = pt.get<double>(section + ".diff_size_combs");
		this->line_width_combs = pt.get<int>(section + ".line_width_combs");
		this->max_size_comb = pt.get<unsigned int>(
				section + ".max_size_combs");
	}
} ;

}

#endif
