#pragma once

#include <vector>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"

namespace pipeline {

typedef struct {
} decoder_settings_t;

class Decoder {
public:
	Decoder();

	void loadSettings(decoder_settings_t&& settings);

	std::vector<Tag> process(std::vector<Tag>&& taglist);

private:
	decoder_settings_t _settings;

	std::vector<decoding_t> getDecodings(pipeline::Tag const& tag, TagCandidate &candidate) const;

	double getMeanIntensity(cv::Mat const& image, cv::Mat const& coords) const;
	double getMeanDistance(cv::Mat const& image, cv::Mat const& coords, const double mean) const;
};
}
