#pragma once

#include <vector>

namespace pipeline {

class Tag;
typedef struct {
} decoder_settings_t;

class Decoder {
public:
	Decoder();

	void loadSettings(decoder_settings_t&& settings);

	std::vector<Tag> process(std::vector<Tag>&& taglist);
};
}
