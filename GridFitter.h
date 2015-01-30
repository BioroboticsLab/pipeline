#pragma once

#include <vector>

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

namespace pipeline {

class Tag;
class TagCandidate;

typedef struct {
} gridfitter_settings_t;

class GridFitter {
public:
	GridFitter();

	void loadSettings(gridfitter_settings_t&& settings);

	std::vector<Tag> process(std::vector<Tag>&& taglist);

private:
	gridfitter_settings_t _settings;

	std::vector<Grid> fitGrid(const Tag &tag, TagCandidate const& candidate);
};
}
