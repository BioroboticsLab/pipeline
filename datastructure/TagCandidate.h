#pragma once

#include <vector>

#include "Ellipse.h"
#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

namespace pipeline {
class TagCandidate {
public:
	explicit TagCandidate(Ellipse const& ellipse)
	    : _ellipse(ellipse)
	{}

	Ellipse const& getEllipse() const { return _ellipse; }

	std::vector<Grid> const& getGrids() { return _grids; }
	void setGrids(std::vector<Grid>&& grids) { _grids = std::move(grids); }
	void addGrid(Grid&& grid) { _grids.push_back(std::move(grid)); }

private:
	Ellipse _ellipse;
	std::vector<Grid> _grids;
};
}
