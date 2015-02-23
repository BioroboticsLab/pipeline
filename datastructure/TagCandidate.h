#pragma once

#include <vector>

#include "Ellipse.h"
#include "PipelineGrid.h"

namespace pipeline {
class TagCandidate {
public:
	explicit TagCandidate(Ellipse const& ellipse)
	    : _ellipse(ellipse)
	{}

	Ellipse const& getEllipse() const { return _ellipse; }

	std::vector<PipelineGrid> const& getGrids() const { return _grids; }
	void setGrids(std::vector<PipelineGrid>&& grids) { _grids = std::move(grids); }
	void addGrid(PipelineGrid&& PipelineGrid) { _grids.push_back(std::move(PipelineGrid)); }

private:
	Ellipse _ellipse;
	std::vector<PipelineGrid> _grids;
};
}
