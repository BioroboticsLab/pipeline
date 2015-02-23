#pragma once

#include <bitset>
#include <vector>

#include "Ellipse.h"
#include "PipelineGrid.h"

namespace pipeline {
typedef std::bitset<Grid::NUM_MIDDLE_CELLS> decoding_t;

class TagCandidate {
public:
	explicit TagCandidate(Ellipse const& ellipse)
	    : _ellipse(ellipse)
	{}

	Ellipse const& getEllipse() const { return _ellipse; }

	std::vector<PipelineGrid>& getGrids() { return _grids; }
	std::vector<PipelineGrid> const& getGridsConst() const { return _grids; }
	void setGrids(std::vector<PipelineGrid>&& grids) { _grids = std::move(grids); }
	void addGrid(PipelineGrid&& PipelineGrid) { _grids.push_back(std::move(PipelineGrid)); }

	std::vector<decoding_t> const& getDecodings() const { return _decodings; }
	void setDecodings(std::vector<decoding_t>&& decodings) { _decodings = std::move(decodings); }
	void addDecoding(decoding_t&& decoding) { _decodings.push_back(std::move(decoding)); }

private:
	Ellipse _ellipse;
	std::vector<PipelineGrid> _grids;
	std::vector<decoding_t> _decodings;
};
}
