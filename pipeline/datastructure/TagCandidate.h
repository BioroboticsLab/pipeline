#pragma once

#include <bitset>
#include <vector>

#include "Ellipse.h"
#include "PipelineGrid.h"
#include "serialization.hpp"

namespace pipeline {
typedef std::bitset<Grid::NUM_MIDDLE_CELLS> decoding_t;

class TagCandidate {
public:
	explicit TagCandidate(Ellipse const& ellipse) :
			_ellipse(ellipse) {
	}

	Ellipse const& getEllipse() const {
		return _ellipse;
	}

	std::vector<PipelineGrid>& getGrids() {
		return _grids;
	}
	std::vector<PipelineGrid> const& getGridsConst() const {
		return _grids;
	}
	void setGrids(std::vector<PipelineGrid>&& grids) {
		_grids = std::move(grids);
	}
	void addGrid(PipelineGrid&& PipelineGrid) {
		_grids.push_back(std::move(PipelineGrid));
	}

	std::vector<decoding_t> const& getDecodings() const {
		return _decodings;
	}
	void setDecodings(std::vector<decoding_t>&& decodings) {
		_decodings = std::move(decodings);
	}
	void addDecoding(decoding_t&& decoding) {
		_decodings.push_back(std::move(decoding));
	}

private:
	Ellipse _ellipse;
	std::vector<PipelineGrid> _grids;
	std::vector<decoding_t> _decodings;

	friend class boost::serialization::access;
	template<class Archive> void serialize(Archive & ar,
			const unsigned int) {
		ar & BOOST_SERIALIZATION_NVP(_ellipse);
		ar & BOOST_SERIALIZATION_NVP(_decodings);
		ar & BOOST_SERIALIZATION_NVP(_grids);
	}
};
}


BOOST_CLASS_EXPORT_KEY(pipeline::TagCandidate)


namespace boost { namespace serialization {
template<class Archive>
inline void load_construct_data(
    Archive &, pipeline::TagCandidate * c,  unsigned  int
){
    // retrieve data from archive required to construct new instance
	//pipeline::Ellipse e;
    //ar >> e;
	/**
		 * @TODO fix ME!!
		 */
    // invoke inplace constructor to initialize instance of PipelineGrid
    ::new(c) pipeline::TagCandidate(pipeline::Ellipse());
}
}}
