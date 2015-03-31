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
			const unsigned int version) {
		ar & this->_ellipse;
		ar & this->_decodings;
		ar & this->_grids;
	}

	//prototype of save_construct_data for non-default constructor
	template<class Archive> friend
	void boost::serialization::save_construct_data(Archive & ar,
			pipeline::TagCandidate * c, const unsigned int file_version);

	//prototype of load_construct_data for non-default constructor
	template<class Archive> friend
	void boost::serialization::load_construct_data(Archive & ar,
			pipeline::TagCandidate * c, const unsigned int file_version);
};
}


BOOST_CLASS_EXPORT_KEY(pipeline::TagCandidate)


namespace boost { namespace serialization {
template<class Archive>
inline void save_construct_data(
    Archive & ar, const pipeline::TagCandidate * c, const unsigned long int file_version
){
    // save data required to construct instance
  //  ar << c->getEllipse();
}

template<class Archive>
inline void load_construct_data(
    Archive & ar, pipeline::TagCandidate * c,  unsigned  int file_version
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
