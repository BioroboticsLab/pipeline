#pragma once

#include <array>
#include <bitset>

#include <boost/optional.hpp>

#include "Grid.h"

#include "../util/Util.h"
#include "serialization.hpp"

#include <boost/serialization/export.hpp>




namespace pipeline {
typedef std::bitset<Grid::NUM_MIDDLE_CELLS> decoding_t;
}

class PipelineGrid: private Grid {
public:
	typedef struct {
		// rasterized coordinates of an grid area (outer ring etc.) for the
		// current grid configuration
		std::vector<cv::Point2i> areaCoordinates;
		/*friend class boost::serialization::access;

				//needed to serialize class implicit
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version) {
					ar & BOOST_SERIALIZATION_NVP(this->areaCoordinates);
				}*/

	} coordinates_t;

	static const uint8_t NOID = 255;

	explicit PipelineGrid(cv::Point2i center, double radius, double angle_z,
			double angle_y, double angle_x);
	explicit PipelineGrid(Util::gridconfig_t const& config);
	virtual ~PipelineGrid() {
	}

	Util::gridconfig_t getConfig() const;

	template<typename Func>
	Func processOuterRingCoordinates(Func&& coordinateFunction);

	template<typename Func>
	Func processInnerWhiteRingCoordinates(Func&& coordinateFunction);

	template<typename Func>
	Func processInnerBlackRingCoordinates(Func&& coordinateFunction);

	template<typename Func>
	Func processGridCellCoordinates(const size_t idx,
			Func&& coordinateFunction);

	template<typename Func>
	Func processOuterRingEdgeCoordinates(Func&& coordinateFunction) const;

	template<typename Func>
	Func processInnerLineCoordinates(Func&& coordinateFunction) const;

	// legacy code
	const std::vector<cv::Point2i> getOuterRingEdgeCoordinates();

	// TODO: maybe merge different draw functions
	cv::Mat getProjectedImage(const cv::Size2i size) const;
	void draw(cv::Mat& img, boost::optional<pipeline::decoding_t> decoding =
			boost::optional<pipeline::decoding_t>());
	void drawContours(cv::Mat& img, const double transparency,
			const cv::Vec3b &color = cv::Vec3b(255, 255, 255)) const;

	void setXRotation(double angle) {
		Grid::setXRotation(angle);
		resetCache();
	}
	double getXRotation() const {
		return Grid::getXRotation();
	}

	void setYRotation(double angle) {
		Grid::setYRotation(angle);
		resetCache();
	}
	double getYRotation() const {
		return Grid::getYRotation();
	}

	void setZRotation(double angle) {
		Grid::setZRotation(angle);
		resetCache();
	}
	double getZRotation() const {
		return Grid::getZRotation();
	}

	void setCenter(cv::Point center);
	cv::Point getCenter() const {
		return Grid::getCenter();
	}

	void setRadius(double radius) {
		Grid::setRadius(radius);
		resetCache();
	}
	double getRadius() const {
		return Grid::getRadius();
	}

	cv::Rect getBoundingBox() const {
		return Grid::getBoundingBox();
	}

	idarray_t const& getIdArray() const {
		return Grid::getIdArray();
	}
	void setIdArray(idarray_t const& array) {
		_ID = array;
	}

	double compare(const PipelineGrid &to) const;

private:


	// contains the cached coordinates of the current grid parameters
	// or is invalid.
	typedef boost::optional<coordinates_t> cached_coordinates_t;

	// image representation of the different areas and edges of the rasterized
	// grid. during the calculation of the coordinates, each area/edge is filled
	// with a scalar value equal to the the index of the area. for edges, a
	// constant (CONTOUR_OFFSET) is added to the scalar.
	cv::Mat _idImage;

	cached_coordinates_t _outerRingCoordinates;
	cached_coordinates_t _innerWhiteRingCoordinates;
	cached_coordinates_t _innerBlackRingCoordinates;
	std::array<cached_coordinates_t, NUM_MIDDLE_CELLS> _gridCellCoordinates;

	// returns the bounding box of a single polygon (identified by the index
	// of the polygon area [0, Grid::NUM_MIDDLE_CELLS)
	cv::Rect getPolygonBoundingBox(size_t idx);

	// convenience function to avoid code duplication, either return the already
	// cached coordinates or calculates the coordinates and then returns them
	template<typename Func>
	Func processCoordinates(cached_coordinates_t& coordinates, const size_t idx,
			Func&& coordinateFunction);

	template<typename Func>
	struct polygon_coords_return_t {
		polygon_coords_return_t(coordinates_t&& coordinates,
				Func&& coordinateFunction) :
				coordinates(std::move(coordinates)), coordinateFunction(
						std::move(coordinateFunction)) {
		}

		polygon_coords_return_t(const polygon_coords_return_t&) = delete;
		polygon_coords_return_t& operator=(const polygon_coords_return_t&) = delete;

		polygon_coords_return_t(polygon_coords_return_t&&) = default;
		polygon_coords_return_t& operator=(polygon_coords_return_t&&) = default;

		coordinates_t coordinates;
		Func coordinateFunction;
	};

	// calculates the rasterized coordinates of the (convex) polygon with the
	// given index
	template<typename Func>
	polygon_coords_return_t<Func> calculatePolygonCoordinates(const size_t idx,
			Func&& coordinateFunction);

	template<typename Func>
	Func processEdgeCoordinates(const size_t idx,
			Func&& coordinateFunction) const;

	template<typename Func>
	Func processLineCoordinates(const cv::Point start, const cv::Point end,
			Func&& coordinateFunction) const;

	// resets the coordinate caches. has to be called after a change of
	// orientation or scale but not after a position change.
	void resetCache();

	template<typename Func>
	class cacheSetter {
	public:
		explicit cacheSetter(const size_t idx, cv::Mat& idImage,
				PipelineGrid::coordinates_t& coordinateCache,
				Func& coordinateFunction, const cv::Point2i idImageOffset,
				const cv::Point2i gridCenter) :
				_idx(idx), _idImage(idImage), _coordinateCache(coordinateCache), _coordinateFunction(
						coordinateFunction), _idImageOffset(idImageOffset), _gridCenter(
						gridCenter) {
		}

		cacheSetter(const cacheSetter&) = delete;
		cacheSetter& operator=(const cacheSetter&) = delete;

		cacheSetter(cacheSetter&&) = default;
		cacheSetter& operator=(cacheSetter&&) = default;

		inline void operator()(cv::Point coords) {
			assert(
					Util::pointInBounds(_idImage.get().size(),
							coords - _idImageOffset));

			// TODO: maybe speed up using raw pointer access
			_idImage.get().template at<uint8_t>(coords - _idImageOffset) = _idx;
			_coordinateCache.get().areaCoordinates.push_back(
					coords + _gridCenter);
			(_coordinateFunction.get())(coords + _gridCenter);
		}

	protected:
		size_t _idx;
		std::reference_wrapper<cv::Mat> _idImage;
		std::reference_wrapper<PipelineGrid::coordinates_t> _coordinateCache;
		std::reference_wrapper<Func> _coordinateFunction;
		cv::Point2i _idImageOffset;
		cv::Point2i _gridCenter;
	};

	template<typename Func>
	class cacheSetterOuter: private cacheSetter<Func> {
	public:
		explicit cacheSetterOuter(const size_t idx, cv::Mat& idImage,
				PipelineGrid::coordinates_t& coordinateCache,
				Func& coordinateFunction, const cv::Point2i idImageOffset,
				const cv::Point2i gridCenter) :
				cacheSetter<Func>(idx, idImage, coordinateCache,
						coordinateFunction, idImageOffset, gridCenter) {
		}

		cacheSetterOuter(const cacheSetterOuter&) = delete;
		cacheSetterOuter& operator=(const cacheSetterOuter&) = delete;

		cacheSetterOuter(cacheSetterOuter&&) = default;
		cacheSetterOuter& operator=(cacheSetterOuter&&) = default;

		inline void operator()(cv::Point coords) {
			assert(
					Util::pointInBounds(this->_idImage.get().size(),
							coords - this->_idImageOffset));

			uint8_t value = this->_idImage.get().template at<uint8_t>(
					coords - this->_idImageOffset);
			if (value == PipelineGrid::NOID) {
				cacheSetter<Func>::operator ()(coords);
			}
		}
	};

	//needed to serialize all the private members
		friend class boost::serialization::access;

//need to split load/save to construct additionally data on load
		template<class Archive>
		void save(Archive & ar, const unsigned int version) const
		{
		    // invoke serialization of the base class
			ar << this->_angle_x;
					    ar << this->_angle_y;
					    ar << this->_angle_z;
					    ar << this->_boundingBox;
					    ar << this->_center;
		}

		template<class Archive>
		void load(Archive & ar, const unsigned int version)
		{
			ar >> this->_angle_x;
					    ar >> this->_angle_y;
					    ar >> this->_angle_z;
					    ar >> this->_boundingBox;
					    ar >> this->_center;

					    this->prepare_visualization_data();
		}

		template<class Archive>
		void serialize(
		    Archive & ar,
		    const unsigned int file_version
		){
		    boost::serialization::split_member(ar, *this, file_version);
		}


		 //prototype of save_construct_data for non-default constructor
		        template<class Archive> friend
		            void boost::serialization::save_construct_data(Archive & ar,
		                          PipelineGrid * g, const unsigned int file_version);

		        //prototype of load_construct_data for non-default constructor
		        template<class Archive> friend
		            void boost::serialization::load_construct_data(Archive & ar,
		            		PipelineGrid * g, const unsigned int file_version);


};

BOOST_CLASS_EXPORT_KEY(PipelineGrid)

namespace boost {
namespace serialization {
template<class Archive>
 void save_construct_data(Archive & ar, const PipelineGrid * g, const unsigned long int file_version) {
	// save data required to construct instance
	/*ar << g->getCenter();
	ar << g->getRadius();
	ar << g->getXRotation();
	ar << g->getYRotation();
	ar << g->getZRotation();*/
}

template<class Archive>
 void load_construct_data(Archive & ar, PipelineGrid * g,unsigned int file_version) {
	// retrieve data from archive required to construct new instance
	/*cv::Point2i center;
	double radius, angle_z, angle_y, angle_x;
	ar >> center;
	ar >> radius;
	ar >> angle_x;
	ar >> angle_y;
	ar >> angle_z;*/

	/**
		 * @TODO fix ME!!
		 */
	Util::gridconfig_t config = Util::gridconfig_t();
	config.angle_x = 0;
	config.angle_y = 0;
	config.angle_z = 0;
	config.center = cv::Point2i(0,0);
	config.radius = 0;


	// invoke inplace constructor to initialize instance of PipelineGrid
	::new (g) PipelineGrid(config);
}
}
}

