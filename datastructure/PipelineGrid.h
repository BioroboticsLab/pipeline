#pragma once

#include <array>
#include <bitset>

#include "serialization.hpp"

#include <boost/serialization/export.hpp>
#include <boost/optional.hpp>

#include "../common/Grid.h"
#include "../util/Util.h"

namespace pipeline {
typedef std::bitset<Grid::NUM_MIDDLE_CELLS> decoding_t;
}

class PipelineGrid: private Grid {
public:
	typedef struct {
		// rasterized coordinates of an grid area (outer ring etc.) for the
		// current grid configuration
		std::vector<cv::Point2i> areaCoordinates;
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
	/* only use for deserialization purposes! */
	explicit PipelineGrid() {}

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
			assert(Util::pointInBounds(_idImage.get().size(), coords - _idImageOffset));

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
			assert(Util::pointInBounds(this->_idImage.get().size(), coords - this->_idImageOffset));

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
	void save(Archive & ar, const unsigned int) const
	{
		// invoke serialization of the base class
		ar << BOOST_SERIALIZATION_NVP(_angle_x);
		ar << BOOST_SERIALIZATION_NVP(_angle_y);
		ar << BOOST_SERIALIZATION_NVP(_angle_z);
		ar << BOOST_SERIALIZATION_NVP(_center);
		ar << BOOST_SERIALIZATION_NVP(_radius);
	}

	template<class Archive>
	void load(Archive & ar, const unsigned int)
	{
		ar >> BOOST_SERIALIZATION_NVP(_angle_x);
		ar >> BOOST_SERIALIZATION_NVP(_angle_y);
		ar >> BOOST_SERIALIZATION_NVP(_angle_z);
		ar >> BOOST_SERIALIZATION_NVP(_center);
		ar >> BOOST_SERIALIZATION_NVP(_radius);

		prepare_visualization_data();
		resetCache();
	}

	template<class Archive>
	void serialize(Archive & ar, const unsigned int file_version)
	{
		boost::serialization::split_member(ar, *this, file_version);
	}
};

BOOST_CLASS_EXPORT_KEY(PipelineGrid)
