#pragma once

#include "../util/opencv_fill_poly/fill_convex_poly.h"
#include "../util/opencv_fill_poly/line_iterator.h"

#include "PipelineGrid.h"

template <typename Func>
Func PipelineGrid::processOuterRingCoordinates(Func coordinateFunction) {
	static_assert(Grid::NUM_MIDDLE_CELLS > 0, "Grid must have at least one cell");

	if (!_innerWhiteRingCoordinates ||
		!_innerBlackRingCoordinates ||
		!_gridCellCoordinates[0]) // if the coordinates for the first cell have been calculated,
								  // the coordinates for all the other cells must have been
								  // calculated as well
	{
		throw std::runtime_error("Outer ring coordinates must be calculated after other coordinates");
	}

	return processCoordinates(_outerRingCoordinates, INDEX_OUTER_WHITE_RING, coordinateFunction);
}

template <typename Func>
Func PipelineGrid::processInnerWhiteRingCoordinates(Func coordinateFunction) {
	return processCoordinates(_innerWhiteRingCoordinates, INDEX_INNER_WHITE_SEMICIRCLE, coordinateFunction);
}

template <typename Func>
Func PipelineGrid::processInnerBlackRingCoordinates(Func coordinateFunction) {
	return processCoordinates(_innerBlackRingCoordinates, INDEX_INNER_BLACK_SEMICIRCLE, coordinateFunction);
}

template <typename Func>
Func PipelineGrid::processGridCellCoordinates(const size_t idx, Func coordinateFunction) {
	return processCoordinates(_gridCellCoordinates[idx], idx + Grid::INDEX_MIDDLE_CELLS_BEGIN, coordinateFunction);
}

template <typename Func>
Func PipelineGrid::processCoordinates(cached_coordinates_t& coordinates, const size_t idx, Func coordinateFunction) {
	// if already in cache, apply functor to previously cached coordinates
	if (coordinates) {
		for (cv::Point2i const& point : coordinates.get().areaCoordinates) {
			coordinateFunction(point);
		}
		return coordinateFunction;
	}

	std::pair<coordinates_t, Func> result = calculatePolygonCoordinates(idx, coordinateFunction);
	coordinates = result.first;

	return result.second;
}

template<typename Func>
class cacheSetter
{
public:
	explicit cacheSetter(const size_t idx, cv::Mat& idImage, PipelineGrid::coordinates_t& coordinateCache, Func& coordinateFunction, const cv::Point2i& idImageOffset, const cv::Point2i& gridCenter)
		: _idx(idx)
		, _idImage(idImage)
		, _coordinateCache(coordinateCache)
		, _coordinateFunction(coordinateFunction)
		, _idImageOffset(idImageOffset)
		, _gridCenter(gridCenter)
	{}

	inline void operator()(cv::Point coords) {
		// TODO: maybe speed up using raw pointer access
		_idImage.get().template at<uint8_t>(coords - _idImageOffset.get()) = _idx;
		_coordinateCache.get().areaCoordinates.push_back(coords + _gridCenter.get());
		(_coordinateFunction.get())(coords + _gridCenter.get());
	}

protected:
	size_t _idx;
	std::reference_wrapper<cv::Mat> _idImage;
	std::reference_wrapper<PipelineGrid::coordinates_t> _coordinateCache;
	std::reference_wrapper<Func> _coordinateFunction;
	std::reference_wrapper<const cv::Point2i> _idImageOffset;
	std::reference_wrapper<const cv::Point2i> _gridCenter;
};

template<typename Func>
class cacheSetterOuter : private cacheSetter<Func>
{
public:
	explicit cacheSetterOuter(const size_t idx, cv::Mat& idImage, PipelineGrid::coordinates_t& coordinateCache, Func& coordinateFunction, const cv::Point2i& idImageOffset, const cv::Point2i gridCenter)
		: cacheSetter<Func>(idx, idImage, coordinateCache, coordinateFunction, idImageOffset, gridCenter)
	{}

	inline void operator()(cv::Point coords) {
		uint8_t value = this->_idImage.get().template at<uint8_t>(coords - this->_idImageOffset.get());
		if (value == PipelineGrid::NOID) {
			cacheSetter<Func>::operator ()(coords);
		}
	}
};

template <typename Func>
std::pair<PipelineGrid::coordinates_t, Func> PipelineGrid::calculatePolygonCoordinates(const size_t idx, Func coordinateFunction)
{
	coordinates_t coordinates;

	// TODO: maybe move coordinateFunction
	if (idx == INDEX_OUTER_WHITE_RING) {
		cacheSetterOuter<Func> cacheFun(idx, _idImage, coordinates, coordinateFunction, _boundingBox.tl(), _center);
		cacheFun = heyho::convex_poly<cacheSetterOuter<Func>, heyho::line_iterator>(
					_boundingBox, _coordinates2D[idx], std::move(cacheFun), 8);
	} else {
		cacheSetter<Func> cacheFun(idx, _idImage, coordinates, coordinateFunction, _boundingBox.tl(), _center);
		cacheFun = heyho::convex_poly<cacheSetter<Func>, heyho::line_iterator>(
					_boundingBox, _coordinates2D[idx], std::move(cacheFun), 8);
	}

	return { coordinates, coordinateFunction };

//	// for each polygon, we speed up the calulation by iterating over the
//	// bounding box of the polygon instead of the bounding box of the grid
//	cv::Rect box = getPolygonBoundingBox(idx);




	//	// for each polygon, we speed up the calulation by iterating over the
	//	// bounding box of the polygon instead of the bounding box of the grid
	//	cv::Rect box = getPolygonBoundingBox(idx);

	//	// we actually have to draw the polygons in order to be able to extract the
	//	// coordinates. because the coordinates are internally centered at [0, 0],
	//	// we have to shift them initially.
	//	// this is unnecessary overhead and will be removed as soon as Tobi's
	//	// functions for polygon rasterization are finished.
	//	std::vector<cv::Point> shiftedPoints;
	//	shiftedPoints.reserve(_coordinates2D[idx].size());
	//	for (const cv::Point2i& origPoint : _coordinates2D[idx]) {
	//		shiftedPoints.push_back(origPoint - _boundingBox.tl() - box.tl());
	//	}

//	// draw polygon and edges on internal id image representation (see comment
//	// in header for _idImage
//	cv::Mat roi = _idImage(box);
//	cv::fillConvexPoly(roi, shiftedPoints, idx, 8);
//	CvHelper::drawPolyline(roi, shiftedPoints, CONTOUR_OFFSET + idx, false, cv::Point(), 3);

//	// iterate over roi and extract edge coordinates and store them in the
//	// associated vector
//	const cv::Point offset = box.tl() + _boundingBox.tl() + _center;
//	const int nRows = roi.rows;
//	const int nCols = roi.cols;
//	uint8_t* point;
//	for (int i = 0; i < nRows; ++i) {
//		point = roi.ptr<uint8_t>(i);
//		for (int j = 0; j < nCols; ++j) {
//			if (point[j] == CONTOUR_OFFSET + idx) coordinates.edgeCoordinates.push_back(cv::Point2i(j, i) + offset);
//		}
//	}

//	// when calculating the coordinates of the outer ring polygon, we have to
//	// make sure that a) the outer ring is drawn first and b) the other area
//	// are drawn before extracting the outer ring coordinates (because initially
//	// the outer ring polygon also contains all or the coordinates that actually
//	// belong to one of the other areas.
//	if (idx == INDEX_OUTER_WHITE_RING) {
//		_innerWhiteRingCoordinates = calculatePolygonCoordinates(INDEX_INNER_WHITE_SEMICIRCLE);
//		_innerBlackRingCoordinates = calculatePolygonCoordinates(INDEX_INNER_BLACK_SEMICIRCLE);
//		for (size_t cellIdx = Grid::INDEX_MIDDLE_CELLS_BEGIN; cellIdx < Grid::INDEX_MIDDLE_CELLS_END; ++cellIdx) {
//			_gridCellCoordinates[cellIdx - Grid::INDEX_MIDDLE_CELLS_BEGIN] = calculatePolygonCoordinates(cellIdx);
//		}
//	}

//	for (int i = 0; i < nRows; ++i) {
//		point = roi.ptr<uint8_t>(i);
//		for (int j = 0; j < nCols; ++j) {
//			if (point[j] == idx) coordinates.areaCoordinates.push_back(cv::Point2i(j, i) + offset);
//		}
//	}

//	return coordinates;
}

