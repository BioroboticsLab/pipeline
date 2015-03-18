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
}

