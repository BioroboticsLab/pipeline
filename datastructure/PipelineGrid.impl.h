#pragma once

#include "../util/opencv_fill_poly/fill_convex_poly.h"
#include "../util/opencv_fill_poly/line_iterator.h"

#include "PipelineGrid.h"

template <typename Func>
Func PipelineGrid::processOuterRingCoordinates(Func&& coordinateFunction) {
	static_assert(Grid::NUM_MIDDLE_CELLS > 0, "Grid must have at least one cell");

	if (!_innerWhiteRingCoordinates ||
	    !_innerBlackRingCoordinates ||
	    !_gridCellCoordinates[0]) // if the coordinates for the first cell have been calculated,
	                              // the coordinates for all the other cells must have been
	                              // calculated as well
	{
		throw std::runtime_error("Outer ring coordinates must be calculated after other coordinates");
	}

	return processCoordinates(_outerRingCoordinates, INDEX_OUTER_WHITE_RING, std::move(coordinateFunction));
}

template <typename Func>
Func PipelineGrid::processInnerWhiteRingCoordinates(Func&& coordinateFunction) {
	return processCoordinates(_innerWhiteRingCoordinates, INDEX_INNER_WHITE_SEMICIRCLE, std::move(coordinateFunction));
}

template <typename Func>
Func PipelineGrid::processInnerBlackRingCoordinates(Func&& coordinateFunction) {
	return processCoordinates(_innerBlackRingCoordinates, INDEX_INNER_BLACK_SEMICIRCLE, std::move(coordinateFunction));
}

template <typename Func>
Func PipelineGrid::processGridCellCoordinates(const size_t idx, Func&& coordinateFunction) {
	return std::move(processCoordinates(_gridCellCoordinates[idx], idx + Grid::INDEX_MIDDLE_CELLS_BEGIN, std::move(coordinateFunction)));
}

template <typename Func>
Func PipelineGrid::processCoordinates(cached_coordinates_t& coordinates, const size_t idx, Func&& coordinateFunction) {
	// if already in cache, apply functor to previously cached coordinates
	if (coordinates) {
		for (cv::Point2i const& point : coordinates.get().areaCoordinates) {
			coordinateFunction(point);
		}
		return std::move(coordinateFunction);
	}

	PipelineGrid::polygon_coords_return_t<Func> result = calculatePolygonCoordinates(idx, std::move(coordinateFunction));
	coordinates = std::move(result.coordinates);

	return std::move(result.coordinateFunction);
}

template <typename Func>
PipelineGrid::polygon_coords_return_t<Func> PipelineGrid::calculatePolygonCoordinates(const size_t idx, Func&& coordinateFunction)
{
	coordinates_t coordinates;

	// TODO: maybe move coordinateFunction
	if (idx == INDEX_OUTER_WHITE_RING) {
		cacheSetterOuter<Func> cacheFun(idx, _idImage, coordinates, coordinateFunction, _boundingBox.tl(), _center);
		cacheFun = heyho::convex_poly<heyho::line_iterator, cacheSetterOuter<Func>, heyho::no_boundaries_tag>(
		            std::move(cacheFun), heyho::no_boundaries_tag(), _coordinates2D[idx], 8);
	} else {
		cacheSetter<Func> cacheFun(idx, _idImage, coordinates, coordinateFunction, _boundingBox.tl(), _center);
		cacheFun = heyho::convex_poly<heyho::line_iterator, cacheSetter<Func>, heyho::no_boundaries_tag>(
		            std::move(cacheFun), heyho::no_boundaries_tag(), _coordinates2D[idx], 8);
	}

	return PipelineGrid::polygon_coords_return_t<Func>(std::move(coordinates), std::move(coordinateFunction));
}

template <typename Func>
Func PipelineGrid::processLineCoordinates(const cv::Point start, const cv::Point end, Func&& coordinateFunction) const
{
	static const int connectivity = 8;

	const int dx = end.x - start.x;
	const int dy = end.y - start.y;

	const double len = std::sqrt(dx * dx + dy * dy);

	// TODO: possible division by zero
	const double dxNorm = static_cast<double>(dx) / len;
	const double dyNorm = static_cast<double>(dy) / len;

	const uint8_t expSobelX = static_cast<uint8_t>(((dxNorm + 1.) / 2.) * 255.);
	const uint8_t expSobelY = static_cast<uint8_t>(((dyNorm + 1.) / 2.) * 255.);

	coordinateFunction.setExpectedSobelGradient(-expSobelY, expSobelX);

	auto it = heyho::line_iterator(heyho::no_boundaries_tag(), start, end, connectivity);

	for (; !it.end(); ++it) {
		coordinateFunction(it.pos());
	}

	return std::move(coordinateFunction);
}

template <typename Func>
Func PipelineGrid::processInnerLineCoordinates(Func&& coordinateFunction) const
{
	const cv::Point start = _coordinates2D[INDEX_INNER_LINE].back();
	const cv::Point end   = _coordinates2D[INDEX_INNER_LINE].front();

	coordinateFunction = processLineCoordinates(start + _center, end + _center, std::move(coordinateFunction));

	return std::move(coordinateFunction);
}

template <typename Func>
Func PipelineGrid::processEdgeCoordinates(const size_t idx, Func&& coordinateFunction) const
{
	const std::vector<cv::Point>& container = _coordinates2D[idx];
	assert(container.size() >= 2);

	std::vector<cv::Point>::const_iterator it = container.cbegin();
	cv::Point start = *it;

	for (; it != container.cend(); ++it) {
		cv::Point end = *it;

		coordinateFunction = processLineCoordinates(start + _center, end + _center, std::move(coordinateFunction));
		start = end;
	}

	return std::move(coordinateFunction);
}


template <typename Func>
Func PipelineGrid::processOuterRingEdgeCoordinates(Func&& coordinateFunction) const
{
	coordinateFunction = processEdgeCoordinates(INDEX_OUTER_WHITE_RING, std::move(coordinateFunction));

	return std::move(coordinateFunction);
}
