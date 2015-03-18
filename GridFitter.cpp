#include "GridFitter.h"

#include <boost/optional.hpp>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "datastructure/Tag.h"
#include "datastructure/TagCandidate.h"
#include "datastructure/PipelineGrid.h"
#include "util/Util.h"
#include "util/ThreadPool.h"
#include "source/tracking/algorithm/BeesBook/Common/Grid.h"
#include "source/utility/CvHelper.h"
#include "source/utility/util.h"

//#define DEBUG_GRIDFITTER

namespace pipeline {
GridFitter::GridFitter()
{}

void GridFitter::loadSettings(settings::gridfitter_settings_t &&settings)
{
	_settings = std::move(settings);
}

std::vector<Tag> GridFitter::process(std::vector<Tag> &&taglist)
{
#ifdef DEBUG_GRIDFITTER
	// run GridFitter in a single thread when debugging is enabled to get
	// better and more consistent stack traces etc.
	for (Tag& tag : taglist) {
		tag.setValid(false);
		for (TagCandidate& candidate : tag.getCandidates()) {
				std::vector<PipelineGrid> grids = fitGrid(tag, candidate);
				if (!grids.empty()) { tag.setValid(true); };
				candidate.setGrids(std::move(grids));
		}
	}
#else
	// otherwise, use double the number of available cores of thread to process
	// the tag candidates
    static const size_t numThreads = std::thread::hardware_concurrency() ?
                std::thread::hardware_concurrency() * 2 : 1;
    ThreadPool pool(numThreads);

    std::vector<std::future<void>> results;
	for (Tag& tag : taglist) {
		tag.setValid(false);
		results.emplace_back(
		    pool.enqueue([&] {
				for (TagCandidate& candidate : tag.getCandidates()) {
					std::vector<PipelineGrid> grids = fitGrid(tag, candidate);
					if (!grids.empty()) { tag.setValid(true); };
					candidate.setGrids(std::move(grids));
				}
		}));
	}

	// wait for all threads to finish
    for (auto && result : results) result.get();
#endif

	// remove tags without any fitted grids
	taglist.erase(std::remove_if(taglist.begin(), taglist.end(), [](Tag& tag) { return !tag.isValid(); }), taglist.end());

    return std::move(taglist);
}

// TODO: remove or move into util namespace
cv::Mat GridFitter::calculateHistogram(const cv::Mat& roi, const Ellipse& ellipse_orig) const
{
    cv::Mat hist;
    const cv::Mat images[1] = { roi };
    const int channels[1] = { 0 };
    const int bins = 256;
    const int histsize[1] = { bins };
    const float range[] = { 0.f, 256.f };
    const float* ranges[] = { range };
    cv::calcHist(images, 1, channels, ellipse_orig.getMask(), hist, 1, histsize, ranges);

    float max = 0;
    for (size_t idx = 0; idx < bins; ++idx) {
        float val = hist.at<float>(idx);
        max = std::max(max, val);
    }

    int height = static_cast<int>(max);
    cv::Mat histImg = cv::Mat(height, bins, CV_8UC3, cv::Scalar::all(255));
    for (size_t idx = 0; idx < bins; ++idx) {
        float val = hist.at<float>(idx);
        cv::line(histImg, cv::Point2i(idx, height), cv::Point2i(idx, height - static_cast<int>(val)), cv::Scalar::all(0));
    }

    return histImg;
}

void GridFitter::visualizeDebug(std::multiset<candidate_t> const& bestGrids, const cv::Size2i roiSize, const Tag& tag, cv::Mat const& binarizedROI, std::string winName)
{
    cv::Mat binarizedROICpy;
    cv::cvtColor(binarizedROI, binarizedROICpy, CV_GRAY2BGR);

    cv::Mat cannyImg;
    cv::cvtColor(tag.getCannySubImage(), cannyImg, CV_GRAY2BGR);

    const size_t to = std::min(_settings.get_gradient_num_results(), bestGrids.size());
    size_t idx = 0;
    for (candidate_t const& candidate : bestGrids) {
        std::vector<cv::Mat> images;

        PipelineGrid grid(candidate.config);

        images.push_back(tag.getOrigSubImage());

        cv::Mat origCopy;
        tag.getOrigSubImage().copyTo(origCopy);
        pipeline::Ellipse const& ell = tag.getCandidates().front().getEllipse();
        cv::ellipse(origCopy, ell.getCen(), ell.getAxis(), ell.getAngle(), 0, 360, cv::Scalar(0, 255, 0), 2);
        images.push_back(origCopy);

        images.push_back(cannyImg);
        images.push_back(binarizedROICpy);
        images.push_back(grid.getProjectedImage(roiSize));

        cv::Mat blendedBin;
        cv::addWeighted(binarizedROICpy, 0.6, grid.getProjectedImage(roiSize), 0.4, 0.0, blendedBin);
        images.push_back(blendedBin);

        cv::Mat blended;
        cv::addWeighted(tag.getOrigSubImage(), 0.8, grid.getProjectedImage(roiSize), 0.2, 0.0, blended);
        images.push_back(blended);

        cv::Mat cannyBlended;
        cv::addWeighted(cannyImg, 0.8, grid.getProjectedImage(roiSize), 0.2, 0.0, cannyBlended);
        images.push_back(cannyBlended);

        cv::Mat origCopyOverlay;
        tag.getOrigSubImage().copyTo(origCopyOverlay);
        grid.drawContours(origCopyOverlay, 0.5);
        images.push_back(origCopyOverlay);

        const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

        std::string title(winName + " (error: " + std::to_string(candidate.error) + ")");
        cv::namedWindow(title);
        cv::imshow(title, canvas);

        ++idx;
        if (idx == to) break;
    }

    if (!bestGrids.empty()) {
        bool cont = true;
        while (cont) {
            const char c = cv::waitKey();
            if (c == 'd') {
                    cv::destroyAllWindows();
                    cont = false;
            } else if (c == 'c') {
                    cont = false;
            }
        }
    }
}

GridFitter::candidate_set GridFitter::getInitialCandidates(const cv::Mat &binarizedROI, const cv::Mat& edgeROI, const Ellipse& ellipse_orig, const cv::Mat &roi)
{
	static const auto initial_rotations        = util::linspace<double>(0, 2 * CV_PI, 64);
	static const auto initial_position_offsets = util::linspace<int>(-4, 4, 9);

    // initial search for gradiant descent candidates in ellipse parameter space
	// note that the position offsets have to be evaluated in the inner loop
	// because shifting a known grid configuration is much faster than
	// recalculating all coordinates.
    candidate_set gridCandidates;
    for (const double rotation : initial_rotations) {
		// get the best candidates for the current rotation
        candidate_set candidatesForRotation;
		// estimate grid parameters from ellipse -> two possible candidates
        const std::array<PipelineGrid::gridconfig_t, 2> configCandidates =
                Util::gridCandidatesFromEllipse(ellipse_orig, rotation);
		for (PipelineGrid::gridconfig_t const& config : configCandidates) {
            PipelineGrid grid(config);
            for (const int pos_x_offset : initial_position_offsets) {
                for (const int pos_y_offset : initial_position_offsets) {
                    grid.setCenter({config.center.x + pos_x_offset, config.center.y + pos_y_offset});
                    const double error = evaluateCandidate(grid, roi, binarizedROI, edgeROI, _settings);
                    candidatesForRotation.insert({error, grid.getConfig()});
                }
            }
        }
		// for each rotation, insert the best candiate into gridCandidates
        gridCandidates.insert(*candidatesForRotation.begin());
    }

	return gridCandidates;
}

std::vector<PipelineGrid> GridFitter::fitGrid(const Tag& tag, const TagCandidate &candidate)
{
	const Ellipse& ellipse_orig = candidate.getEllipse();

    // region of interest of tag candidate
    const cv::Size2i roiSize = tag.getBox().size();
    cv::Mat roi;
    // TODO: shouldn't be BGR in the first place
    cv::cvtColor(tag.getOrigSubImage(), roi, CV_BGR2GRAY);

    cv::Mat binarizedROI(roiSize, CV_8UC1);
    cv::adaptiveThreshold(roi, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, _settings.get_adaptive_block_size(),
                          _settings.get_adaptive_c());

    const cv::Mat& edgeROI = tag.getCannySubImage();

	// get initial candidates using brute force search over small number of
	// rotations and position offsets
	candidate_set gridCandidates = getInitialCandidates(binarizedROI, edgeROI, ellipse_orig, roi);

	// optimize best candidates using gradient descent
	GradientDescent optimizer(gridCandidates, roi, binarizedROI, edgeROI, _settings);
	optimizer.optimize();

	const candidate_set& bestGrids = optimizer.getBestGrids();

#ifdef DEBUG_GRIDFITTER
    std::cout << "min initial candidate error: " << gridCandidates.begin()->error << std::endl;
    std::cout << "min final candidate error: " << bestGrids.begin()->error << std::endl;

    visualizeDebug(gridCandidates, roiSize, tag, binarizedROI, "initial fit");
    visualizeDebug(bestGrids, roiSize, tag, binarizedROI, "best fit");
#endif

	// return the settings.numResults best candidates
    std::vector<PipelineGrid> results;
    {
        const size_t to = std::min(_settings.get_gradient_num_results(), bestGrids.size());
        size_t idx = 0;
        for (candidate_t const& gridCandidate : bestGrids) {
            PipelineGrid::gridconfig_t const& config = gridCandidate.config;
            results.emplace_back(config.center + tag.getBox().tl(), config.radius, config.angle_z, config.angle_y, config.angle_x);

            ++idx;
            if (idx == to) break;
        }
    }

    return results;
}

double GridFitter::evaluateCandidate(PipelineGrid& grid, cv::Mat const& roi, cv::Mat const& binarizedROI, cv::Mat const& edgeROI,  settings::gridfitter_settings_t &settings)
{
	double error = 0;

	const cv::Rect boundingBox = grid.getBoundingBox();
	// return max error if either width or height is zero
	if (!boundingBox.area()) return std::numeric_limits<double>::max();
	// also return max error if grid bounding box is not within roi
	if (boundingBox.x < 0 || boundingBox.y < 0) return std::numeric_limits<double>::max();
	if (boundingBox.x + boundingBox.width >= roi.rows ||
		boundingBox.y + boundingBox.height >= roi.cols) return std::numeric_limits<double>::max();

	{
		error_counter_t<expected_white_error_fun_t> errorFun(binarizedROI);
		errorFun = grid.processInnerWhiteRingCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError();
	}

	{
		error_counter_t<expected_black_error_fun_t> errorFun(binarizedROI);
		errorFun = grid.processInnerBlackRingCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError();
	}

	for (size_t cellIdx = 0; cellIdx < Grid::NUM_MIDDLE_CELLS; ++cellIdx) {
		variance_online_calculator_t errorFun(binarizedROI);
		errorFun = grid.processGridCellCoordinates(cellIdx, std::move(errorFun));
		error += errorFun.getNormalizedVariance() / Grid::NUM_MIDDLE_CELLS;
	}

	{
		error_counter_t<expected_white_error_fun_t> errorFun(binarizedROI);
		errorFun = grid.processOuterRingCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError();
	}

	error /= 4.;

//    // define region of interest based on bounding box of grid candidate.
//    // from now on, we do all calculations on this roi to speed up all
//    // computations that have to iterate over the (sub)image
//    cv::Mat subROI;
//    subROI = roi(boundingBox);

//    // region of interest of binarized and edge image
//    cv::Mat subBinarized = binarizedROI(boundingBox);
//    cv::Mat subEdges     = edgeROI(boundingBox);

//    // get coordinates of current grid configuration
//    const PipelineGrid::coordinates_t& outerRingCoordinates  = grid.processOuterRingCoordinates();
//    const PipelineGrid::coordinates_t& innerWhiteCoordinates = grid.processInnerWhiteRingCoordinates();
//    const PipelineGrid::coordinates_t& innerBlackCoordinates = grid.processInnerBlackRingCoordinates();

//    // compare outer circle
//    size_t outerCircleError = 0;
//    for (const cv::Point2i coords : outerRingCoordinates.areaCoordinates) {
//        const uint8_t value = subBinarized.at<uint8_t>(coords - boundingBox.tl());
//        // no branching => better performance
//        outerCircleError += 255 - value;
//    }
//    if (outerRingCoordinates.areaCoordinates.size())
//        error += settings.get_err_func_alpha_outer() * (outerCircleError / outerRingCoordinates.areaCoordinates.size()) / 255.;

//    size_t outerCircleEdgeError = 0;
//    for (const cv::Point2i coords : outerRingCoordinates.edgeCoordinates) {
//        const uint8_t value      = subEdges.at<uint8_t>(coords - boundingBox.tl());
//        outerCircleEdgeError += 255 - value;
//    }
//    if (outerRingCoordinates.edgeCoordinates.size())
//        error += settings.get_err_func_alpha_outer_edge() * (outerCircleEdgeError / outerRingCoordinates.edgeCoordinates.size()) / 255.;

//    // compare inner white semicircle
//    size_t innerWhiteError = 0;
//    for (const cv::Point2i coords : innerWhiteCoordinates.areaCoordinates) {
//        const uint8_t value      = subBinarized.at<uint8_t>(coords - boundingBox.tl());
//        innerWhiteError += 255 - value;
//    }
//    if (innerWhiteCoordinates.areaCoordinates.size())
//        error += settings.get_err_func_alpha_inner() * (innerWhiteError / innerWhiteCoordinates.areaCoordinates.size()) / 255.;

//    // compare inner black semicircle
//    size_t innerBlackError = 0;
//    for (const cv::Point2i coords : innerBlackCoordinates.areaCoordinates) {
//        const uint8_t value      = subBinarized.at<uint8_t>(coords - boundingBox.tl());
//        innerBlackError += value;
//    }
//    if (innerBlackCoordinates.areaCoordinates.size())
//        error += settings.get_err_func_alpha_inner() * (innerBlackError / innerBlackCoordinates.areaCoordinates.size()) / 255;

//	// at the moment, there is no datastructure containg only the coordinates
//	// between the two inner semicircles. they can be calculated by doing a
//	// set intersection of the coordinates of the two semicircles, but this is
//	// very slow.
//    // TODO: more efficient implementation for this!
////    auto vectorCmp = [&](const cv::Point2i& first, const cv::Point2i second) {
////        if (first.x != second.x) return first.x < second.x;
////        return first.y < second.y;
////    };
////    std::vector<cv::Point2i> blackCoordinates = innerBlackCoordinates.edgeCoordinates;
////    std::sort(blackCoordinates.begin(), blackCoordinates.end(), vectorCmp);
////    std::vector<cv::Point2i> whiteCoordinates = innerWhiteCoordinates.edgeCoordinates;
////    std::sort(whiteCoordinates.begin(), whiteCoordinates.end(), vectorCmp);

////    std::vector<cv::Point2i> intersection;
////    std::set_intersection(blackCoordinates.begin(), blackCoordinates.end(),
////                          whiteCoordinates.begin(), whiteCoordinates.end(),
////                          std::back_inserter(intersection), vectorCmp);
////    size_t innerEdgeError = 0;
////    for (const cv::Point2i coords : intersection) {
////        const uint8_t value      = subEdges.at<uint8_t>(coords - boundingBox.tl());
////        innerEdgeError += value;
////    }
////    if (intersection.size())
////        error += settings.alpha_inner_edge * (innerEdgeError / intersection.size()) / 255;

//    // add variance of each grid cells to error
//    for (size_t cellIdx = 0; cellIdx < Grid::NUM_MIDDLE_CELLS; ++cellIdx) {
//        PipelineGrid::coordinates_t const& cellCoordinates = grid.processGridCellCoordinates(cellIdx);
//        if (cellCoordinates.areaCoordinates.size() > 1) {
//            // two-pass variance calculation. first pass calculates mean of
//            // points in cell, second pass calculates variance based on mean.
//            // TODO: evaluate whether a one-pass method significantly improves
//            // performance and whether a more numerically stable algorithm is
//            // required.
//            // see: http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
//            size_t sum = 0;
//            for (const cv::Point2i coords : cellCoordinates.areaCoordinates) {
//                const uint8_t value      = subBinarized.at<uint8_t>(coords);
//                sum += value;
//            }
//            const double mean = static_cast<double>(sum) / static_cast<double>(cellCoordinates.areaCoordinates.size());

//            double sum2 = 0;
//            for (const cv::Point2i coords : cellCoordinates.areaCoordinates) {
//                const double value       = static_cast<double>(subBinarized.at<uint8_t>(coords));
//                sum2 += (value - mean) * (value - mean);
//            }
//            const double variance = (sum2 / (cellCoordinates.areaCoordinates.size() - 1));

//            error += settings.get_err_func_alpha_variance() * (variance / 255.);
//        }
//    }

	return error;
}

GridFitter::GradientDescent::GradientDescent(const GridFitter::candidate_set &initialCandidates, const cv::Mat &roi, const cv::Mat &binarizedRoi, const cv::Mat& edgeRoi,  settings::gridfitter_settings_t &settings)
    : _initialCandidates(initialCandidates)
    , _settings(settings)
    , _roi(roi)
    , _binarizedRoi(binarizedRoi)
    , _edgeRoi(edgeRoi)
{}

void GridFitter::GradientDescent::optimize()
{
    const size_t num = std::min(_settings.get_gradient_num_initial(), _initialCandidates.size());
    candidate_set::iterator candidate_it = _initialCandidates.begin();
	// iterate over the settings.numInitial best initial candidates
    for (size_t idx = 0; idx < num; ++idx) {
        candidate_t candidate = *candidate_it;
        const PipelineGrid::gridconfig_t& initial_config = candidate.config;

        size_t iteration = 0;
        PipelineGrid grid(initial_config);
        PipelineGrid::gridconfig_t config = initial_config;
        double error = evaluateCandidate(grid, _roi, _binarizedRoi, _edgeRoi, _settings);
        storeConfig(error, config);

		// gradient descent
		while ((error > _settings.get_gradient_error_threshold()) && (iteration < _settings.get_gradient_max_iterations())) {
			double const initerror = error;

			std::tie(error, config) = step(config, error, SCALE);
			std::tie(error, config) = step(config, error, POSX);
			std::tie(error, config) = step(config, error, POSY);
			std::tie(error, config) = step(config, error, ANGLE_X);
			std::tie(error, config) = step(config, error, ANGLE_Y);
			std::tie(error, config) = step(config, error, ANGLE_Z);

            ++iteration;
			// abort gradient descent if error measurement did not improve by
			// a significant amount during the last six steps
			// TODO: bestGrid should be specific to current candidate!
			assert(!_bestGrids.empty());
			if (std::abs(initerror - _bestGrids.begin()->error) < 0.001) break;

			// instead of continuing the next iteration with the result of the
			// last step, use the best currently found config from now on.
			error  = _bestGrids.begin()->error;
			config = _bestGrids.begin()->config;
		}
        ++candidate_it;
	}
}

std::pair<double, PipelineGrid::gridconfig_t> GridFitter::GradientDescent::step(PipelineGrid::gridconfig_t const& config, double error, const GridFitter::GradientDescent::StepParameter param)
{
	// when adjusting one of the position parameters, we can not adjust the step
	// size based on the difference between of the errors and the learning rate
	// (because the coordinates are discrete values).
	// instead, in each step we only change the position by either 1 or -1.
	// furthermore, if the error does not improve after a position change,
	// instead of just applying the reverse direction, we check if the error
	// acutally improves when going in the reverse direction.

	static const std::array<int, 2> directions {-1, 1};

	const double alpha     = _settings.get_alpha();
	const double eps_scale = _settings.get_eps_scale();
	const int eps_pos      = _settings.get_eps_pos();
	const double eps_angle = _settings.get_eps_angle();

	for (int direction : directions) {
		PipelineGrid::gridconfig_t newConfig(config);
		// adjust parameter
		switch (param) {
		case SCALE:
			newConfig.radius = newConfig.radius + direction * eps_scale;
			break;
		case POSX:
			newConfig.center = newConfig.center + cv::Point2i(direction * eps_pos, 0);
			break;
		case POSY:
			newConfig.center = newConfig.center + cv::Point2i(0, direction * eps_pos);
			break;
		case ANGLE_X:
			newConfig.angle_x = newConfig.angle_x + direction * eps_angle;
			break;
		case ANGLE_Y:
			newConfig.angle_y = newConfig.angle_y + direction * eps_angle;
			break;
		case ANGLE_Z:
			newConfig.angle_z = newConfig.angle_z + direction * eps_angle;
			break;
		default:
			assert(false);
			break;
		}

		// TODO: don't construct new Grid in each step
		PipelineGrid newGrid(newConfig);
		double newError = evaluateCandidate(newGrid, _roi, _binarizedRoi, _edgeRoi, _settings);

		// adjust parameter based on learning rate and new error
		if (param != POSX && param != POSY) {
			switch (param) {
			case SCALE:
				newConfig.radius = config.radius + direction * alpha * (error - newError);
				break;
			case ANGLE_X:
				newConfig.angle_x = config.angle_x + direction * alpha * (error - newError);
				break;
			case ANGLE_Y:
				newConfig.angle_y = config.angle_y + direction * alpha * (error - newError);
				break;
			case ANGLE_Z:
				newConfig.angle_z = config.angle_z + direction * alpha * (error - newError);
				break;
			default:
				break;
			}

			PipelineGrid newGrid(newConfig);
			newError = evaluateCandidate(newGrid, _roi, _binarizedRoi, _edgeRoi, _settings);

			storeConfig(newError, newConfig);
			return {newError, newConfig};
		} else if (newError < error) {
			storeConfig(newError, newConfig);
			return {newError, newConfig};
		}
	}
	return {error, config};
}

void GridFitter::GradientDescent::storeConfig(const double error, const PipelineGrid::gridconfig_t& config)
{
	_bestGrids.insert({error, config});

	if (_bestGrids.size() > _settings.get_gradient_num_results()) {
		_bestGrids.erase(std::prev(_bestGrids.end()));
	}
}

double GridFitter::variance_twopass_calculator_t::getNormalizedVariance() const
{
	if (_pixelNum < 2) return 0.;

	const double mean = static_cast<double>(_sum) / static_cast<double>(_pixelNum);

	double sum = 0.;
	for (const uint8_t value : _values) {
		std::cout << std::to_string(value) << std::endl;
		sum += (static_cast<double>(value) - mean) * (static_cast<double>(value) - mean);
	}

	static const double maxVariance = (255. * 255.) / 4.;
	const double variance = sum / (_pixelNum - 1);

	return variance / maxVariance;
}

double GridFitter::variance_online_calculator_t::getNormalizedVariance() const
{
	if (_pixelNum < 2) return 0.;

	// http://math.stackexchange.com/q/83046
	static const double maxVariance = (255. * 255.) / 4.;
	const double variance           = (_m2 / (_pixelNum - 1));

	//return variance;
	return (variance / maxVariance);
}

}
