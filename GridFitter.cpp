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

#include "datastructure/PipelineGrid.impl.h"

namespace pipeline {
GridFitter::GridFitter()
{
	cacheSettings();
}

void GridFitter::loadSettings(settings::gridfitter_settings_t &&settings)
{
	_settings = std::move(settings);
	cacheSettings();
}

void GridFitter::cacheSettings()
{
	_settings_cache.err_func_alpha_inner      = _settings.get_err_func_alpha_inner();
	_settings_cache.err_func_alpha_inner_edge = _settings.get_err_func_alpha_inner_edge();
	_settings_cache.err_func_alpha_outer      = _settings.get_err_func_alpha_outer();
	_settings_cache.err_func_alpha_outer_edge = _settings.get_err_func_alpha_outer_edge();
	_settings_cache.err_func_alpha_variance   = _settings.get_err_func_alpha_variance();
	_settings_cache.gradient_num_initial      = _settings.get_gradient_num_initial();
	_settings_cache.gradient_num_results      = _settings.get_gradient_num_results();
	_settings_cache.gradient_error_threshold  = _settings.get_gradient_error_threshold();
	_settings_cache.gradient_max_iterations   = _settings.get_gradient_max_iterations();
	_settings_cache.alpha                     = _settings.get_alpha();
	_settings_cache.eps_scale                 = _settings.get_eps_scale();
	_settings_cache.eps_angle                 = _settings.get_eps_angle();
	_settings_cache.eps_pos                   = _settings.get_eps_pos();
}

std::vector<Tag> GridFitter::process(std::vector<Tag> &&taglist)
{
#if defined(DEBUG_GRIDFITTER) || !defined(NDEBUG) || defined(PipelineStandalone)
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

void GridFitter::visualizeDebug(std::multiset<candidate_t> const& grids, const cv::Mat& roi, const cv::Size2i roiSize, const cv::Mat& edgeRoiX, const cv::Mat& edgeRoiY, const Tag &tag, cv::Mat const& binarizedROI, std::string winName, const size_t numBest)
{
	cv::Mat binarizedROICpy;
	cv::cvtColor(binarizedROI, binarizedROICpy, CV_GRAY2BGR);

	cv::Mat edgeX;
	cv::cvtColor(edgeRoiX, edgeX, CV_GRAY2BGR);

	cv::Mat edgeY;
	cv::cvtColor(edgeRoiY, edgeY, CV_GRAY2BGR);

	cv::Mat roiCpy;
	cv::cvtColor(roi, roiCpy, CV_GRAY2BGR);

	const size_t to = std::min(grids.size(), numBest);
	size_t idx = 0;
	for (candidate_t const& candidate : grids) {
		std::vector<cv::Mat> images;

		PipelineGrid grid(candidate.config);

		images.push_back(tag.getOrigSubImage());
		images.push_back(roiCpy);

		cv::Mat origCopy;
		roiCpy.copyTo(origCopy);
		pipeline::Ellipse const& ell = tag.getCandidates().front().getEllipse();
		cv::ellipse(origCopy, ell.getCen(), ell.getAxis(), ell.getAngle(), 0, 360, cv::Scalar(0, 255, 0), 2);
		images.push_back(origCopy);

		images.push_back(edgeX);
		images.push_back(edgeY);
		images.push_back(binarizedROICpy);
		images.push_back(grid.getProjectedImage(roiSize));

		cv::Mat blendedBin;
		cv::addWeighted(binarizedROICpy, 0.6, grid.getProjectedImage(roiSize), 0.4, 0.0, blendedBin);
		images.push_back(blendedBin);

		cv::Mat blended;
		cv::addWeighted(tag.getOrigSubImage(), 0.8, grid.getProjectedImage(roiSize), 0.2, 0.0, blended);
		images.push_back(blended);

		cv::Mat cannyBlendedX;
		edgeX.copyTo(cannyBlendedX);
		grid.drawContours(cannyBlendedX, 0.9, cv::Vec3b(150, 200, 170));
		images.push_back(cannyBlendedX);

		cv::Mat cannyBlendedY;
		edgeY.copyTo(cannyBlendedY);
		grid.drawContours(cannyBlendedY, 0.9, cv::Vec3b(150, 200, 170));
		images.push_back(cannyBlendedY);

		cv::Mat origCopyOverlay;
		roiCpy.copyTo(origCopyOverlay);
		grid.drawContours(origCopyOverlay, 0.5);
		images.push_back(origCopyOverlay);

		const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

		std::string title(winName + " " + std::to_string(idx) + " (error: " + std::to_string(candidate.error) + ")");
		cv::namedWindow(title);
		cv::imshow(title, canvas);

		++idx;
		if (idx == to) break;
	}

	if (!grids.empty()) {
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

GridFitter::candidate_set GridFitter::getInitialCandidates(const cv::Mat &binarizedROI, const cv::Mat& sobelXRoi, const cv::Mat& sobelYRoi, const Ellipse& ellipse_orig, const cv::Mat &roi)
{
	static const auto initial_rotations        = util::linspace<double>(0, 2 * CV_PI, 32);
	static const auto initial_position_offsets = util::linspace<int>(-3, 3, 7);

	// initial search for gradiant descent candidates in ellipse parameter space
	// note that the position offsets have to be evaluated in the inner loop
	// because shifting a known grid configuration is much faster than
	// recalculating all coordinates.
	candidate_set gridCandidates;
	for (const double rotation : initial_rotations) {
		// get the best candidates for the current rotation
		candidate_set candidatesForRotation;
		// estimate grid parameters from ellipse -> two possible candidates
		const std::array<Util::gridconfig_t, 2> configCandidates =
		        Util::gridCandidatesFromEllipse(ellipse_orig, rotation);
		for (Util::gridconfig_t const& config : configCandidates) {
			PipelineGrid grid(config);
			for (const int pos_x_offset : initial_position_offsets) {
				for (const int pos_y_offset : initial_position_offsets) {
					grid.setCenter({config.center.x + pos_x_offset, config.center.y + pos_y_offset});
					const double error = evaluateCandidate(grid, roi, binarizedROI, sobelXRoi, sobelYRoi, _settings_cache);
					candidatesForRotation.insert({error, grid.getConfig()});
				}
			}
			// for each rotation and ellipse candidate, insert the best candiate into gridCandidates
			gridCandidates.insert(*candidatesForRotation.begin());
		}
	}

	return gridCandidates;
}

std::vector<PipelineGrid> GridFitter::fitGrid(const Tag& tag, const TagCandidate &candidate)
{
#ifdef DEBUG_GRIDFITTER
	cv::destroyAllWindows();
#endif

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

	cv::Mat blurredRoi;
	cv::GaussianBlur(roi, blurredRoi, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

	auto getSobel = [&](const int dx, const int dy) {
		cv::Mat sobelImg;
		cv::Scharr(blurredRoi, sobelImg, CV_32F, dx, dy);

		double minVal, maxVal;
		cv::minMaxLoc(sobelImg, &minVal, &maxVal);
		sobelImg.convertTo(sobelImg, CV_8U, 255.0/(maxVal - minVal), -minVal * 255.0/(maxVal - minVal));

		return sobelImg;
	};

	cv::Mat edgeRoiX = getSobel(1, 0);
	cv::Mat edgeRoiY = getSobel(0, 1);

	// TODO: constant/setting for axis border
	// TODO: add sanity checks
	const cv::Mat ellipseMask = candidate.getEllipse().getMask(cv::Size(10, 10));
	cv::Mat equalizedRoi;
	roi.copyTo(equalizedRoi, ellipseMask);
	cv::equalizeHist(equalizedRoi, roi);

	// get initial candidates using brute force search over small number of
	// rotations and position offsets
	candidate_set gridCandidates = getInitialCandidates(binarizedROI, edgeRoiX, edgeRoiY, ellipse_orig, roi);

#ifdef DEBUG_GRIDFITTER
	visualizeDebug(gridCandidates, roi, roiSize, edgeRoiX, edgeRoiY, tag, binarizedROI, "candidate", _settings.get_gradient_num_initial());
#endif

	// optimize best candidates using gradient descent
	GradientDescent optimizer(gridCandidates, roi, binarizedROI, edgeRoiX, edgeRoiY, _settings_cache);
	optimizer.optimize();

	const candidate_set& bestGrids = optimizer.getBestGrids();

#ifdef DEBUG_GRIDFITTER
	std::cout << "min initial candidate error: " << gridCandidates.begin()->error << std::endl;
	std::cout << "min final candidate error: " << bestGrids.begin()->error << std::endl;

	visualizeDebug(bestGrids, roi, roiSize, edgeRoiX, edgeRoiY, tag, binarizedROI, "best fit", _settings.get_gradient_num_results());
#endif

	// return the settings.numResults best candidates
	std::vector<PipelineGrid> results;
	{
		const size_t to = std::min(_settings.get_gradient_num_results(), bestGrids.size());
		size_t idx = 0;
		for (candidate_t const& gridCandidate : bestGrids) {
			Util::gridconfig_t const& config = gridCandidate.config;
			results.emplace_back(config.center + tag.getBox().tl(), config.radius, config.angle_z, config.angle_y, config.angle_x);

			++idx;
			if (idx == to) break;
		}
	}

	return results;
}

double GridFitter::evaluateCandidate(PipelineGrid& grid, cv::Mat const& roi, cv::Mat const& binarizedROI, cv::Mat const& sobelXRoi, const cv::Mat& sobelYRoi, const settings_cache_t &settings)
{
	double error = 0;

	const cv::Rect boundingBox = grid.getBoundingBox();
	// return max error if either width or height is zero
	if (!boundingBox.area()) return std::numeric_limits<double>::max();
	// also return max error if grid bounding box is not within roi
	if (boundingBox.x < 0 || boundingBox.y < 0) return std::numeric_limits<double>::max();
	if (boundingBox.x + boundingBox.width >= roi.rows ||
	    boundingBox.y + boundingBox.height >= roi.cols) return std::numeric_limits<double>::max();

	enum ROI {
		BINARY = 0,
		GRAYSCALE
	};

	static const ROI roiKind = ROI::GRAYSCALE;
	static const double numErrorMeasurements = 6.;

	const cv::Mat& selectedRoi = roiKind == ROI::BINARY ? binarizedROI : roi;

	{
		error_counter_t<expected_white_error_fun_t> errorFun(selectedRoi);
		errorFun = grid.processInnerWhiteRingCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError() * settings.err_func_alpha_inner;
	}

	{
		error_counter_t<expected_black_error_fun_t> errorFun(selectedRoi);
		errorFun = grid.processInnerBlackRingCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError() * settings.err_func_alpha_inner;
	}

	for (size_t cellIdx = 0; cellIdx < Grid::NUM_MIDDLE_CELLS; ++cellIdx) {
		variance_online_calculator_t errorFun(selectedRoi);
		errorFun = grid.processGridCellCoordinates(cellIdx, std::move(errorFun));
		error += (errorFun.getNormalizedVariance() / Grid::NUM_MIDDLE_CELLS) * settings.err_func_alpha_variance;
	}

	{
		error_counter_t<expected_white_error_fun_t> errorFun(selectedRoi);
		errorFun = grid.processOuterRingCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError() * settings.err_func_alpha_outer;
	}

	{
		sobel_error_counter_t errorFun(sobelXRoi, sobelYRoi);
		errorFun = grid.processOuterRingEdgeCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError() * settings.err_func_alpha_outer_edge;
	}

	{
		sobel_error_counter_t errorFun(sobelXRoi, sobelYRoi);
		errorFun = grid.processInnerLineCoordinates(std::move(errorFun));
		error += errorFun.getNormalizedError() * settings.err_func_alpha_inner_edge;
	}

	error /= numErrorMeasurements;

	return error;
}

GridFitter::GradientDescent::GradientDescent(const GridFitter::candidate_set &initialCandidates, const cv::Mat &roi, const cv::Mat &binarizedRoi, const cv::Mat& edgeRoiX, const cv::Mat &edgeRoiY,  const settings_cache_t& settings)
    : _initialCandidates(initialCandidates)
    , _settings(settings)
    , _roi(roi)
    , _binarizedRoi(binarizedRoi)
    , _edgeRoiX(edgeRoiX)
    , _edgeRoiY(edgeRoiY)
	, _random_device()
	, _random_engine(_random_device())
{}

void GridFitter::GradientDescent::optimize()
{
	const size_t num = std::min(_settings.gradient_num_initial, _initialCandidates.size());
	candidate_set::iterator candidate_it = _initialCandidates.begin();
	// iterate over the settings.numInitial best initial candidates
	for (size_t idx = 0; idx < num; ++idx) {
		candidate_t candidate = *candidate_it;
		const Util::gridconfig_t& initial_config = candidate.config;

		size_t iteration = 0;
		PipelineGrid grid(initial_config);
		Util::gridconfig_t config = initial_config;
		double error = evaluateCandidate(grid, _roi, _binarizedRoi, _edgeRoiX, _edgeRoiY, _settings);
		storeConfig(error, config);

		candidate_set bestGridsForCandidate;
		bestGridsForCandidate.insert(candidate);

		std::array<StepParameter, 6> parameters { SCALE, POSX, POSY, ANGLE_X, ANGLE_Y, ANGLE_Z };

		// gradient descent
		size_t numWithoutImprovement = 0;
		while ((error > _settings.gradient_error_threshold) && (iteration < _settings.gradient_max_iterations)) {
			double const initerror = error;

			// shuffle order of parameters
			std::shuffle(parameters.begin(), parameters.end(), _random_engine);
			for (const StepParameter param : parameters) {
				std::tie(error, config) = step(bestGridsForCandidate, config, error, param);
			}

			++iteration;
			// abort gradient descent if error measurement did not improve by
			// a significant amount during the last six steps
			assert(!bestGridsForCandidate.empty());
			if ((initerror - bestGridsForCandidate.begin()->error) < 0.0001) {
				++numWithoutImprovement;
				if (numWithoutImprovement >= 5) break;
			} else {
				numWithoutImprovement = 0;
			}

			// instead of continuing the next iteration with the result of the
			// last step, use the best currently found config from now on.
			error  = bestGridsForCandidate.begin()->error;
			config = bestGridsForCandidate.begin()->config;
		}
		assert(!bestGridsForCandidate.empty());
		storeConfig(bestGridsForCandidate.begin()->error, bestGridsForCandidate.begin()->config);
		++candidate_it;
	}
}

std::pair<double, Util::gridconfig_t> GridFitter::GradientDescent::step(candidate_set &bestGrids, Util::gridconfig_t const& config, double error, const GridFitter::GradientDescent::StepParameter param)
{
	// when adjusting one of the position parameters, we can not adjust the step
	// size based on the difference between of the errors and the learning rate
	// (because the coordinates are discrete values).
	// instead, in each step we only change the position by either 1 or -1.
	// furthermore, if the error does not improve after a position change,
	// instead of just applying the reverse direction, we check if the error
	// acutally improves when going in the reverse direction.

	static const std::array<int, 2> directions {-1, 1};

	const double alpha     = _settings.alpha;
	const double eps_scale = _settings.eps_scale;
	const int eps_pos      = _settings.eps_pos;
	const double eps_angle = _settings.eps_angle;

	for (int direction : directions) {
		Util::gridconfig_t newConfig(config);
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
		double newError = evaluateCandidate(newGrid, _roi, _binarizedRoi, _edgeRoiX, _edgeRoiY, _settings);

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
			newError = evaluateCandidate(newGrid, _roi, _binarizedRoi, _edgeRoiX, _edgeRoiY, _settings);

			bestGrids.insert({newError, newConfig});
			return {newError, newConfig};
		} else if (newError < error) {
			bestGrids.insert({newError, newConfig});
			return {newError, newConfig};
		}
	}
	return {error, config};
}

void GridFitter::GradientDescent::storeConfig(const double error, const Util::gridconfig_t& config)
{
	_bestGrids.insert({error, config});

	if (_bestGrids.size() > _settings.gradient_num_results) {
		_bestGrids.erase(std::prev(_bestGrids.end()));
	}
}

double GridFitter::variance_twopass_calculator_t::getNormalizedVariance() const
{
	if (_pixelNum < 2) return 0.;

	const double mean = static_cast<double>(_sum) / static_cast<double>(_pixelNum);

	double sum = 0.;
	for (const uint8_t value : _values) {
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
