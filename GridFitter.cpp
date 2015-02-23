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
#include "source/tracking/algorithm/BeesBook/Common/Grid.h"
#include "source/utility/CvHelper.h"
#include "source/utility/util.h"

//#define DEBUG_GRIDFITTER

namespace pipeline {
GridFitter::GridFitter()
{}

void GridFitter::loadSettings(gridfitter_settings_t &&settings)
{
	_settings = std::move(settings);
}

std::vector<Tag> GridFitter::process(std::vector<Tag> &&taglist)
{
	for (Tag& tag : taglist) {
		tag.setValid(false);
		for (TagCandidate& candidate : tag.getCandidates()) {
			std::vector<Grid> grids = fitGrid(tag, candidate);
			if (!grids.empty()) { tag.setValid(true); };
			candidate.setGrids(std::move(grids));
		}
	}

	// remove tags without any fitted grids
	taglist.erase(std::remove_if(taglist.begin(), taglist.end(), [](Tag& tag) { return !tag.isValid(); }), taglist.end());

	return taglist;
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

void GridFitter::visualizeDebug(std::multiset<candidate_t> const& bestGrids, const size_t numResults, const cv::Size2i roiSize, const Tag& tag, cv::Mat const& binarizedROI) const
{
    cv::Mat binarizedROICpy;
    cv::cvtColor(binarizedROI, binarizedROICpy, CV_GRAY2BGR);

    const size_t to = std::min(numResults, bestGrids.size());
    size_t idx = 0;
    for (candidate_t const& candidate : bestGrids) {
        std::vector<cv::Mat> images;

        PipelineGrid grid(candidate.config);

        images.push_back(tag.getOrigSubImage());
        images.push_back(binarizedROICpy);
        images.push_back(grid.getProjectedImage(roiSize));

        cv::Mat blendedBin;
        cv::addWeighted(binarizedROICpy, 0.6, grid.getProjectedImage(roiSize), 0.4, 0.0, blendedBin);
        images.push_back(blendedBin);

        cv::Mat blended;
        cv::addWeighted(tag.getOrigSubImage(), 0.8, grid.getProjectedImage(roiSize), 0.2, 0.0, blended);
        images.push_back(blended);

        const auto canvas = CvHelper::makeCanvas(images, images[0].rows + 10, 1);

        std::string title("best fit (error: " + std::to_string(candidate.error) + ")");
        cv::namedWindow(title);
        cv::imshow(title, canvas);

        ++idx;
        if (idx == to) break;
    }

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

GridFitter::candidate_set GridFitter::getInitialCandidates(const cv::Mat &binarizedROI, const Ellipse& ellipse_orig, const cv::Mat &roi) const
{
	static const auto initial_rotations        = util::linspace<double>(0, 2 * CV_PI, 32);
	static const auto initial_position_offsets = util::linspace<int>(-4, 4, 9);

    // initial search for gradiant descent candidates in ellipse parameter space
    candidate_set gridCandidates;
    for (const int pos_x_offset : initial_position_offsets) {
        for (const int pos_y_offset : initial_position_offsets) {
            for (const double rotation : initial_rotations) {
                Ellipse ell(ellipse_orig);
                ell.setCen(ellipse_orig.getCen() + cv::Point2i(pos_x_offset, pos_y_offset));
                const std::array<PipelineGrid::gridconfig_t, 2> configCandidates =
                        Util::gridCandidatesFromEllipse(ell, rotation);
                for (PipelineGrid::gridconfig_t const& config : configCandidates) {
                    PipelineGrid candidate(config);
                    const double error = evaluateCandidate(candidate, roi, binarizedROI, _settings);
                    gridCandidates.insert({error, config});
                }
            }
        }
    }

	return gridCandidates;
}

std::vector<Grid> GridFitter::fitGrid(const Tag& tag, const TagCandidate &candidate) const
{
	const Ellipse& ellipse_orig = candidate.getEllipse();

    // region of interest of tag candidate
    const cv::Size2i roiSize = tag.getBox().size();
    cv::Mat roi;
    // TODO: shouldn't be BGR in the first place
    cv::cvtColor(tag.getOrigSubImage(), roi, CV_BGR2GRAY);

    cv::Mat binarizedROI(roiSize, CV_8UC1);
    cv::adaptiveThreshold(roi, binarizedROI, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C,
                          cv::THRESH_BINARY, _settings.adaptiveBlockSize,
                          _settings.adaptiveC);

#ifdef DEBUG_GRIDFITTER_HIST
    calculateHistogram(roi, ellipse_orig);
#endif

	// initial search parameter space
	candidate_set gridCandidates = getInitialCandidates(binarizedROI, ellipse_orig, roi);

    std::cout << "min initial candidate error: " << gridCandidates.begin()->error << std::endl;

    static const size_t numResults = 1;

	GradientDescent optimizer(gridCandidates, roi, binarizedROI, _settings);
	optimizer.optimize();

	const candidate_set& bestGrids = optimizer.getBestGrids();


    std::cout << "min final candidate error: " << bestGrids.begin()->error << std::endl;

#ifdef DEBUG_GRIDFITTER
    visualizeDebug(bestGrids, numResults, roiSize, tag, binarizedROI);
#endif

    std::vector<Grid> results;
    {
        const size_t to = std::min(numResults, bestGrids.size());
        size_t idx = 0;
        for (candidate_t const& candidate : bestGrids) {
            PipelineGrid::gridconfig_t const& config = candidate.config;
            results.emplace_back(config.center, config.radius, config.angle_z, config.angle_y, config.angle_x);

            ++idx;
            if (idx == to) break;
        }
    }

    return results;
}

double GridFitter::evaluateCandidate(PipelineGrid& grid, cv::Mat const& roi, cv::Mat const& binarizedROI, const gridfitter_settings_t &settings)
{
	// TODO: fix PipelineGrid bugs
	try {
    double error = 0;

    // bounding box and center of grid candidate
    const cv::Rect boundingBox   = grid.getBoundingBox();
    const cv::Point2i gridCenter = grid.getCenter();

    // temporarily shift grid position so that top left corner is at
    // position (0, 0) of boundingBox
    // TODO: maybe it's better to adjust get*Coordinates functions so that
    // they take the roi location into account
    grid.setCenter(gridCenter - boundingBox.tl());

    // define region of interest based on bounding box of grid candidate
    // from now on, we do all calculations on this roi to speed up all
    // computations that have to iterate over the (sub)image
    cv::Mat subROI;
    subROI = roi(boundingBox);
    const cv::Size subROIsize = subROI.size();

    // region of interest of binarized image
    cv::Mat subBinarized;
    subBinarized = binarizedROI(boundingBox);

    // get coordinates of current grid configuration
    const cv::Mat& outerRingCoordinates  = grid.getOuterRingCoordinates(subROIsize);
    const cv::Mat& innerWhiteCoordinates = grid.getInnerWhiteRingCoordinates(subROIsize);
    const cv::Mat& innerBlackCoordinates = grid.getInnerBlackRingCoordinates(subROIsize);
    const std::vector<cv::Mat>& cellCoordinates = grid.getGridCellCoordinates(subROIsize);

    // compare outer circle
    size_t outerCircleError = 0;
    for (size_t idx = 0; idx < outerRingCoordinates.total(); ++idx) {
        const cv::Point2i coords = outerRingCoordinates.at<cv::Point2i>(idx);
        const uint8_t value      = subBinarized.at<uint8_t>(coords);
        // no branching => better performance
        outerCircleError += 255 - value;
    }
    error += (settings.alpha_outer * outerCircleError) / outerRingCoordinates.total();

    // compare inner white semicircle
    size_t innerWhiteError = 0;
    for (size_t idx = 0; idx < innerWhiteCoordinates.total(); ++idx) {
        const cv::Point2i coords = innerWhiteCoordinates.at<cv::Point2i>(idx);
        const uint8_t value      = subBinarized.at<uint8_t>(coords);
        innerWhiteError += 255 - value;
    }
    error += (settings.alpha_inner * innerWhiteError) / innerWhiteCoordinates.total();

    // compare inner black semicircle
    size_t innerBlackError = 0;
    for (size_t idx = 0; idx < innerBlackCoordinates.total(); ++idx) {
        const cv::Point2i coords = innerBlackCoordinates.at<cv::Point2i>(idx);
        const uint8_t value      = subBinarized.at<uint8_t>(coords);
        innerBlackError += value;
    }
    error += (settings.alpha_inner * innerBlackError) / innerBlackCoordinates.total();

    error /= 255.;

    // add variance of each grid cells to error
    for (cv::Mat const& cell : cellCoordinates) {
        // two-pass variance calculation. first pass calculates mean of
        // points in cell, second pass calculates variance based on mean.
        // TODO: evaluate whether a one-pass method significantly improves
        // performance and whether a more numerically stable algorithm is
        // required.
        // see: http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
        size_t sum = 0;
        for (size_t idx = 0; idx < cell.total(); ++idx) {
            const cv::Point2i coords = cell.at<cv::Point2i>(idx);
            const uint8_t value      = subBinarized.at<uint8_t>(coords);
            sum += value;
        }
        const double mean = static_cast<double>(sum) / static_cast<double>(cell.total());

        double sum2 = 0;
        for (size_t idx = 0; idx < cell.total(); ++idx) {
            const cv::Point2i coords = cell.at<cv::Point2i>(idx);
            const double value       = static_cast<double>(subBinarized.at<uint8_t>(coords));
            sum2 += (value - mean) * (value - mean);
        }
        const double variance = (sum2 / (cell.total() - 1));

        error += settings.alpha_variance * (variance / 255.);
    }

    // reset grid to original position
    grid.setCenter(gridCenter);

	return error;
	}
	catch (std::exception const& ex) {
		return std::numeric_limits<double>::max();
	}
}

GridFitter::GradientDescent::GradientDescent(const GridFitter::candidate_set &initialCandidates, const cv::Mat &roi, const cv::Mat &binarizedRoi, const gridfitter_settings_t &settings)
    : _initialCandidates(initialCandidates)
    , _settings(settings)
    , _roi(roi)
    , _binarizedRoi(binarizedRoi)
{}

void GridFitter::GradientDescent::optimize()
{
    const size_t num = std::min(_settings.numInitial, _initialCandidates.size());
    candidate_set::iterator candidate_it = _initialCandidates.begin();
    for (size_t idx = 0; idx < num; ++idx) {
        candidate_t candidate = *candidate_it;
        const PipelineGrid::gridconfig_t& initial_config = candidate.config;

        size_t iteration = 0;
        PipelineGrid grid(initial_config);
        PipelineGrid::gridconfig_t config = initial_config;
        grid  = PipelineGrid(initial_config);
        double error = evaluateCandidate(grid, _roi, _binarizedRoi, _settings);
        while ((error > _settings.errorThreshold) && (iteration < _settings.maxIterations)) {
            double const initerror = error;

			std::tie(error, config) = step(config, error, SCALE);
			std::tie(error, config) = step(config, error, POSX);
			std::tie(error, config) = step(config, error, POSY);
			std::tie(error, config) = step(config, error, ANGLE_X);
			std::tie(error, config) = step(config, error, ANGLE_Y);
			std::tie(error, config) = step(config, error, ANGLE_Z);

            ++iteration;
			if (std::abs(initerror - error) < 0.01) break;
        }
        ++candidate_it;
	}
}

std::pair<double, PipelineGrid::gridconfig_t> GridFitter::GradientDescent::step(PipelineGrid::gridconfig_t const& config, double error, const GridFitter::GradientDescent::StepParameter param)
{
	static const std::array<int, 2> directions {-1, 1};

	const double alpha     = _settings.alpha;
	const double eps_scale = _settings.eps_scale;
	const int eps_pos      = _settings.eps_pos;
	const double eps_angle = _settings.eps_angle;

	for (int direction : directions) {
		PipelineGrid::gridconfig_t newConfig(config);
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

		PipelineGrid newGrid(newConfig);
		double newError = evaluateCandidate(newGrid, _roi, _binarizedRoi, _settings);

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
			newError = evaluateCandidate(newGrid, _roi, _binarizedRoi, _settings);

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

	if (_bestGrids.size() > _settings.numResults) {
		_bestGrids.erase(std::prev(_bestGrids.end()));
	}
}
}
