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

#define DEBUG_GRIDFITTER

namespace pipeline {
GridFitter::GridFitter()
{}

void GridFitter::loadSettings(gridfitter_settings_t &&settings)
{
	_settings = std::move(settings);
}

std::vector<Tag> GridFitter::process(std::vector<Tag> &&taglist)
{
#ifdef DEBUG_GRIDFITTER
	for (Tag& tag : taglist) {
		tag.setValid(false);
		for (TagCandidate& candidate : tag.getCandidates()) {
				std::vector<PipelineGrid> grids = fitGrid(tag, candidate);
				if (!grids.empty()) { tag.setValid(true); };
				candidate.setGrids(std::move(grids));
		}
	}
#else
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

void GridFitter::visualizeDebug(std::multiset<candidate_t> const& bestGrids, const cv::Size2i roiSize, const Tag& tag, cv::Mat const& binarizedROI, std::string winName) const
{
    cv::Mat binarizedROICpy;
    cv::cvtColor(binarizedROI, binarizedROICpy, CV_GRAY2BGR);

    cv::Mat cannyImg;
    cv::cvtColor(tag.getCannySubImage(), cannyImg, CV_GRAY2BGR);

    const size_t to = std::min(_settings.numResults, bestGrids.size());
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

GridFitter::candidate_set GridFitter::getInitialCandidates(const cv::Mat &binarizedROI, const Ellipse& ellipse_orig, const cv::Mat &roi) const
{
	static const auto initial_rotations        = util::linspace<double>(0, 2 * CV_PI, 32);
	static const auto initial_position_offsets = util::linspace<int>(-4, 4, 9);

    // initial search for gradiant descent candidates in ellipse parameter space
    candidate_set gridCandidates;
    for (const double rotation : initial_rotations) {
        candidate_set candidatesForRotation;
        const std::array<PipelineGrid::gridconfig_t, 2> configCandidates =
                Util::gridCandidatesFromEllipse(ellipse_orig, rotation);
        for (PipelineGrid::gridconfig_t const& config : configCandidates) {
            PipelineGrid grid(config);
            for (const int pos_x_offset : initial_position_offsets) {
                for (const int pos_y_offset : initial_position_offsets) {
                    grid.setCenter({config.center.x + pos_x_offset, config.center.y + pos_y_offset});
                    const double error = evaluateCandidate(grid, roi, binarizedROI, _settings);
                    candidatesForRotation.insert({error, grid.getConfig()});
                }
            }
        }
        gridCandidates.insert(*candidatesForRotation.begin());
    }

	return gridCandidates;
}

std::vector<PipelineGrid> GridFitter::fitGrid(const Tag& tag, const TagCandidate &candidate) const
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

	// initial search parameter space
	candidate_set gridCandidates = getInitialCandidates(binarizedROI, ellipse_orig, roi);

	GradientDescent optimizer(gridCandidates, roi, binarizedROI, _settings);
	optimizer.optimize();

	const candidate_set& bestGrids = optimizer.getBestGrids();

#ifdef DEBUG_GRIDFITTER
    std::cout << "min initial candidate error: " << gridCandidates.begin()->error << std::endl;
    std::cout << "min final candidate error: " << bestGrids.begin()->error << std::endl;

    visualizeDebug(gridCandidates, roiSize, tag, binarizedROI, "initial fit");
    visualizeDebug(bestGrids, roiSize, tag, binarizedROI, "best fit");
#endif

    std::vector<PipelineGrid> results;
    {
        const size_t to = std::min(_settings.numResults, bestGrids.size());
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

double GridFitter::evaluateCandidate(PipelineGrid& grid, cv::Mat const& roi, cv::Mat const& binarizedROI, const gridfitter_settings_t &settings)
{
    double error = 0;

    // bounding box of grid candidate
    const cv::Rect boundingBox = grid.getBoundingBox();

    if (!boundingBox.area()) return std::numeric_limits<double>::max();
    if (boundingBox.x < 0 || boundingBox.y < 0) return std::numeric_limits<double>::max();
    if (boundingBox.x + boundingBox.width >= roi.rows ||
        boundingBox.y + boundingBox.height >= roi.cols) return std::numeric_limits<double>::max();

    // define region of interest based on bounding box of grid candidate
    // from now on, we do all calculations on this roi to speed up all
    // computations that have to iterate over the (sub)image
    cv::Mat subROI;
    subROI = roi(boundingBox);

    // region of interest of binarized image
    cv::Mat subBinarized;
    subBinarized = binarizedROI(boundingBox);

    // get coordinates of current grid configuration
    const PipelineGrid::coordinates_t& outerRingCoordinates  = grid.getOuterRingCoordinates();
    const PipelineGrid::coordinates_t& innerWhiteCoordinates = grid.getInnerWhiteRingCoordinates();
    const PipelineGrid::coordinates_t& innerBlackCoordinates = grid.getInnerBlackRingCoordinates();

    // compare outer circle
    size_t outerCircleError = 0;
    for (const cv::Point2i coords : outerRingCoordinates) {
        const uint8_t value      = subBinarized.at<uint8_t>(coords - boundingBox.tl());
        // no branching => better performance
        outerCircleError += 255 - value;
    }
    if (outerRingCoordinates.size())
        error += (settings.alpha_outer * outerCircleError) / outerRingCoordinates.size();

    // compare inner white semicircle
    size_t innerWhiteError = 0;
    for (const cv::Point2i coords : innerWhiteCoordinates) {
        const uint8_t value      = subBinarized.at<uint8_t>(coords - boundingBox.tl());
        innerWhiteError += 255 - value;
    }
    if (innerWhiteCoordinates.size())
        error += (settings.alpha_inner * innerWhiteError) / innerWhiteCoordinates.size();

    // compare inner black semicircle
    size_t innerBlackError = 0;
    for (const cv::Point2i coords : innerBlackCoordinates) {
        const uint8_t value      = subBinarized.at<uint8_t>(coords - boundingBox.tl());
        innerBlackError += value;
    }
    if (innerBlackCoordinates.size())
        error += (settings.alpha_inner * innerBlackError) / innerBlackCoordinates.size();

    error /= 255.;

    // add variance of each grid cells to error
    for (size_t cellIdx = 0; cellIdx < Grid::NUM_MIDDLE_CELLS; ++cellIdx) {
        PipelineGrid::coordinates_t const& cellCoordinates = grid.getGridCellCoordinates(cellIdx);
        if (cellCoordinates.size() > 1) {
            // two-pass variance calculation. first pass calculates mean of
            // points in cell, second pass calculates variance based on mean.
            // TODO: evaluate whether a one-pass method significantly improves
            // performance and whether a more numerically stable algorithm is
            // required.
            // see: http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
            size_t sum = 0;
            for (const cv::Point2i coords : cellCoordinates) {
                const uint8_t value      = subBinarized.at<uint8_t>(coords);
                sum += value;
            }
            const double mean = static_cast<double>(sum) / static_cast<double>(cellCoordinates.size());

            double sum2 = 0;
            for (const cv::Point2i coords : cellCoordinates) {
                const double value       = static_cast<double>(subBinarized.at<uint8_t>(coords));
                sum2 += (value - mean) * (value - mean);
            }
            const double variance = (sum2 / (cellCoordinates.size() - 1));

            error += settings.alpha_variance * (variance / 255.);
        }
    }

	return error;
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
        storeConfig(error, config);
        while ((error > _settings.errorThreshold) && (iteration < _settings.maxIterations)) {
            double const initerror = error;

			std::tie(error, config) = step(config, error, SCALE);
			if (!_bestGrids.empty()) {
				error  = _bestGrids.begin()->error;
				config = _bestGrids.begin()->config;
			}
			std::tie(error, config) = step(config, error, POSX);
			if (!_bestGrids.empty()) {
				error  = _bestGrids.begin()->error;
				config = _bestGrids.begin()->config;
			}
			std::tie(error, config) = step(config, error, POSY);
			if (!_bestGrids.empty()) {
				error  = _bestGrids.begin()->error;
				config = _bestGrids.begin()->config;
			}
			std::tie(error, config) = step(config, error, ANGLE_X);
			if (!_bestGrids.empty()) {
				error  = _bestGrids.begin()->error;
				config = _bestGrids.begin()->config;
			}
			std::tie(error, config) = step(config, error, ANGLE_Y);
			if (!_bestGrids.empty()) {
				error  = _bestGrids.begin()->error;
				config = _bestGrids.begin()->config;
			}
			std::tie(error, config) = step(config, error, ANGLE_Z);

            ++iteration;
			if (std::abs(initerror - error) < 0.01) break;

			if (!_bestGrids.empty()) {
				error  = _bestGrids.begin()->error;
				config = _bestGrids.begin()->config;
			}
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

		// TODO: don't construct new Grid in each step
		// TODO: don't recalculate coordinates for position change
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
