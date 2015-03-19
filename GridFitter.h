#pragma once

#include <set>
#include <vector>

#include "source/tracking/algorithm/BeesBook/Common/Grid.h"

#include "datastructure/PipelineGrid.h"
#include "datastructure/settings.h"

namespace pipeline {

class Ellipse;
class Tag;
class TagCandidate;

typedef struct {
	// error function weights
	// TODO: individual error values should be scaled to [0, 1] to make the
	// error weights more intuitive
	// inner semicircles
	double alpha_inner    = 500.0;
	// outer circle
	double alpha_outer    = 100.0;
	// variance of grid cells
	double alpha_variance = 0.8;
	// outer border edge
	double alpha_outer_edge = 5.0;
	// line between inner semicircles
	// not used at the moment
	double alpha_inner_edge = 10.0;

	// adaptive thresholding parameters
    // size of neighbourhood area
    int adaptiveBlockSize = 23;
    // constant which is substracted from mean of neighborhood area
    double adaptiveC      = 3;

	// gradient descent parameters
	// number of initial brute force results that are fed into the second
	// (gradient descent) stage
	size_t numInitial = 1;
	// max number of results to return
	size_t numResults = 1;

	// stop gradient descent when error < errorThreshold
	double errorThreshold = 80.;
	// stop gradient after maxIterations iterations
	size_t maxIterations  = 100;

	// step size for angles, position and scale
	double eps_angle = 0.02;
	int eps_pos      = 1;
	double eps_scale = 0.1;

	// learning rate
	double alpha     = 0.01;
} gridfitter_settings_t;

class GridFitter {
public:
	GridFitter();

	void loadSettings(settings::gridfitter_settings_t&& settings);

	/**
	 * @brief process
	 * @param taglist vector of detected rois with ellipse candidates
	 * @return same as input, but with grid candidates added to each ellipse
	 *         candidate for each tag
	 */
	std::vector<Tag> process(std::vector<Tag>&& taglist);

private:
	// represents a grid candidate, contains a grid config (angles, position,
	// and scale) and the corresponding error
	struct candidate_t {
		double error;
		PipelineGrid::gridconfig_t config;

		// comparison operators for std::set/multiset support
		bool operator<(const candidate_t& other) const {
			return error < other.error;
		}

		bool operator==(const candidate_t& other) const {
			return error == other.error;
		}
	};

	// stores multiple grid candidates, automatically sorted by error. It's
	// possible for multiple distinct grid candidates to exist with the same
	// associated error, therefore we have to use a multiset
	typedef std::multiset<candidate_t> candidate_set;

	class GradientDescent {
	public:
		GradientDescent(const candidate_set& initialCandidates,
		                const cv::Mat& roi,
		                const cv::Mat& binarizedRoi,
						const cv::Mat& edgeRoi,
		                settings::gridfitter_settings_t& settings);

		/**
		 * @brief optimize tries to improve the fit of the given initial
		 * grid candiates using a simple gradient descent implementation.
		 */
		void optimize();

		/**
		 * @return a set consisting of the settings.maxResults best candiates
		 * after the gradient descent. returns an empty set if called before
		 * executing optimize(),
		 */
		candidate_set const& getBestGrids() const { return _bestGrids; }

	private:
		const candidate_set& _initialCandidates;
		settings::gridfitter_settings_t& _settings;


		// region of interest grayscale image

		const cv::Mat& _roi;
		// binarized region of interest
		const cv::Mat& _binarizedRoi;
		// edge image of region of interest
		const cv::Mat& _edgeRoi;

		// contains the settings.maxResults (or less) best grid candiates.
		// candiates should only be inserted using the storeConfig() method.
		candidate_set _bestGrids;

		enum StepParameter {
			POSX = 0,
			POSY,
			ANGLE_X,
			ANGLE_Y,
			ANGLE_Z,
			SCALE
		};

		/**
		 * @brief step a single gradient descent step
		 * @param config previous grid config
		 * @param error errors associated with previous config
		 * @param param which parameter to change in current step
		 */
		std::pair<double, PipelineGrid::gridconfig_t> step(const PipelineGrid::gridconfig_t &config, double error, const StepParameter param);

		/**
		 * @brief storeConfig store a config in _bestGrids
		 * stores to given config and associated error in _bestGrids and removes
		 * the worst config iff _bestGrids.size() > settings.maxResults
		 */
		void storeConfig(const double error, const PipelineGrid::gridconfig_t &config);
	};

	settings::gridfitter_settings_t _settings;




	/**
	 * @brief fitGrid try to find the best match for the current candidate
	 * @param tag contains the region of interest of the candiate
	 * @param candidate contains the ellipse candidate
	 * @return a vector of grid candidates for the ellipse candidate
	 *
	 * general idea: two pass algorithm
	 *
	 * 1) detect grid candidates with a brute force approach over a small
	 * parameter space (mostly to get an estimate for the rotation around the
	 * own axis which can not be calculated from the ellipse candidate)
	 *
	 * 2) for the settings.numInitial candidates, try to improve the fit using
	 * a gradient descent. return the settings.numResults best grid candiates.
	 */
		 std::vector<PipelineGrid> fitGrid(const Tag &tag, TagCandidate const& candidate);

	/**
	 * @brief getInitialCandidates brute force search for grid candidates
	 * @return set containing the settings.numInitial best candidates (but
	 * not more than one per rotation)
	 */
candidate_set getInitialCandidates(cv::Mat const& binarizedROI, const cv::Mat& edgeROI, const Ellipse& ellipse_orig, cv::Mat const& roi);

	/**
	 * @brief evaluateCandidate calculate error for given grid candidate
	 * @return error measurement
	 *
	 * error function evaluates:
	 * 1) inner semicircle/outer grid regions: each pixel of the binarized
	 * region of interest is compared to the expected binary value according
	 * to the grid model.
	 * 2) grid cells: for each grid cell, we only know that it must be either
	 * completely white or black. we use the variance to approximate the error
	 * for each grid cell.
	 * 3) edges: given a sufficiently good edge extraction, we can expect the
	 * edges between the grid and be be (the outer edge of the outer ring) and
	 * the line between the two inner semicircles to always exist.
	 */
static double evaluateCandidate (PipelineGrid& grid, const cv::Mat& roi, const cv::Mat& binarizedROI, const cv::Mat& edgeROI, settings::gridfitter_settings_t& settings);

	/**
	 * @brief calculateHistogram can be used for debug purposes
	 */
    cv::Mat calculateHistogram(const cv::Mat& roi, const Ellipse& ellipse_orig) const;

	/**
	 * @brief visualizeDebug visualize best fit and intermediate results
	 */
    void visualizeDebug(const std::multiset<candidate_t>& bestGrids, const cv::Size2i roiSize, const Tag& tag, const cv::Mat& binarizedROI, std::string winName);
};
}
