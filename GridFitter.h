#pragma once

#include <set>
#include <vector>

#include "datastructure/PipelineGrid.h"
#include "datastructure/settings.h"

namespace pipeline {

class Ellipse;
class Tag;
class TagCandidate;

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

	settings::gridfitter_settings_t _settings;

	// cached settings
	typedef struct {
		double err_func_alpha_inner;
		double err_func_alpha_outer;
		double err_func_alpha_variance;
		double err_func_alpha_inner_edge;
		double err_func_alpha_outer_edge;
		size_t gradient_num_initial;
		size_t gradient_num_results;
		double gradient_error_threshold;
		double gradient_max_iterations;
		double alpha;
		double eps_scale;
		double eps_angle;
		int eps_pos;
	} settings_cache_t;

	settings_cache_t _settings_cache;
	void cacheSettings();

	class GradientDescent {
	public:
		GradientDescent(const candidate_set& initialCandidates,
		                const cv::Mat& roi,
		                const cv::Mat& binarizedRoi,
		                const cv::Mat& edgeRoiX,
		                const cv::Mat& edgeRoiY,
		                const settings_cache_t& settings);

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

		const settings_cache_t& _settings;

		// region of interest grayscale image
		const cv::Mat& _roi;
		// binarized region of interest
		const cv::Mat& _binarizedRoi;
		// edge image of region of interest
		const cv::Mat& _edgeRoiX;
		const cv::Mat& _edgeRoiY;

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

		std::random_device _random_device;
		std::mt19937 _random_engine;

		/**
		 * @brief step a single gradient descent step
		 * @param config previous grid config
		 * @param error errors associated with previous config
		 * @param param which parameter to change in current step
		 */
		std::pair<double, PipelineGrid::gridconfig_t> step(candidate_set& bestGrids, const PipelineGrid::gridconfig_t &config, double error, const StepParameter param);

		/**
		 * @brief storeConfig store a config in _bestGrids
		 * stores to given config and associated error in _bestGrids and removes
		 * the worst config iff _bestGrids.size() > settings.maxResults
		 */
		void storeConfig(const double error, const PipelineGrid::gridconfig_t &config);
	};

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
	candidate_set getInitialCandidates(cv::Mat const& binarizedROI, const cv::Mat& sobelXRoi, const cv::Mat &sobelYRoi, const Ellipse& ellipse_orig, cv::Mat const& roi);

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
	static double evaluateCandidate (PipelineGrid& grid, const cv::Mat& roi, const cv::Mat& binarizedROI, const cv::Mat& sobelXRoi, const cv::Mat &sobelYRoi, settings_cache_t const& settings);

	/**
	 * @brief calculateHistogram can be used for debug purposes
	 */
	cv::Mat calculateHistogram(const cv::Mat& roi, const Ellipse& ellipse_orig) const;

	/**
	 * @brief visualizeDebug visualize best fit and intermediate results
	 */
	void visualizeDebug(const std::multiset<candidate_t>& bestGrids, const cv::Mat &roi, const cv::Size2i roiSize, const cv::Mat& edgeRoiX, const cv::Mat& edgeRoiY, const Tag& tag, const cv::Mat& binarizedROI, std::string winName, const size_t numBest);

	template <typename ErrorCounterFun>
	class error_counter_t {
	public:
		explicit error_counter_t(const cv::Mat& roi)
		    : _roi(roi), _errorSum(0), _pixelNum(0)
		{}

		error_counter_t(const error_counter_t&) = delete;
		error_counter_t& operator=(const error_counter_t&) = delete;

		error_counter_t(error_counter_t&&) = default;
		error_counter_t& operator=(error_counter_t&&) = default;

		inline void operator()(cv::Point coords)
		{
			const uint8_t value = _roi.get().template at<uint8_t>(coords);
			_errorSum += errorFun(value);
			++_pixelNum;
		}

		inline double getNormalizedError() const
		{
			return static_cast<double>(_errorSum) / (static_cast<double>(_pixelNum) * 255.);
		}

	private:
		ErrorCounterFun errorFun;
		std::reference_wrapper<const cv::Mat> _roi;
		size_t _errorSum;
		size_t _pixelNum;
	};

	class sobel_error_counter_t {
	public:
		explicit sobel_error_counter_t(const cv::Mat& sobelX, const cv::Mat& sobelY)
		    : _sobelX(sobelX), _sobelY(sobelY)
		    , _expectedX(0), _expectedY(0)
		    , _errorSum(0), _pixelNum(0)
		{}

		sobel_error_counter_t(const sobel_error_counter_t&) = delete;
		sobel_error_counter_t& operator=(const sobel_error_counter_t&) = delete;

		sobel_error_counter_t(sobel_error_counter_t&&) = default;
		sobel_error_counter_t& operator=(sobel_error_counter_t&&) = default;

		inline void operator()(cv::Point coords)
		{
			const uint8_t dx = _sobelX.get().template at<uint8_t>(coords);
			const uint8_t dy = _sobelY.get().template at<uint8_t>(coords);

			_errorSum += std::abs(static_cast<int16_t>(dx) - static_cast<int16_t>(_expectedX));
			_errorSum += std::abs(static_cast<int16_t>(dy) - static_cast<int16_t>(_expectedY));

			_pixelNum += 2;
		}

		inline void setExpectedSobelGradient(const uint8_t dx, const uint8_t dy)
		{
			_expectedX = dx;
			_expectedY = dy;
		}

		inline double getNormalizedError() const
		{
			return static_cast<double>(_errorSum) / (static_cast<double>(_pixelNum) * 255.);
		}

	private:
		std::reference_wrapper<const cv::Mat> _sobelX;
		std::reference_wrapper<const cv::Mat> _sobelY;
		uint8_t _expectedX;
		uint8_t _expectedY;
		size_t _errorSum;
		size_t _pixelNum;
	};

	struct expected_white_error_fun_t {
		inline uint8_t operator()(const uint8_t value) { return 255 - value; }
	};

	struct expected_black_error_fun_t {
		inline uint8_t operator()(const uint8_t value) { return value; }
	};

	// http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Two-pass_algorithm
	class variance_twopass_calculator_t {
	public:
		explicit variance_twopass_calculator_t(const cv::Mat& roi)
		    : _roi(roi), _pixelNum(0), _sum(0)
		{}

		variance_twopass_calculator_t(const variance_twopass_calculator_t&) = delete;
		variance_twopass_calculator_t& operator=(const variance_twopass_calculator_t&) = delete;

		variance_twopass_calculator_t(variance_twopass_calculator_t&&) = default;
		variance_twopass_calculator_t& operator=(variance_twopass_calculator_t&&) = default;

		inline void operator()(cv::Point coords)
		{
			const uint8_t value = _roi.get().template at<uint8_t>(coords);
			++_pixelNum;
			_sum += value;
			_values.push_back(value);
		}

		double getNormalizedVariance() const;

	private:
		std::reference_wrapper<const cv::Mat> _roi;
		std::vector<uint8_t> _values;
		size_t _pixelNum;
		size_t _sum;
	};

	// http://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
	class variance_online_calculator_t {
	public:
		explicit variance_online_calculator_t(const cv::Mat& roi)
		    : _roi(roi), _pixelNum(0), _mean(0.), _m2(0.)
		{}

		variance_online_calculator_t& operator=(variance_online_calculator_t&&) = default;
		variance_online_calculator_t(variance_online_calculator_t&&) = default;
		variance_online_calculator_t& operator=(const variance_online_calculator_t&) = delete;
		variance_online_calculator_t(const variance_online_calculator_t&) = delete;

		inline void operator()(cv::Point coords)
		{
			const double value = _roi.get().template at<uint8_t>(coords);
			++_pixelNum;
			const double delta = value - _mean;
			_mean += (delta / _pixelNum);
			_m2   += delta * (value - _mean);
		}

		inline double getNormalizedVariance() const;

	private:
		std::reference_wrapper<const cv::Mat> _roi;
		std::vector<uint8_t> _values;
		size_t _pixelNum;
		double _mean;
		double _m2;
	};
};
}
