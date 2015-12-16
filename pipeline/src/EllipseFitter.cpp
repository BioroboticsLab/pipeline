/*
 * EllipseFitter.cpp
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#include <cmath>

#include "../EllipseFitter.h"

#include "../datastructure/Tag.h"
#include "../datastructure/Ellipse.h"
#include "../util/ThreadPool.h"

namespace {
inline double pointDistance(double px, double py, double qx, double qy)
{
	return (sqrt((qy - py) * (qy - py) + (qx - px) * (qx - px)));
}

inline double pointDistanceNoSqrt(double px, double py, double qx, double qy)
{
	return (((qy - py) * (qy - py) + (qx - px) * (qx - px)));
}

struct compareVote {
	inline bool operator()(pipeline::Ellipse const& a, pipeline::Ellipse const& b)
	{
		return a.getVote() > b.getVote();
	}
};
}

namespace pipeline {

//#define DEBUG_MODE_ELLIPSEFITTER_XIE

void EllipseFitter::loadSettings(settings::ellipsefitter_settings_t &&settings)
{
	_settings = std::move(settings);
}

void EllipseFitter::loadSettings(const settings::ellipsefitter_settings_t &settings)
{
	_settings = settings;
}

/**
 * @param tag for which ellipses should be detected
 */
void EllipseFitter::detectEllipse(Tag &tag) {
	const double ellipsefitter_max_minor = _settings.get_max_minor_axis();
	const double ellipsefitter_max_major = _settings.get_max_major_axis();
	const double ellipsefitter_min_major = _settings.get_min_major_axis();
	const double ellipsefitter_min_minor = _settings.get_min_minor_axis();

    const double ellipse_regularisation = _settings.get_ellipse_regularisation();

	const int threshold_edge_pixels = _settings.get_threshold_edge_pixels();
	const int threshold_best_vote   = _settings.get_threshold_best_vote();

	const cv::Mat& subImage  = tag.getOrigSubImage();
    const cv::Mat cannyImage = computeCannyEdgeMap(subImage);

	tag.setCannySubImage(cannyImage);

	//edge_pixel array, all edge pixels are stored in this array
	std::vector<cv::Point2i> ep;
	ep.reserve(cv::countNonZero(cannyImage) + 1);

	std::vector<Ellipse> candidates;

	// (1) all white (being edge) pixels are written into the ep array
	for (auto mit = cannyImage.begin<unsigned char>(), end = cannyImage.end<unsigned char>();
	     mit != end; ++mit) {
		if (*mit.ptr == 255) {
			ep.push_back(mit.pos());
		}
	}

	// (2) initiate the accumulator array
    // accuracyMultiplier sets the discretization of the accumulator array. A multiplier of 4
    // leads to a discretization step of 0.25.
    static const double accuracyMultiplier = 4.;
    const double accumulatorElements = static_cast<double>(_settings.get_max_minor_axis() / 2.) *
            accuracyMultiplier + 1;
    std::vector<size_t> accu(static_cast<size_t>(std::ceil(accumulatorElements)));

	// (3) loop through each edge pixel in ep
	const size_t epsize = ep.size();
	for (size_t lc1 = 0; lc1 < epsize; ++lc1) {
		const int p1x     = ep[lc1].x;
		const int p1y     = ep[lc1].y;
		const double p1xf = static_cast<double>(ep[lc1].x);
		const double p1yf = static_cast<double>(ep[lc1].y);
		// (4) loop through each other edge pixel in ep and propose a major axis between p1 and p2
		for (size_t lc2 = 0; lc2 < epsize; ++lc2) {
			const int p2x     = ep[lc2].x;
			const int p2y     = ep[lc2].y;
			const double p2xf = static_cast<double>(ep[lc2].x);
			const double p2yf = static_cast<double>(ep[lc2].y);
			if ((lc2 > lc1) && ((p1x != p2x) || (p1y != p2y))) {
				// the proposed ellipse' length of the major axis lies between this->ELLIPSEFITTER_MIN_MAJOR and this->ELLIPSEFITTER_MAX_MAJOR
				const double dist = pointDistance(p1xf, p1yf, p2xf, p2yf);
				if (dist > ellipsefitter_min_major && dist < ellipsefitter_max_major) {
					// (5) calculate the ellipse' center, half length of the major axis (a) and orientation (alpha) based on p1 and p2
					const double centerx = (p1xf + p2xf) / 2;
					const double centery = (p1yf + p2yf) / 2;
                    const double a       = dist / 2.;
					const double alpha   = atan2((p2yf - p1yf), (p2xf - p1xf));
					// (6) loop through each third edge pixel eventually lying on the ellipse
					for (size_t lc3 = 0; lc3 < epsize; ++lc3) {
						const int px     = ep[lc3].x;
						const int py     = ep[lc3].y;
						const double pxf = static_cast<double>(ep[lc3].x);
						const double pyf = static_cast<double>(ep[lc3].y);
						if (((px != p2x) || (py != p2y)) && ((px != p1x) || (py != p1y))) {
							const double d = pointDistance(pxf, pyf, centerx, centery);
							if (d <= a && d > ellipsefitter_min_minor / 2.0) {
								// (7) estimate the half length of the minor axis (b)
								const double f      = pointDistanceNoSqrt(pxf, pyf, p2xf, p2yf);
								const double costau = (a * a + d * d - f) / (2 * a * d);
//								const double tau    = acos(costau);
//								const double sintau = sin(tau);
								const double sintau = sqrt(1 - costau * costau);
								const double b      = (a * d * sintau) / sqrt(((a * a) - (d * d * costau * costau)));
								const double b2     = 2.0 * b;
                                const size_t idx    = cvRound(b * accuracyMultiplier);
                                // (8) increment the accumulator for the minor axis' half length (b) just estimated
                                if (b2 <= ellipsefitter_max_minor && b2 >= ellipsefitter_min_minor &&
                                        (idx / accuracyMultiplier) <= a) {
                                    accu[idx] += 1;
                                }
							}
						}
					}

					// (10) find the maximum within the accumulator, is it above the threshold?
					const auto max_it  = std::max_element(accu.begin(), accu.end());
					const auto max_ind = std::distance(accu.begin(), max_it);
                    const double minor = static_cast<double>(max_ind) / accuracyMultiplier;
					int vote_minor     = *max_it;

                    if (vote_minor >= (threshold_edge_pixels / accuracyMultiplier)) {
						// (11) save ellipse parameters
                        const double major = a;
                        const cv::Point2i cen(cvRound(centerx), cvRound(centery));
                        const cv::Size2d axis(major, minor);
						const double angle = (alpha * 180) / CV_PI;

                        // more "circular" ellipses are weighted more than very thin ellipses
                        vote_minor = static_cast<int>(vote_minor * (ellipse_regularisation * minor / major));

						if (candidates.size() == 0) {
                            candidates.emplace_back(vote_minor, cen, axis, angle, tag.getBox().size());
                            if (vote_minor >= (threshold_best_vote)) {
								goto foundEllipse;
							}
						}
						for (size_t idx = 0; idx < candidates.size(); idx++) {
							Ellipse& ell = candidates[idx];
							if (std::abs(ell.getCen().x - cen.x) < 8
							    && std::abs(ell.getCen().y - cen.y) < 8
                                && std::abs(ell.getAxis().width - major) < 8
                                && std::abs(ell.getAxis().height - minor) < 8
							    &&
							    //check angle in relation to minor/major axis
							    std::abs(ell.getAngle() - angle) < (180.0 * ell.getAxis().height) / ell.getAxis().width) {
                                // if candidates are similar, only keep best candidate
								if (ell.getVote() < vote_minor) {
									ell.setCen(cen);
                                    ell.setAxis(axis);
									ell.setAngle(angle);
									ell.setVote(vote_minor);
                                }
								break;
							}
							if (idx == candidates.size() - 1) {
                                candidates.emplace_back(vote_minor, cen, axis, angle, tag.getBox().size());

                                if (vote_minor >= (threshold_best_vote)) {
									goto foundEllipse;
								}
							}
						}

						// (12) remove associated edge pixels from ep
						//not done currently to get the best ellipse possible
						//if (it2 != ep.end()) ep.erase(it2);
					}

					// (13) clear accumulator anyway
					std::fill(accu.begin(), accu.end(), 0);
				}
			}
		}
	}

foundEllipse:
	const size_t num = std::min<size_t>(3, candidates.size());
	// sort the candidates, so that the num-best candidates are at the
	// beginning of the list
	std::partial_sort(candidates.begin(), candidates.begin() + num,
	                  candidates.end(), compareVote {});

#ifdef DEBUG_MODE_ELLIPSEFITTER
        for (size_t i = 0; i < candidates.size(); ++i) {
			Ellipse const& ell = candidates[i];
			if ((i >= num) || (ell.getVote() < _settings.get_threshold_vote())) {
					std::cout << "Ignore Ellipse With Vote " << ell.getVote() << std::endl;
					if (config::DEBUG_MODE_ELLIPSEFITTER_IMAGE) {
						visualizeEllipse(tag, ell, "ignored_ellipse");
					}
			}
		}
	}
#endif
	// remove remaining candidates
	candidates.erase(candidates.begin() + num, candidates.end());
	// remove all candidates with vote < ELLIPSEFITTER_THRESHOLD_VOTE
	candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
	                                [&](Ellipse& ell) { return ell.getVote() < _settings.get_threshold_vote(); }),
	        candidates.end());
	// add remaining candidates to tag
	for (Ellipse const& ell : candidates) {
#ifdef DEBUG_MODE_ELLIPSEFITTER
		std::cout << "Add Ellipse With Vote " << ell.getVote() << std::endl;

			if (config::DEBUG_MODE_ELLIPSEFITTER_IMAGE) {
				visualizeEllipse(tag, ell, "added_ellipse");
			}
#endif
		tag.addCandidate(TagCandidate(ell));
	}
	if (tag.getCandidatesConst().empty()) {
		tag.setValid(false);
	}
#ifdef DEBUG_MODE_ELLIPSEFITTER_IMAGE
		cv::destroyAllWindows();
#endif
#ifdef DEBUG_MODE_ELLIPSEFITTER
		std::cout << "Found " << tag.getCandidates().size()
		          << " ellipse candidates for Tag " << tag.getId() << std::endl;
#endif
}

void EllipseFitter::visualizeEllipse(Tag const& tag, Ellipse const& ell, std::string const& title) {
	cv::Mat subroiTest = tag.getOrigSubImage().clone();
	ellipse(subroiTest, ell.getCen(), ell.getAxis(), ell.getAngle(), 0, 360, cv::Scalar(0, 0, 255));
	std::string text = "Score " + std::to_string(ell.getVote());
	cv::putText(subroiTest, text, cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX_SMALL,
	            0.7, cv::Scalar(0, 255, 0));
	cv::namedWindow(title, cv::WINDOW_AUTOSIZE);
	cv::imshow(title, subroiTest);
	cv::waitKey();
}

int EllipseFitter::calcScore(Ellipse ell, cv::Mat canny){


	cv::Mat score_mat,score_mat_cp, canny_cp;
	canny_cp = canny.clone();
	cv::Mat mask = cv::Mat(canny.rows, canny.cols, canny.type());

	mask.setTo(cv::Scalar(0,0,0));


    ellipse(mask, ell.getCen(), cv::Size(cvRound(ell.getAxis().width) + 2,
                                         cvRound(ell.getAxis().height) + 2) ,
            ell.getAngle(), 0, 360, cv::Scalar(255, 255, 255), 3);

	cv::normalize(mask,mask,0,1,cv::NORM_MINMAX);
	cv::normalize(canny_cp,canny_cp,0,1, cv::NORM_MINMAX);

	cv::bitwise_and(canny_cp,mask, score_mat);
	cv::normalize(score_mat,score_mat_cp,0,255, cv::NORM_MINMAX);

#ifdef DEBUG_MODE_ELLIPSEFITTER
	cv::namedWindow("result", cv::WINDOW_NORMAL);
	cv::imshow("result", score_mat_cp);
	cv::waitKey();
#endif

	cv::Scalar score = cv::sum(score_mat);

	return static_cast<int>(score.val[0]*100);



}

cv::Mat EllipseFitter::computeCannyEdgeMap(cv::Mat const& grayImage) {
	cv::Mat localGrayImage = grayImage.clone();

	cv::GaussianBlur(localGrayImage, localGrayImage, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

	cv::Mat cannyEdgeMap;

	double low  = static_cast<double>(_settings.get_canny_initial_high() -
									  _settings.get_canny_values_distance());
	double high = static_cast<double>(_settings.get_canny_initial_high());

	const double min_mean = static_cast<double>(_settings.get_canny_mean_min());;
	const double max_mean = static_cast<double>(_settings.get_canny_mean_max());;

	std::array<double, 2> history {-1., -1.};
	double average_value;
	do
	{
		cv::Canny(localGrayImage, cannyEdgeMap,low, high);
		average_value = cv::mean(cannyEdgeMap).val[0];

		// direction of adjustment
		int direction = 0;
		if (average_value < min_mean) direction = -1;
		else if (average_value > max_mean) direction = 1;

		low  += direction * 5;
		high += direction * 5;

		// abort if iterating between two values -> endless loop
		if (std::abs(history[0] - average_value) < 0.00001) break;

		history[0] = history[1];
		history[1] = average_value;
	} while (average_value <= min_mean || average_value >= max_mean);

	return cannyEdgeMap;
}


std::vector<Tag> EllipseFitter::process(std::vector<Tag>&& taglist) {
#if defined(DEBUG_MODE_ELLIPSEFITTER) || !defined(NDEBUG) || defined(PipelineStandalone)
	for (Tag& tag : taglist) {
		detectEllipse(tag);
	}
#else
static const size_t numThreads = std::thread::hardware_concurrency() ?
	            std::thread::hardware_concurrency() * 2 : 1;
	ThreadPool pool(numThreads);
	std::vector < std::future < void >> results;
	for (Tag& tag : taglist) {
		results.emplace_back(
		            pool.enqueue([&] {
			detectEllipse(tag);
		}));
	}
	for (auto && result : results) result.get();
#endif
	return std::move(taglist);
}

} /* namespace decoder */
