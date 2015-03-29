/*
 * EllipseFitter.cpp
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#include "EllipseFitter.h"

#include "datastructure/Tag.h"
#include "datastructure/Ellipse.h"
#include "util/ThreadPool.h"

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

EllipseFitter::EllipseFitter() {
#ifdef PipelineStandalone
	this->loadConfigVars(config::DEFAULT_ELLIPSEFITTER_CONFIG);
#endif
}

void EllipseFitter::loadSettings(settings::ellipsefitter_settings_t &&settings)
{
	_settings = std::move(settings);
}

#ifdef PipelineStandalone
EllipseFitter::EllipseFitter(std::string configFile) {
	this->loadConfigVars(configFile);
}
#endif


void EllipseFitter::detectEllipse(Tag &tag) {

	const cv::Mat& subImage = tag.getOrigSubImage();
	const cv::Mat cannyImage = computeCannyEdgeMap(subImage);

	tag.setCannySubImage(cannyImage);

#ifdef DEBUG_MODE_ELLIPSEFITTER
	//cv::destroyAllWindows();

	/*	cv::namedWindow("Canny",cv::WINDOW_NORMAL);
		cv::imshow("Canny", cannyImage);
		cv::waitKey(0);

		cv::namedWindow("Original", cv::WINDOW_NORMAL);
		cv::imshow("Original", subImage);
		cv::waitKey(0);*/
#endif



	std::vector<Ellipse> candidates;

	std::vector<std::vector<cv::Point> > contours;

	std::vector<cv::Vec4i> hierarchy;
	cv::Mat subroiTest = cannyImage.clone();
	cv::cvtColor(subroiTest, subroiTest, cv::COLOR_GRAY2BGR);
	cv::findContours(cannyImage, contours, hierarchy,
	                 CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

	/// Find the rotated rectangles and ellipses for each contour

	std::vector<cv::RotatedRect> minEllipse(contours.size()+1);
	std::vector<std::vector<cv::Point> > hulls(contours.size()+1) ;

	std::vector<cv::Point> merged_contour_points;


	for (size_t i = 0; i < contours.size(); i++) {
		std::vector<cv::Point> hull;
		if(contours[i].size() > 5){
			//cv::convexHull(contours[i],hulls[i]);
			minEllipse[i] = cv::fitEllipse(contours[i]);

		}
		for (size_t j = 0; j < contours[i].size(); j++) {
			merged_contour_points.push_back(contours[i][j]);
		}

	}


	/// Draw  ellipses of the contours
	cv::Mat drawing;
	cv::Scalar color;

	for( size_t i = 0; i< contours.size(); i++ )
	{
		cv::RotatedRect ell = minEllipse[i];

		const int width  = static_cast<int>(ell.size.width) / 2 + 1;
		const int height = static_cast<int>(ell.size.height) / 2 + 1;

		const cv::Size s(std::max(width, height), std::min(width, height));

		Ellipse new_ell(0, ell.center, s, ell.angle, subImage.size());

		///check for the right features to be a comb
		if ((ell.size.width >= _settings.get_min_major_axis() &&
		     ell.size.width >= _settings.get_min_minor_axis()) &&
		    (ell.size.height <= _settings.get_max_major_axis() &&
		     ell.size.width  <= _settings.get_max_minor_axis()))
		{
			color= 	cv::Scalar (0,0,255);
			new_ell.setVote(calcScore(new_ell,cannyImage));
			candidates.push_back(new_ell);
		} else {
#ifdef DEBUG_MODE_ELLIPSEFITTER
			color = 	cv::Scalar  (255,0,0);
			new_ell.setVote(0);
#endif
		}

#ifdef DEBUG_MODE_ELLIPSEFITTER
		ellipse(subroiTest, new_ell.getCen(), new_ell.getAxis(), new_ell.getAngle(), 0, 360,
		        color);

#endif
	}
#ifdef DEBUG_MODE_ELLIPSEFITTER
	cv::namedWindow("All", cv::WINDOW_NORMAL);
	cv::imshow("All", subroiTest);
	cv::waitKey();
	std::vector<cv::Vec3f> circles;
#endif



	const size_t num = std::min<size_t>(3, candidates.size());
#ifdef DEBUG_MODE_ELLIPSEFITTER

	/*for (size_t i = 0; i < candidates.size(); ++i) {
		Ellipse const& ell = candidates[i];
		if ((i >= num) || (ell.getVote() < _settings.get_threshold_vote())) {

			visualizeEllipse(tag, ell, "ignored_ellipse ");
		}
	}*/

#endif
	// remove remaining candidates
	candidates.erase(candidates.begin() + num, candidates.end());
	// remove all candidates with vote < RECOGNIZER_THRESHOLD_VOTE
	candidates.erase(
	            std::remove_if(candidates.begin(), candidates.end(),
	                           [&](Ellipse& ell) {return ell.getVote() < _settings.get_threshold_vote();}),
	        candidates.end());
	// add remaining candidates to tag
	for (Ellipse const& ell : candidates) {
#ifdef DEBUG_MODE_ELLIPSEFITTER

		std::cout << "Add Ellipse With Vote " << ell.getVote() << std::endl;

		visualizeEllipse(tag, ell, "added_ellipse");

#endif
		tag.addCandidate(TagCandidate(ell));
	}
	if (tag.getCandidatesConst().empty()) {
		if(this->_settings.get_use_xie_as_fallback()){
#ifdef DEBUG_MODE_ELLIPSEFITTER
			std::cout<< "start Xie-Detection" << std::endl;
#endif
			detectXieEllipse(tag);
		}else{
			tag.setValid(false);
		}
	}
#ifdef DEBUG_MODE_ELLIPSEFITTER

	std::cout << "Found " << tag.getCandidates().size()
	          << " ellipse candidates for Tag " << tag.getId() << std::endl;

#endif
}


/**
 * @param tag for which ellipses should be detected
 */
void EllipseFitter::detectXieEllipse(Tag &tag) {
	const double ellipsefitter_max_minor = _settings.get_max_minor_axis();
	const double ellipsefitter_max_major = _settings.get_max_major_axis();
	const double ellipsefitter_min_major = _settings.get_min_major_axis();
	const double ellipsefitter_min_minor = _settings.get_min_minor_axis();

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
	std::vector<int> accu((this->_settings.get_max_minor_axis()) / 2 + 1);

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
								// (8) increment the accumulator for the minor axis' half length (b) just estimated
								if (b2 <= ellipsefitter_max_minor && b2 >= ellipsefitter_min_minor) {
									accu[cvRound(b) - 1] += 1;
								}
							}
						}
					}

					// (10) find the maximum within the accumulator, is it above the threshold?
					const auto max_it  = std::max_element(accu.begin(), accu.end());
					const auto max_ind = std::distance(accu.begin(), max_it);
					int vote_minor     = *max_it;

					if (vote_minor >= threshold_edge_pixels) {
						// (11) save ellipse parameters
						const cv::Point2i cen(cvRound(centerx), cvRound(centery));
						const cv::Size axis(cvRound(a), max_ind);
						const double angle = (alpha * 180) / CV_PI;

						const float j = cvRound(a);
						const float n = max_ind;

						// more "circular" ellipses are weighted more than very thin ellipses
						vote_minor = static_cast<int>(vote_minor * (50 * n / j));

						if (candidates.size() == 0) {
							candidates.emplace_back(vote_minor, cen, axis, angle, tag.getBox().size());
							if (vote_minor >= threshold_best_vote) {
								goto foundEllipse;
							}
						}
						for (size_t idx = 0; idx < candidates.size(); idx++) {
							Ellipse& ell = candidates[idx];
							if (std::abs(ell.getCen().x - cen.x) < 8
							    && std::abs(ell.getCen().y - cen.y) < 8
							    && std::abs(ell.getAxis().width - j) < 8
							    && std::abs(ell.getAxis().height - n) < 8
							    &&
							    //check angle in relation to minor/major axis
							    std::abs(ell.getAngle() - angle) < (180.0 * ell.getAxis().height) / ell.getAxis().width) {
								if (ell.getVote() < vote_minor) {
									ell.setCen(cen);
									ell.setAxis(cv::Size2f(j, n));
									ell.setAngle(angle);
									ell.setVote(vote_minor);
								}
								break;
							}
							if (idx == candidates.size() - 1) {
								candidates.emplace_back(vote_minor, cen, axis, angle, tag.getBox().size());

								if (vote_minor >= threshold_best_vote) {
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

#ifdef PipelineStandalone
	if (config::DEBUG_MODE_ELLIPSEFITTER) {
		for (size_t i = 0; i < candidates.size(); ++i) {
			Ellipse const& ell = candidates[i];
			if ((i >= num) || (ell.getVote() < _settings.get_threshold_vote())) {
				if (config::DEBUG_MODE_ELLIPSEFITTER) {
					std::cout << "Ignore Ellipse With Vote " << ell.getVote() << std::endl;
					if (config::DEBUG_MODE_ELLIPSEFITTER_IMAGE) {
						visualizeEllipse(tag, ell, "ignored_ellipse");
					}
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
#endif
#ifdef PipelineStandalone
		if (config::DEBUG_MODE_ELLIPSEFITTER) {

			if (config::DEBUG_MODE_ELLIPSEFITTER_IMAGE) {
				visualizeEllipse(tag, ell, "added_ellipse");
			}
		}
#endif
		tag.addCandidate(TagCandidate(ell));
	}
	if (tag.getCandidatesConst().empty()) {
		tag.setValid(false);
	}
#ifdef PipelineStandalone
	if (config::DEBUG_MODE_ELLIPSEFITTER_IMAGE) {
		cv::destroyAllWindows();
	}
	if (config::DEBUG_MODE_ELLIPSEFITTER) {
		std::cout << "Found " << tag.getCandidates().size()
		          << " ellipse candidates for Tag " << tag.getId() << std::endl;
	}
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


	ellipse(mask, ell.getCen(), cv::Size(ell.getAxis().width +2,ell.getAxis().height+2) , ell.getAngle(), 0, 360,
	        cv::Scalar(255, 255, 255), 3);


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

	cv::GaussianBlur(localGrayImage, localGrayImage, cv::Size(3, 3), 0, 0,
	                 cv::BORDER_DEFAULT);

	cv::Mat cannyEdgeMap;

	double low=static_cast<double>(this->_settings.get_canny_initial_high()- this->_settings.get_canny_values_distance());
	double high = static_cast<double>(this->_settings.get_canny_initial_high());


	double average_old = -1, average_old_2 =-1;
	double min_mean = static_cast<double>(this->_settings.get_canny_mean_min());;
	double max_mean = static_cast<double>(this->_settings.get_canny_mean_max());;

canny:
	cv::Canny(localGrayImage, cannyEdgeMap,low, high);

	cv::Scalar mean = cv::mean(cannyEdgeMap);
	double average_value = mean.val[0];
	//std::cout << "canny mean " << mean.val[0] << " "<< std::endl;


	if (average_old_2 != average_value ||  average_old == average_old_2 ) {

		if (average_value < min_mean) {
			if(low > 0)
				low-=5;
			if(high > 0)
				high-=5;
			average_old_2 = average_old;
			average_old = average_value;

			//std::cout << "new values " << high << " " <<low << std::endl;

			goto canny;
		} else if (average_value > max_mean) {
			if(low < 255)
				low+= 5;
			if(high < 255)
				high+=5;
			average_old_2 = average_old;
			average_old = average_value;
			//std::cout << "new values " << high << " " <<low << std::endl;
			goto canny;
		}
	}



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

#ifdef PipelineStandalone
void EllipseFitter::loadConfigVars(std::string filename) {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(filename, pt);

	_settings.max_major_axis =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".max_major_axis");
	_settings.min_major_axis =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".min_major_axis");
	_settings.max_minor_axis =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".max_minor_axis");
	_settings.min_minor_axis =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".min_minor_axis");
	_settings.threshold_edge_pixels =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".threshold_edge_pixels");
	_settings.threshold_vote =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".threshold_vote");
	_settings.threshold_best_vote =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".threshold_best_vote");
	_settings.canny_threshold_high =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".canny_threshold_low");
	_settings.canny_threshold_low =
	        pt.get<int>(config::APPlICATION_ENVIROMENT + ".canny_threshold_high");
}
#endif

} /* namespace decoder */
