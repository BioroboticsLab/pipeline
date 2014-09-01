/*
 s * Recognizer.cpp
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#include "Recognizer.h"

double pointDistance(Point2d p, Point2d q) {
	return (sqrt((q.y - p.y) * (q.y - p.y) + (q.x - p.x) * (q.x - p.x)));
}

namespace decoder {

/**************************************
 *
 * 			constructor
 *
 **************************************/

Recognizer::Recognizer() {
	this->loadConfigVars(config::DEFAULT_RECOGNIZER_CONFIG);

}

Recognizer::Recognizer(string configFile) {
	this->loadConfigVars(configFile);

}

Recognizer::~Recognizer() {
	// TODO Auto-generated destructor stub
}

/**************************************
 *
 * 			stuff
 *
 **************************************/

/**
 * @param tag for which ellipses should be detected
 */
void Recognizer::detectXieEllipse(Tag &tag) {

	Mat cannyImage;
	tag.getCannySubImage().copyTo(cannyImage);

	//edge_pixel array, all edge pixels are stored in this array
	vector<Point2i> ep;
	ep.reserve(cv::countNonZero(cannyImage) + 1);

	vector<Ellipse>::iterator cand;
	int max_ind;

	double a, b, tau, d, f, alpha, costau, sintau;

	Point2d p, p1, p2, center;
	vector<Ellipse> candidates;

	// (1) all white (being edge) pixels are written into the ep array
	MatIterator_<char> mit, end;
	for (mit = cannyImage.begin<char>(), end = cannyImage.end<char>();
			mit != end; mit++) {
		if (*mit.ptr == 255) {
			ep.push_back(mit.pos());
		}
	}

	// (2) initiate the accumulator array
	std::vector<int> accu((this->RECOGNIZER_MAX_MINOR) / 2 + 1);
	int vote_minor;

	// (3) loop through each edge pixel in ep
	for (std::vector<Point2i>::iterator it = ep.begin(); it != ep.end(); it++) {
		p1.x = (*it).x;
		p1.y = (*it).y;

		// (4) loop through each other edge pixel in ep and propose a major axis between p1 and p2
		for (std::vector<Point2i>::iterator it2 = ep.begin(); it2 != ep.end();
				it2++) {
			if ((it2 > it) && (*it) != (*it2)) {
				p2.x = (*it2).x;
				p2.y = (*it2).y;
				// the proposed ellipse' length of the major axis lies between this->RECOGNIZER_MIN_MAJOR and this->RECOGNIZER_MAX_MAJOR
				if (pointDistance(p1, p2) > this->RECOGNIZER_MIN_MAJOR
						&& pointDistance(p1, p2) < this->RECOGNIZER_MAX_MAJOR) {
					// (5) calculate the ellipse' center, half length of the major axis (a) and orientation (alpha) based on p1 and p2
					center.x = (p1.x + p2.x) / 2;
					center.y = (p1.y + p2.y) / 2;
					a = pointDistance(p1, p2) / 2;
					alpha = atan2((p2.y - p1.y), (p2.x - p1.x));
					// (6) loop through each third edge pixel eventually lying on the ellipse
					for (std::vector<Point2i>::iterator it3 = ep.begin();
							it3 != ep.end(); it3++) {

						if ((*it3) != (*it2) && (*it3) != (*it)) {
							p.x = (*it3).x;
							p.y = (*it3).y;
							d = pointDistance(p, center);

							if (d > this->RECOGNIZER_MIN_MINOR / 2.0
									&& d <= a) {
								// (7) estimate the half length of the minor axis (b)
								f = pointDistance(p, p2);
								costau = (a * a + d * d - f * f) / (2 * a * d);
								tau = acos(costau);
								sintau = sin(tau);
								b = sqrt(
										(a * a * d * d * sintau * sintau)
												/ ((a * a)
														- (d * d * costau
																* costau)));

								// (8) increment the accumulator for the minor axis' half length (b) just estimated
								if (b <= this->RECOGNIZER_MAX_MINOR / 2.0
										&& b
												>= this->RECOGNIZER_MIN_MINOR
														/ 2.0) {
									accu[cvRound(b) - 1] += 1;
									//support[cvRound(b) - 1].push_back((*it3));
								}
							}
						}
					}

					// (10) find the maximum within the accumulator, is it above the threshold?
					max_ind = std::max_element(accu.begin(), accu.end())
							- accu.begin() + 1;
					vote_minor = (*(max_ind + accu.begin() - 1));

					if (vote_minor >= this->RECOGNIZER_THRESHOLD_EDGE) {

						// (11) save ellipse parameters
						Point2i cen = Point2i(cvRound(center.x),
								cvRound(center.y));

						Size axis = Size(cvRound(a), max_ind);

						double angle = (alpha * 180) / CV_PI;

						//scoring:
						float jm = this->RECOGNIZER_MIN_MAJOR
								+ (this->RECOGNIZER_MAX_MAJOR
										- this->RECOGNIZER_MIN_MAJOR) / 2.0;
						float nm = this->RECOGNIZER_MIN_MINOR
								+ (this->RECOGNIZER_MAX_MINOR
										- this->RECOGNIZER_MIN_MINOR) / 2.0;
						float j = cvRound(a);
						float n = max_ind;

						// more "circular" ellipses are weighted more than very thin ellipses
						//std::cout << n/j << std::endl;
						vote_minor = vote_minor * (50 * n / j);

						if (candidates.size() == 0) {
							candidates.push_back(
									Ellipse(vote_minor, cen, axis, angle));
						}
						for (int el = 0; el < candidates.size(); el++) {
							if (abs(candidates[el].cen.x - cen.x) < 8
									&& abs(candidates[el].cen.y - cen.y) < 8
									&& abs(candidates[el].axis.width - j) < 8
									&& abs(candidates[el].axis.height - n) < 8
									&&

									//check angle in relation to minor/major axis
									abs(candidates[el].angle - angle)
											< (180.0
													* candidates[el].axis.height)
													/ candidates[el].axis.width) {

								if (candidates[el].vote < vote_minor) {
									candidates[el].cen.x = cen.x;
									candidates[el].cen.y = cen.y;
									candidates[el].axis.width = j;
									candidates[el].axis.height = n;
									candidates[el].angle = angle;
									candidates[el].vote = vote_minor;
								}
								break;
							}
							if (el == candidates.size() - 1) {

								candidates.push_back(
										Ellipse(vote_minor, cen, axis, angle));
							}
						}

						// (12) remove associated edge pixels from ep
						//not done currently to get the best ellipse possible
						//if (it2 != ep.end()) ep.erase(it2);
					}

					// (13) clear accumulator anyway
					for (std::vector<int>::iterator ini = accu.begin();
							ini != accu.end(); ini++) {
						(*ini) = 0;
					}
				}
			}
		}
		//if (it != ep.end()) ep.erase(it);
	}

	// sort the candidates list according to their vote
	std::sort(candidates.begin(), candidates.end());

	for (int i = 0; i < candidates.size(); i++) {
		Ellipse currentEllipse = candidates[i];
		if (currentEllipse.vote < this->RECOGNIZER_THRESHOLD_VOTE) {
			candidates.erase(candidates.begin() + i);

		} else {
			if (config::DEBUG_MODE_RECOGNIZER) {

				Mat subimageoutput;
				tag.getOrigSubImage().copyTo(subimageoutput);
				ellipse(subimageoutput, currentEllipse.cen, currentEllipse.axis,
						currentEllipse.angle, 0, 360, Scalar(255, 0, 0), 2);
				string text = "" + to_string(currentEllipse.vote);
				cv::putText(subimageoutput, text, Point(10, 10),
						FONT_HERSHEY_COMPLEX_SMALL, 0.7, Scalar(255, 0, 0));

				namedWindow("Ellipse", WINDOW_NORMAL);
				imshow("Ellipse", subimageoutput);
				waitKey();
				Mat subimageoutputcanny;
				tag.getCannySubImage().copyTo(subimageoutputcanny);
				ellipse(subimageoutputcanny, currentEllipse.cen,
						currentEllipse.axis, currentEllipse.angle, 0, 360,
						Scalar(255, 0, 0), 2);

				namedWindow("Ellipse", WINDOW_NORMAL);
				imshow("Ellipse", subimageoutputcanny);
				waitKey();
			}
			TagCandidate c = TagCandidate(candidates[i]);
			tag.addCandidate(c);
		}
	}

	if (tag.getCandidates().size() == 0) {
		tag.setValid(false);
	}

	if (config::DEBUG_MODE_RECOGNIZER) {
		std::cout << "Found " << candidates.size() << " ellipse candidates for "
				<< tag.getId() << std::endl;
	}

}

void Recognizer::process(TagList &taglist) {
	cout << "Start Recognizing" << endl;
	for (int i = 0; i < taglist.size(); i++) {
		cout << "Start Recognizing Tag " << i << endl;
		Tag t = taglist.getTag(i);
		this->detectXieEllipse(t);
	}

}

void Recognizer::loadConfigVars(string filename) {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini(filename, pt);

	this->RECOGNIZER_MAX_MAJOR = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".max_major_axis");
	this->RECOGNIZER_MIN_MAJOR = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".min_major_axis");
	this->RECOGNIZER_MAX_MINOR = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".max_minor_axis");
	this->RECOGNIZER_MIN_MINOR = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".min_minor_axis");
	this->RECOGNIZER_THRESHOLD_EDGE = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".threshold_edge_pixels");
	this->RECOGNIZER_THRESHOLD_VOTE = pt.get<int>(
			config::APPlICATION_ENVIROMENT + ".threshold_vote");

}

} /* namespace decoder */
