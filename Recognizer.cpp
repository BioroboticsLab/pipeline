/*
 * Recognizer.cpp
 *
 *  Created on: 12.08.2014
 *      Author: mareikeziese
 */

#include "Recognizer.h"

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
    inline bool operator()(decoder::Ellipse const& a, decoder::Ellipse const& b)
    {
        return a.vote > b.vote;
    }
};
}

namespace decoder {
Recognizer::Recognizer() {
#ifdef PipelineStandalone
    this->loadConfigVars(config::DEFAULT_RECOGNIZER_CONFIG);
#else
    loadConfigVars();
#endif
}

#ifdef PipelineStandalone
Recognizer::Recognizer(string configFile) {
    this->loadConfigVars(configFile);
}
#endif

/**
 * @param tag for which ellipses should be detected
 */
void Recognizer::detectXieEllipse(Tag &tag) {
    const double recognizer_max_minor = RECOGNIZER_MAX_MINOR;
    const double recognizer_max_major = RECOGNIZER_MAX_MAJOR;
    const double recognizer_min_major = RECOGNIZER_MIN_MAJOR;
    const double recognizer_min_minor = RECOGNIZER_MIN_MINOR;

    const Mat& subImage  = tag.getOrigSubImage();
    const Mat cannyImage = computeCannyEdgeMap(subImage);

    tag.setCannySubImage(cannyImage);

    //edge_pixel array, all edge pixels are stored in this array
    std::vector<Point2i> ep;
    ep.reserve(cv::countNonZero(cannyImage) + 1);

    std::vector<Ellipse> candidates;

    // (1) all white (being edge) pixels are written into the ep array
    MatConstIterator_<unsigned char> mit, end;
    for (mit = cannyImage.begin<unsigned char>(), end = cannyImage.end<unsigned char>();
      mit != end; ++mit) {
        if (*mit.ptr == 255) {
            ep.push_back(mit.pos());
        }
    }

    // (2) initiate the accumulator array
    std::vector<int> accu((this->RECOGNIZER_MAX_MINOR) / 2 + 1);

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
                // the proposed ellipse' length of the major axis lies between this->RECOGNIZER_MIN_MAJOR and this->RECOGNIZER_MAX_MAJOR
                const double dist = pointDistance(p1xf, p1yf, p2xf, p2yf);
                if (dist > recognizer_min_major && dist < recognizer_max_major) {
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
                            if (d <= a && d > recognizer_min_minor / 2.0) {
                                // (7) estimate the half length of the minor axis (b)
                                const double f      = pointDistanceNoSqrt(pxf, pyf, p2xf, p2yf);
                                const double costau = (a * a + d * d - f) / (2 * a * d);
                                const double tau    = acos(costau);
                                const double sintau = sin(tau);
                                const double b      = (a * d * sintau) / sqrt(((a * a) - (d * d * costau * costau)));
                                const double b2     = 2.0 * b;
                                // (8) increment the accumulator for the minor axis' half length (b) just estimated
                                if (b2 <= recognizer_max_minor && b2 >= recognizer_min_minor) {
                                    accu[cvRound(b) - 1] += 1;
                                }
                            }
                        }
                    }

                    // (10) find the maximum within the accumulator, is it above the threshold?
                    const auto max_it  = std::max_element(accu.begin(), accu.end());
                    const auto max_ind = std::distance(accu.begin(), max_it);
                    int vote_minor     = *max_it;

                    if (vote_minor >= RECOGNIZER_THRESHOLD_EDGE) {
                        // (11) save ellipse parameters
                        const Point2i cen(cvRound(centerx), cvRound(centery));
                        const Size axis(cvRound(a), max_ind);
                        const double angle = (alpha * 180) / CV_PI;

                        const float j = cvRound(a);
                        const float n = max_ind;

                        // more "circular" ellipses are weighted more than very thin ellipses
                        vote_minor = vote_minor * (50 * n / j);

                        if (candidates.size() == 0) {
                            candidates.emplace_back(vote_minor, cen, axis, angle);
                            if (vote_minor >= RECOGNIZER_THRESHOLD_BEST_VOTE) {
                                goto foundEllipse;
                            }
                        }
                        for (size_t idx = 0; idx < candidates.size(); idx++) {
                            Ellipse& ell = candidates[idx];
                            if (abs(ell.cen.x - cen.x) < 8
                              && abs(ell.cen.y - cen.y) < 8
                              && abs(ell.axis.width - j) < 8
                              && abs(ell.axis.height - n) < 8
                              &&
                              //check angle in relation to minor/major axis
                              abs(ell.angle - angle) < (180.0 * ell.axis.height) / ell.axis.width) {
                                if (ell.vote < vote_minor) {
                                    ell.cen.x       = cen.x;
                                    ell.cen.y       = cen.y;
                                    ell.axis.width  = j;
                                    ell.axis.height = n;
                                    ell.angle       = angle;
                                    ell.vote        = vote_minor;
                                }
                                break;
                            }
                            if (idx == candidates.size() - 1) {
                                candidates.emplace_back(vote_minor, cen, axis, angle);

                                if (vote_minor >= RECOGNIZER_THRESHOLD_BEST_VOTE) {
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
    if (config::DEBUG_MODE_RECOGNIZER) {
        for (size_t i = 0; i < candidates.size(); ++i) {
            Ellipse const& ell = candidates[i];
            if ((i >= num) || (ell.vote < RECOGNIZER_THRESHOLD_VOTE)) {
                if (config::DEBUG_MODE_RECOGNIZER) {
                    std::cout << "Ignore Ellipse With Vote " << ell.vote << std::endl;
                    if (config::DEBUG_MODE_RECOGNIZER_IMAGE) {
                        visualizeEllipse(tag, ell, "ignored_ellipse");
                    }
                }
            }
        }
    }
#endif
    // remove remaining candidates
    candidates.erase(candidates.begin() + num, candidates.end());
    // remove all candidates with vote < RECOGNIZER_THRESHOLD_VOTE
    candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
      [&](Ellipse& ell) { return ell.vote < RECOGNIZER_THRESHOLD_VOTE; }),
      candidates.end());
    // add remaining candidates to tag
    for (Ellipse const& ell : candidates) {
#ifdef PipelineStandalone
        if (config::DEBUG_MODE_RECOGNIZER) {
            std::cout << "Add Ellipse With Vote " << ell.vote << std::endl;
            if (config::DEBUG_MODE_RECOGNIZER_IMAGE) {
                visualizeEllipse(tag, ell, "added_ellipse");
            }
        }
#endif
        tag.addCandidate(TagCandidate(ell));
    }
    if (tag.getCandidates().size() == 0) {
        tag.setValid(false);
    }
#ifdef PipelineStandalone
    if (config::DEBUG_MODE_RECOGNIZER_IMAGE) {
        destroyAllWindows();
    }
    if (config::DEBUG_MODE_RECOGNIZER) {
        std::cout << "Found " << tag.getCandidates().size()
                  << " ellipse candidates for Tag " << tag.getId() << std::endl;
    }
#endif
}

void Recognizer::visualizeEllipse(Tag const& tag, Ellipse const& ell, std::string const& title) {
    Mat subroiTest;
    tag.getOrigSubImage().copyTo(subroiTest);
    ellipse(subroiTest, ell.cen, ell.axis, ell.angle, 0, 360, Scalar(0, 0, 255));
    string text = "Score " + std::to_string(ell.vote);
    cv::putText(subroiTest, text, Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL,
      0.7, Scalar(0, 255, 0));
    namedWindow(title, WINDOW_NORMAL);
    imshow(title, subroiTest);
    waitKey();
}

Mat Recognizer::computeCannyEdgeMap(Mat grayImage) {
    Mat localGrayImage;
    grayImage.copyTo(localGrayImage);

    cv::GaussianBlur(localGrayImage, localGrayImage, Size(3, 3), 0, 0,
      BORDER_DEFAULT);

    Mat cannyEdgeMap;
    Canny(localGrayImage, cannyEdgeMap, this->RECOGNIZER_LCANNYTHRES,
      this->RECOGNIZER_HCANNYTHRES);
#ifdef PipelineStandalone
    if (config::DEBUG_MODE_RECOGNIZER_IMAGE) {
        namedWindow("Canny", WINDOW_NORMAL);
        imshow("Canny", cannyEdgeMap);
        waitKey(0);
        destroyWindow("Canny");
    }
#endif

    return cannyEdgeMap;
}

std::vector<Tag> Recognizer::process(std::vector<Tag>&& taglist) {
    static const size_t numThreads = 8;
    ThreadPool pool(numThreads);
    std::vector < std::future < void >> results;
    for (Tag& tag : taglist) {
        results.emplace_back(
            pool.enqueue([&] {
                detectXieEllipse(tag);
            }));
    }
    for (auto && result : results) result.get();
    return taglist;
}

#ifdef PipelineStandalone
void Recognizer::loadConfigVars(string filename) {
    //TODO: add config in Tracker
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
    this->RECOGNIZER_THRESHOLD_BEST_VOTE = pt.get<int>(
        config::APPlICATION_ENVIROMENT + ".threshold_best_vote");
    this->RECOGNIZER_LCANNYTHRES = pt.get<int>(
        config::APPlICATION_ENVIROMENT + ".canny_threshold_low");
    this->RECOGNIZER_HCANNYTHRES = pt.get<int>(
        config::APPlICATION_ENVIROMENT + ".canny_threshold_high");
}
#endif

void Recognizer::loadConfigVars()
{
    RECOGNIZER_MAX_MAJOR = RecognizerParams::max_major_axis;
    RECOGNIZER_MIN_MAJOR = RecognizerParams::min_major_axis;
    RECOGNIZER_MAX_MINOR = RecognizerParams::max_minor_axis;
    RECOGNIZER_MIN_MINOR = RecognizerParams::min_minor_axis;
    RECOGNIZER_THRESHOLD_BEST_VOTE = RecognizerParams::threshold_best_vote;
    RECOGNIZER_THRESHOLD_EDGE      = RecognizerParams::threshold_edge_pixels;
    RECOGNIZER_THRESHOLD_VOTE      = RecognizerParams::threshold_vote;
    RECOGNIZER_HCANNYTHRES         = RecognizerParams::canny_threshold_high;
    RECOGNIZER_LCANNYTHRES         = RecognizerParams::canny_threshold_low;
}
} /* namespace decoder */
