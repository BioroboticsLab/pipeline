#ifndef TAGRECOGNITION_H
#define	TAGRECOGNITION_H

#include <vector>
#include <iostream>
#include "Beesbook.h"
#include "BeesbookGUI.h"
#include "Supplementary.h"


using namespace cv;

bool tagRecognition( Mat & cannyEdgeMap, Rect &boundingBox, Mat &image, Ellipse &currentEllipse, Mat &subroi, Mat &ellcall, Mat &ellcallt , vector<Ellipse> &ellipseCandidate);

void ellipseTransformation( Mat &subroi, Ellipse &ellipse, Mat &elMask );

vector<Point2f> getOrientationVector( Mat &roi, Point3f &circle );


#endif	/* TAGRECOGNITION_H */
