/*
 * ContourInfo.cpp
 *
 *  Created on: Oct 16, 2014
 *      Author: Alon Yaari
 */

#include "ContourInfo.h"

using namespace cv;

ContourInfo::ContourInfo()
{
	index           = 0;
	size            = 0;
	isConvex        = false;
	moreThan2Points = false;
	moreThan4Points = false;
	parentIx        = -1;
	area            = 0.0;
	radius          = 0.0;
	center          = Point(0, 0);
	isAParent       = false;
	isAChild        = false;
	isTriangle      = false;
	isCircle        = false;
	isCross         = false;
}

ContourInfo::ContourInfo(size_t ix, std::vector<std::vector<cv::Point> >* fullList, int hier3)
{
	index           = ix;
	isTriangle		= false;
	isCircle		= false;
	isCross			= false;
	size            = fullList->at(ix).size();
	moreThan2Points = (size > 2);
	if (moreThan2Points) {
		minEnclosingCircle(fullList->at(ix), center, radius);
		isConvex    = isContourConvex(fullList->at(ix));
		area        = fabs(contourArea(Mat(fullList->at(ix)))); }
	else {
		isConvex    = false;
		area        = 0.0;
		radius      = 0.0; }
	moreThan4Points = (size > 4);
	if (moreThan4Points)
		rr          = fitEllipse(fullList->at(ix));
	if (GetIsBigEnough())
		parentIx    = hier3;
	else
		parentIx    = -1;
}

void ContourInfo::SetAsParent()
{
	isAParent = (moreThan2Points && GetIsBigEnough());
}



















//
