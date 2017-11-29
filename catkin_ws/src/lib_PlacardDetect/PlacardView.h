/*
 * PlacardView.h
 *
 *  Created on: Sep 14, 2014
 *      Author: Alon Yaari
 */

#ifndef PLACARDVIEW_H_
#define PLACARDVIEW_H_



#include <iostream>
#include <vector>
#include <map>
#include "ContourInfo.h"

#define DEG120    2.094395	// 120 degrees in rad
#define DEG100    1.745329	// 100 degrees in rad
#define DEG140    2.443461	// 140 degrees in rad


//#define DEBUGGING

struct corner {
	cv::Point A;
	cv::Point B;
	cv::Point C;
};

struct placardFound {
	cv::Point2f  location;
	bool         valid;
	float        radius;
};

class PlacardView {

public:
	PlacardView();
	~PlacardView() { writer.release(); };
	bool ProcessImage(cv::Mat& img, placardFound& circle, placardFound& triangle, placardFound& cross);
	std::string DebuggingStats();

private:
	void   CleanContourGeometry();
	void   CalcContourDetails();
	void   IdentifyParents();
	double Distance(double dX0, double dY0, double dX1, double dY1);
	void   ImagePreprocess();
	void   ContourProcessing();
	bool   GoodChildToParentAreaRatio(size_t ixChild, size_t ixParent);
	bool   TriangleDiscovery1(ContourInfo ci);
	bool   TriangleDiscovery2(ContourInfo ci);
	bool   CircleDiscoveryClock(ContourInfo ci);
	bool   CircleComparison(ContourInfo ci);
	bool   CruciformComparison(ContourInfo ci);
	bool   CruciformMatching();
	void   MatchOneSizedCross(ContourInfo ci, double& minVal, cv::Point& minLoc, cv::Point& otherCorner);
	void   IdentifyPlacards();
	void   ParentMask();
	void   PlacardDiscovery();
	void   CreateTemplateCross();

	cv::VideoWriter writer;

	size_t contourCount;

	// CV images between major processing steps
	cv::Mat inFrame;					// Color input image to process
	cv::Mat labImage;					// Image converted to Lab color format
	cv::Mat imgBinary;					// Binary image as result of preprocessing
	cv::Mat imgBinaryROI;
	cv::Mat templateCross;

#ifdef DEBUGGING
	cv::Mat imgClrBinary;
#endif

	// Vectors for managing contours
	std::vector<std::vector<cv::Point> >   rawContours;    // First pass, detecting all contours
	std::vector<std::vector<cv::Point> >   approxContours;
	std::vector<cv::Vec4i>                 hier;           // Parent-child relationship of contours vector
    std::vector<ContourInfo>               contourDetails;

	std::vector<std::vector<cv::Point> > circles;
	std::vector<std::vector<cv::Point> > triangles;
	std::vector<std::vector<cv::Point> > cruciforms;


	// Adjustable parameters
	cv::Size	claheTileSize;			// CLAHE tile grid size
	int			claheClipLimit;			// CLAHE clip limit
	cv::Size	blurSize;				// Gaussian blur tile size
	int			blurSigma;				// Sigma x and y value for gausian blur
	int			threshVal;				// Binary threshold cutoff value
	cv::Size	morphologySize;			// morphology operations tile size
	int         morphKsize;				// morphology tile kernel size
	cv::Point	morphologyPoint;		// morphology tile center location



};

#endif












//


