/*
 * PlacardView.cpp
 *
 *  Created on: Sep 14, 2014
 *      Author: Alon Yaari
 */

#include "PlacardView.h"

using namespace std;
using namespace cv;

#define CLOCK_POINTS 8


PlacardView::PlacardView()
{
	contourCount    = 0;
    claheTileSize   = Size(3, 3);
    claheClipLimit  = 4;
    blurSize        = Size(1, 1); // Blur size values must be ODD
    blurSigma       = 4;
    threshVal       = 235;
    morphKsize      = 4; // (2 * baseSize + 1)
    morphologySize  = Size(morphKsize, morphKsize);
    morphologyPoint = Point(-1, -1);  // (-1, -1) refers to the center point
    //CreateTemplateCross();

#ifdef DEBUGGING
    cvNamedWindow("PLACARDS", WINDOW_AUTOSIZE);
#endif
}

double PlacardView::Distance(double dX0, double dY0, double dX1, double dY1)
{
    return sqrt((dX1 - dX0) * (dX1 - dX0) + (dY1 - dY0) * (dY1 - dY0));
}


bool PlacardView::ProcessImage(Mat& img, placardFound& circle, placardFound& triangle, placardFound& cross)
{
    rawContours.clear();
    hier.clear();
    approxContours.clear();
    contourDetails.clear();
    circles.clear();
    triangles.clear();
    cruciforms.clear();
    circle.valid   = false;
    triangle.valid = false;
    cross.valid    = false;
    inFrame = img;
    if (inFrame.empty())
        return false;

    // Take the input color image and process it into a thresholded binary image
    ImagePreprocess();

    // Find the contours in the binary image
    ContourProcessing();

    // Process the contours to find the placard shapes
    PlacardDiscovery();

    // Special process to find crosses
    CruciformMatching();

    IdentifyPlacards();


    circle.valid = (circles.size() > 0);
    if (circle.valid)
        minEnclosingCircle(circles[0], circle.location, circle.radius);
    triangle.valid = (triangles.size() > 0);
    if (triangle.valid)
        minEnclosingCircle(triangles[0], triangle.location, triangle.radius);
    cross.valid = (cruciforms.size() > 0);
    if (cross.valid)
        minEnclosingCircle(cruciforms[0], cross.location, cross.radius);
    return true;
}

void PlacardView::IdentifyPlacards()
{

	for (size_t i = 0; i < contourCount; i++) {
		ContourInfo ci = contourDetails[i];
		if (ci.IsTriangle())
			triangles.push_back(approxContours[ci.Get_index()]);
		if (ci.IsCircle())
			circles.push_back(approxContours[ci.Get_index()]);
		if (ci.IsCross())
			cruciforms.push_back(approxContours[ci.Get_index()]); }
#ifdef DEBUGGING
    Scalar clr = Scalar(0, 0, 255);
    drawContours(inFrame, circles, -1, clr, 2, CV_AA);
    clr = Scalar(0, 255, 0);
    drawContours(inFrame, triangles, -1, clr, 2, CV_AA);
    clr = Scalar(255, 0, 0);
    drawContours(inFrame, cruciforms, -1, clr, 2, CV_AA);
    imshow("PLACARDS", inFrame);
    cvWaitKey(1);
#endif

}

void PlacardView::ImagePreprocess()
{
    // Step 1: Convert image to Lab
    cvtColor(inFrame, labImage, CV_BGR2Lab);
    vector<Mat> labPlanes(3);                    // Place to store extracted channels
    split(labImage, labPlanes);                    // Extract the L channel (actually all the channels)

    // Step 2: Apply CLAHE to L channel
    Mat imgGray;
    Ptr<CLAHE> clahe = cv::createCLAHE();
    clahe->setTilesGridSize(claheTileSize);
    clahe->setClipLimit(claheClipLimit);
    clahe->apply(labPlanes[0], imgGray);        // Apply CLAHE to the L channel

    // Step 3: Gaussian blur
    //    - This step might be unnecessary with good camera
    Mat imgGrayBlur;
    GaussianBlur(imgGray, imgGrayBlur, blurSize, blurSigma, blurSigma, BORDER_DEFAULT);

    // Step 4: Binary thresholding
    //    - After CLAHE, this should be consistent regardless of view scene
    threshold(imgGrayBlur, imgBinary, threshVal, 255.0, ADAPTIVE_THRESH_MEAN_C);

    // Step 5: Manipulate binary image
    //    - input image is imgBinary
    //  - morphology steps applied to imgMorph
    //  - results put back into imgBinary
    Mat imgMorph;
    Mat element = getStructuringElement(MORPH_RECT, morphologySize, morphologyPoint);
    erode(imgBinary, imgMorph, element, morphologyPoint, 1);
    dilate(imgMorph, imgMorph, element, morphologyPoint, 1);
    dilate(imgMorph, imgMorph, element, morphologyPoint, 1);
    erode(imgMorph, imgBinary, element, morphologyPoint, 1);

#ifdef DEBUGGING
    cvtColor(imgBinary, imgClrBinary, CV_GRAY2BGR);
#endif
}

// CleanContourGeometry()
//		Populate a vector with approxPolys of the raw contours
void PlacardView::CleanContourGeometry()
{
	// Store cleaned versions of the contours
	for (size_t i = 0; i < contourCount; i++) {
		if (rawContours[i].size() > 2) {
			vector<Point> approx;
			double preLength = arcLength(Mat(rawContours[i]), true);
			approxPolyDP(Mat(rawContours[i]), approx, preLength * 0.02, true);
			approxContours.push_back(approx); }
		else
			approxContours.push_back(rawContours[i]); }
}

void PlacardView::CalcContourDetails()
{
	for (size_t i = 0; i < contourCount; i++) {
		ContourInfo ci = ContourInfo(i, &approxContours, hier[i][3]);
		contourDetails.push_back(ci); }
}

void PlacardView::IdentifyParents()
{
	for (size_t i = 0; i < contourCount; i++) {

		if (contourDetails[i].IsChild())
			contourDetails[contourDetails[i].Get_ParentIndex()].SetAsParent();	}
}

void PlacardView::ContourProcessing()
{
    // NO NEED FOR CANNY
    //        The binary image is ready for contour processing

    // Step 1: Find contours in the image
    //		- Find contours destroys the input image so make a copy
    Mat imgFindContours;
    imgBinary.copyTo(imgFindContours);
    findContours(imgFindContours, rawContours, hier, RETR_TREE, CHAIN_APPROX_NONE);
    contourCount = rawContours.size();
    CleanContourGeometry();
    CalcContourDetails();
    IdentifyParents();
}

bool PlacardView::GoodChildToParentAreaRatio(size_t ixChild, size_t ixParent)
{
	double childArea  = contourDetails[ixChild].Get_area();
	double parentArea = contourDetails[ixParent].Get_area();
	double ratio      = childArea / parentArea;
	return (ratio < 0.1 || ratio > 0.5);
}

// TriangleDiscovery1()
//        - Have exactly 3 corners
//        - Have similar angles on all three corners
//        Expect each angle to be +/-20 degrees of 120 degrees (2.0944 rad)
bool PlacardView::TriangleDiscovery1(ContourInfo ci)
{
	if (!ci.IsConvex()) return false;
	if (ci.Get_size() != 3) return false;
	corner corn;
	corn.A = approxContours[ci.Get_index()][0];
	corn.B = approxContours[ci.Get_index()][1];
	corn.C = approxContours[ci.Get_index()][2];
	double distAB = Distance(corn.A.x, corn.A.y, corn.B.x, corn.B.y);
	double distAC = Distance(corn.A.x, corn.A.y, corn.C.x, corn.C.y);
	double distBC = Distance(corn.B.x, corn.B.y, corn.C.x, corn.C.y);
	double sqAB = distAB * distAB;
	double sqAC = distAC * distAC;
	double sqBC = distBC * distBC;
	double radABC = acos((sqBC - sqAB - sqAC) / (2 * distAB * distAC));
	double radBCA = acos((sqAC - sqBC - sqAB) / (2 * distBC * distAB));
	double radCAB = acos((sqAB - sqAC - sqBC) / (2 * distAC * distBC));

	// Is this a triangle?
	//        Expect each angle to be +/-20 degrees of 120 degrees (2.0944 rad)
	if (radABC > DEG100 && radABC < DEG140)
		if (radBCA > DEG100 && radBCA < DEG140)
			if (radCAB > DEG100 && radCAB < DEG140)
				return true;
	return false;
}

// TriangleDiscovery2()
//        Ratio of triangle to circle that encloses it
bool PlacardView::TriangleDiscovery2(ContourInfo ci)
{
	size_t size = ci.Get_size();
	if (size < 3 || size > 7) return false;
	double radius     = ci.Get_radius();
	double circleArea = CV_PI * (double) (radius * radius);
	double ratio      = ci.Get_area() / circleArea;
	return (ratio > 0.4 && ratio < 0.5);
}


// CirlceDiscoveryClock()
//		- At least 5 points needed (because minEncEllipse needs 5)
//		- Fit a min enclosing ellipse
//		- Draw a new elipse 75% of the size of the child
//		- Go around the ellipse and verify that each pixel value is black
//		- If they are all black, this is most likely a circle
bool PlacardView::CircleDiscoveryClock(ContourInfo ci)
{
	if (!ci.HasMoreThan4Points()) return false;
	RotatedRect rr = ci.Get_rotatedRect();
	double radiusX = 0.75 * rr.size.width / 2.0;
	double radiusY = 0.75 * rr.size.height / 2.0;
	vector<Point> ellipsePoints;
	Point p;
	short countZeros = 0;
	double calcX[] = {  0.00,  0.71,  1.00,  0.71,  0.00, -0.71, -1.00, -0.71 };
	double calcY[] = { -1.00, -0.71,  0.00,  0.71,  1.00,  0.71,  0.00, -0.71 };
	for (int j = 0; j < CLOCK_POINTS; j++) {
		p.x = ci.Get_center().x + (radiusX * calcX[j]);
		p.y = ci.Get_center().y + (radiusY * calcY[j]);
		ellipsePoints.push_back(p);
		countZeros += (imgBinary.at<uint8_t>(p.y, p.x) ? 0 : 1); }
	return (countZeros == CLOCK_POINTS);
}

// CircleComparison()
//        - Have greater than 5 corners
//        - Similar in size to its bounding ellipse
bool PlacardView::CircleComparison(ContourInfo ci)
{
	if (ci.Get_size() < 6) return false;
	RotatedRect rr = ci.Get_rotatedRect();
	double ellipseArea = CV_PI * (rr.size.height / 2.0) * (rr.size.width / 2.0);
	double ratio = ci.Get_area() / ellipseArea;
	return (ratio > 0.82);
}

bool PlacardView::CruciformComparison(ContourInfo ci)
{
	if (ci.Get_size() < 8) return false;
	size_t i = ci.Get_index();
	vector<int> hullIndices;
	convexHull(approxContours[i], hullIndices, false);
	vector<Vec4i> defects;
	convexityDefects(approxContours[i], hullIndices, defects);
	if (defects.size() == 4) {
		vector<Point> hullPoints;
		convexHull(approxContours[i], hullPoints, false);
		Rect boundRect = boundingRect(approxContours[i]);
		double ratio = ci.Get_area() / boundRect.area();
		return (ratio > .42 && ratio < .48); }
	return false;
}

void PlacardView::PlacardDiscovery()
{
	//ParentMask();

    // Examine each child
    for (size_t i = 0; i < contourCount; i++) {
    	ContourInfo* ci = &contourDetails[i];

    	// Only consider children
    	if (!ci->IsChild()) continue;

		// Triangle?
        if (TriangleDiscovery1(*ci) || TriangleDiscovery2(*ci)) {
        	ci->SetAsTriangle();
        	continue; }

        // Circle?
        if (CircleDiscoveryClock(*ci)) {
        	ci->SetAsCircle();
        	continue; }

        // Cross?
        //if (CruciformMatching())
       // 	continue;
        if (CruciformComparison(*ci)) {
        	ci->SetAsCross();
        	continue; } }
}

void PlacardView::ParentMask()
{
	return;
	imgBinaryROI = Mat::zeros(imgBinary.size(), CV_8UC1);
	for (size_t i = 0; i < contourCount; i++) {
    	ContourInfo ci = contourDetails[i];
    	if (!ci.IsParent() || ci.Get_size() < 5) continue;
		RotatedRect rr = ci.Get_rotatedRect();
		Scalar clr = Scalar (1);
		Point2f vertices2f[4];
		Point vertices[4];
		rr.points(vertices2f);
		for(int i = 0; i < 4; ++i)
			 vertices[i] = vertices2f[i];
		fillConvexPoly(imgBinaryROI, vertices, 4, clr); }

	// Multiply the binary image with the mask
	//		Zeros out all non-parent portions of the image
	for (int w = 0; w < imgBinary.size().width; w++) {
		for (int h = 0; h < imgBinary.size().height; h++) {
			int x = (int) imgBinaryROI.at<uchar>(h, w);
			int y = (int) imgBinaryROI.at<uchar>(h, w);
			imgBinaryROI.at<uchar>(h, w) = x * y; } }
}

void PlacardView::MatchOneSizedCross(ContourInfo ci, double& minVal, Point& minLoc, Point& otherCorner)
{
	return;
	Mat crossSized;
	Size crSize = Size(ci.Get_rotatedRect().size.width, ci.Get_rotatedRect().size.height);
	if (crSize.height < 6 || crSize.width < 6) return;
	resize(templateCross, crossSized, crSize, 0, 0, INTER_NEAREST);

	Mat matchResults;
	int resCols = imgBinary.cols - crossSized.cols + 1;
	int resRows = imgBinary.rows - crossSized.rows + 1;
	matchResults.create(resCols, resRows, CV_32FC1);
	matchTemplate(imgBinaryROI, crossSized, matchResults, CV_TM_SQDIFF_NORMED);
	Point maxLoc;
	double maxVal;
	minMaxLoc(matchResults, &minVal, &maxVal, &minLoc, &maxLoc, Mat());

	// Winner is the minimum score across the match results
	minMaxLoc(matchResults, &minVal, &maxVal, &minLoc, &maxLoc);
	otherCorner = Point(minLoc.x + resCols, minLoc.y + resRows);
}

// PlacardView()
// Iterate through the parents
//		Draw a masked area for each parent
//		Attempt to match a scaled template to the mask
bool PlacardView::CruciformMatching()
{
	return false;
	// Matching
	//		Resize template for each child
	double mostMin = 2.0;
	double minVal;
	int theCross = -1;
	Point matchLoc, matchCorner;
	Point minPoint, otherCorner;
	int matchType = CV_TM_SQDIFF_NORMED;
	bool bUseMin = (matchType  == CV_TM_SQDIFF || matchType == CV_TM_SQDIFF_NORMED);
	for (size_t i = 0; i < contourCount; i++) {
    	ContourInfo ci = contourDetails[i];
    	if (!ci.IsChild()) continue;
    	double minVal;
    	MatchOneSizedCross(ci, minVal, minPoint, otherCorner);
		if (minVal < mostMin) {
			theCross      = i;
			mostMin     = minVal;
			matchLoc    = minPoint;
			matchCorner = otherCorner; } }
	if (mostMin < 0.3 && theCross >= 0) {
		contourDetails[theCross].SetAsCross();
		return true; }
	return false;
}

void PlacardView::CreateTemplateCross()
{
	return;
	templateCross = Mat::zeros(160, 160, CV_8UC1);
	for (int i = 0; i < 60; i++) {
		for (int j = 0; j < 60; j++) {
			templateCross.at<uchar>(j, i) = 255;
			templateCross.at<uchar>(j + 100, i) = 255;
			templateCross.at<uchar>(j, i + 100) = 255;
			templateCross.at<uchar>(j + 100, i + 100) = 255; } }
}












//
