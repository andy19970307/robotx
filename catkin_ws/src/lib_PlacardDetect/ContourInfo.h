/*
 * ContourInfo.h
 *
 *  Created on: Oct 16, 2014
 *      Author: Alon Yaari
 */

#ifndef CONTOURINFO_H_
#define CONTOURINFO_H_

#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define AREA_THRESH 50

class ContourInfo {
public:
	ContourInfo();
	ContourInfo(size_t ix, std::vector<std::vector<cv::Point> >* fullList, int hier3);
	~ContourInfo() {};
	size_t					Get_index() { return index; };
	size_t					Get_size() { return size; };
	cv::RotatedRect			Get_rotatedRect() { return rr; };
	bool					IsConvex() { return isConvex; };
	bool					HasMoreThan4Points() { return moreThan4Points; };
	bool					HasMoreThan2Points() { return moreThan2Points; };
	double					Get_area() { return area; };
	cv::Point2f				Get_center() { return center; };
	float					Get_radius() { return radius; };
	bool					IsChild() { return (parentIx >= 0); };
	bool					IsParent() { return isAParent; };
	size_t					Get_ParentIndex() { return parentIx; };
	void					SetAsParent();
	void					SetAsChild() { isAChild = moreThan2Points; };
	bool					GetIsBigEnough() { return (area > AREA_THRESH); };
	void					SetAsTriangle() { isTriangle = true; };
	void					SetAsCircle() { isCircle = true; };
	void					SetAsCross() { isCross = true; };
	bool					IsCircle() { return isCircle; };
	bool					IsTriangle() { return isTriangle; };
	bool					IsCross() { return isCross; };


private:
	bool					OKasPlacard() { return (moreThan2Points && IsChild()); };
	size_t					index;
	size_t					size;
	cv::RotatedRect 	  	rr;
	bool					isConvex;
	bool					moreThan4Points;
	bool					moreThan2Points;
	int						parentIx;
	double					area;
	cv::Point2f				center;
	float					radius;
	bool					isAParent;
	bool					isAChild;
	bool					isTriangle;
	bool					isCircle;
	bool					isCross;
};




#endif








//
