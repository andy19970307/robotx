/*
 * PlacardView.h
 *
 *  Created on: Sep 14, 2014
 *      Author: Nick Wang
 */

#ifndef NWPLACARDVIEW_H_
#define NWPLACARDVIEW_H_

#include <iostream>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"

// MSER
#include "region.h"
#include "mser.h"

// AprilTags includes
#include <algorithm>
#include <cmath>
#include <climits>

#include "PlacardView.h"

class nwPlacardView {

public:
	nwPlacardView();
	~nwPlacardView() {};
	int initTemplates(cv::Mat& img_circle, cv::Mat& img_triangle, cv::Mat& img_cross);
	bool ProcessImage(cv::Mat& img, placardFound& circle, placardFound& triangle, placardFound& cross);

private:

	// MSER
	// Detector based on MSER
	// placard_pts shoudl return 3 pts, 1) circle, 2) triangle, and 3) cross
	// no detection should return (0, 0)
	void detect_placard_regions(cv::Mat im, std::vector<cv::Rect> &regions);
	// matching using assigned feature detector and descriptor
	// return number of good match
	int match_template(cv::Mat im_region, cv::Mat im_templ);

	void detect_regions(cv::Mat im,
			std::vector<Region> & regions_black, std::vector<Region> & regions_white);
	void filter_regions(
			std::vector<Region> regions_black, std::vector<Region> regions_white,
			std::vector<Region> & regions_out);
	void get_gradient_maps(cv::Mat& _grey_img,
			cv::Mat& _gradient_magnitude, cv::Mat& _gradient_direction);
	void visualize_regions(std::vector<Region> regions);

	/////////////////////////////////////////////
	// Extract keypoints
	//     SIFT: good for junctions
	//     FAST: good for thin-stroke text and blurry text
	/////////////////////////////////////////////
	std::vector<cv::KeyPoint> get_sift_corners(cv::Mat im_gray, int nOctaveLayers = 4);
	std::vector<cv::KeyPoint> get_fast_corners(cv::Mat im_gray, int kp_num = 3000);

	// Decoding (circle, triangle, placard), return confidence
	//TODO: HOG, HOG 2x2
	//TODO: Histogram of Junction
	//TODO: AprilTags::Segments, how many turns

	int decode_circle		(cv::Mat im_region);
	int decode_triangle	(cv::Mat im_region);
	int decode_cross		(cv::Mat im_region);


	// AprilTags
//	// Step 1-2
//	void extractFloatImages(const cv::Mat& image,
//			std::vector<AprilTags::FloatImage> &float_images);
//	// Step 3-6
//	void extractSegments(
//			const cv::Mat& image, const cv::Mat& im_mask,
//			std::vector<AprilTags::FloatImage> float_images,
//			std::vector<AprilTags::Segment> & segments);
//	// Step 7 twist
//	void extractQuads(
//			const cv::Mat& image, std::vector<AprilTags::Segment> segments,
//			std::vector<AprilTags::Quad> & quads);

	// Templates
	cv::Mat im_circle;
	cv::Mat im_triangle;
	cv::Mat im_cross;

	// CV images between major processing steps
	cv::Mat inFrame;					// Color input image to process
	cv::Mat labImage;					// Image converted to Lab color format
	cv::Mat imgGray;
	cv::Mat imgBinary;					// Binary image as result of preprocessing

	// Vectors for managing contours
    std::vector<std::vector<cv::Point> >   triangles;       // Polys identified as triangles
    std::vector<std::vector<cv::Point> >   circles;			// Polys identified as circles
    std::vector<std::vector<cv::Point> >   cruciforms;      // Polys identified as cruciforms

	// Adjustable parameters
	cv::Size	claheTileSize;			// CLAHE tile grid size
	int			claheClipLimit;			// CLAHE clip limit
	cv::Size	blurSize;				// Gaussian blur tile size
	int			blurSigma;				// Sigma x and y value for gausian blur
	int			erBlackOnWhiteVar;		// intensity variation of black symbol on white.
	float		erMinStrokeWidth;		//

	int 		minGoodMatch;
};

#endif
