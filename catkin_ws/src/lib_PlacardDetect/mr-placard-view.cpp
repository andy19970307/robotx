/*
 * nwPlacardView.cpp
 *
 *  Created on: Sep 14, 2014
 *      Author: Nick Wang
 */

#include "mr-placard-view.h"

using namespace std;
using namespace cv;
//using namespace AprilTags;

nwPlacardView::nwPlacardView()
{
	// for pre-processing
	claheTileSize   = Size(3, 3);
	claheClipLimit  = 4;
	blurSize        = Size(1, 1); // Blur size values must be ODD
	blurSigma       = 4;

	// MSER detection
	erBlackOnWhiteVar = 30;		//
	erMinStrokeWidth  = 1.5;	//

	// decoding
	minGoodMatch = 4;

}

int nwPlacardView::initTemplates(
		cv::Mat& img_circle, cv::Mat& img_triangle, cv::Mat& img_cross){

	this->im_circle = img_circle;
	this->im_triangle = img_triangle;
	this->im_cross = img_cross;

}

bool nwPlacardView::ProcessImage(Mat& img, placardFound& circle, placardFound& triangle, placardFound& cross)
{
	triangles.clear();
	circles.clear();
	cruciforms.clear();
	circle.valid   = false;
	triangle.valid = false;
	cross.valid    = false;
	inFrame = img;
	if (inFrame.empty()){
		return false;
	}
	cv::cvtColor(img, imgGray, CV_BGR2GRAY);
    GaussianBlur(imgGray, imgGray, blurSize, blurSigma, blurSigma, BORDER_DEFAULT);

	// detecting, find candidates using MSER
	std::vector<cv::Rect> er_placards;
	this->detect_placard_regions(img, er_placards);

	//cout << "There are " << er_placards.size() << " candidates" << endl;
	// testing for detection
	if(!this->im_circle.data || !this->im_triangle.data || !this->im_cross.data){

		for(int i = 0; i < er_placards.size(); i++){
			Rect r = er_placards[i];

			cv::Point c = cv::Point(r.x + 0.5*r.width, r.y + 0.5*r.height);
			int d = r.width;
			if(r.height > d) d = r.height;

			if (i == 0){
				circle.valid = true;
				circle.location.x = c.x;
				circle.location.y = c.y;
				circle.radius = d / 2;
			}

			if (i == 1){
				triangle.valid = true;
				triangle.location.x = c.x;
				triangle.location.y = c.y;
				triangle.radius = d / 2;
			}

			if (i == 2){
				cross.valid = true;
				cross.location.x = c.x;
				cross.location.y = c.y;
				cross.radius = d / 2;
			}
		}

		return true;
	}

	// decoding
	vector<int> gm, gm_sort;
	int conf_r_max = 0;
	int conf_c_max = 0;
	int conf_t_max = 0;

	for(int i = 0; i < er_placards.size(); i++){
		Rect r = er_placards[i];
		Mat im_region = imgGray(r).clone();

		// sort by good match number
		int conf_r = this->decode_cross(im_region);
		int conf_c = this->decode_circle(im_region);
		int conf_t = this->decode_triangle(im_region);

		gm.push_back(conf_r);
		gm.push_back(conf_c);
		gm.push_back(conf_t);

		gm_sort.push_back(conf_r);
		gm_sort.push_back(conf_c);
		gm_sort.push_back(conf_t);

		std::sort (gm_sort.begin(), gm_sort.end(), std::greater<int>());

		int p_id = 0;
		for(int i = 0; i < gm.size(); i++){
			if(gm[i] == gm_sort[0] & gm_sort[0] > 0){
				p_id = i + 1;
			}
		}
		//cout << "p_id " << p_id << endl;
		gm.clear();
		gm_sort.clear();

		cv::Point c = cv::Point(r.x + 0.5*r.width, r.y + 0.5*r.height);
		int d = r.width;
		if(r.height > d) d = r.height;

		if (p_id == 1 & conf_r > conf_r_max){
			cross.valid = true;
			cross.location.x = c.x;
			cross.location.y = c.y;
			cross.radius = d / 2;

			conf_r_max = conf_r;
		}
		if (p_id == 2 & conf_c > conf_c_max){
			circle.valid = true;
			circle.location.x = c.x;
			circle.location.y = c.y;
			circle.radius = d / 2;

			conf_c_max = conf_c;
		}

		if (p_id == 3 & conf_t > conf_t_max){
			triangle.valid = true;
			triangle.location.x = c.x;
			triangle.location.y = c.y;
			triangle.radius = d / 2;

			conf_t_max = conf_t;
		}

	}

	// decoding



//	circle.valid = (circles.size() > 0);
//	if (circle.valid){
//		//minEnclosingCircle(circles[0], circle.location, circle.radius);
//	}
//	triangle.valid = (triangles.size() > 0);
//	if (triangle.valid){
//		//minEnclosingCircle(triangles[0], triangle.location, triangle.radius);
//	}
//	cross.valid = (cruciforms.size() > 0);
//	if (cross.valid){
//		//minEnclosingCircle(cruciforms[0], cross.location, cross.radius);
//	}

	return true;
}

void nwPlacardView::get_gradient_maps(Mat& _grey_img,
		Mat& _gradient_magnitude, cv::Mat& _gradient_direction){

	cv::Mat C = cv::Mat_<double>(_grey_img);

	cv::Mat kernel = (cv::Mat_<double>(1,3) << -1,0,1);
	cv::Mat grad_x;
	filter2D(C, grad_x, -1, kernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);

	cv::Mat kernel2 = (cv::Mat_<double>(3,1) << -1,0,1);
	cv::Mat grad_y;
	filter2D(C, grad_y, -1, kernel2, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);

	for(int i=0; i<grad_x.rows; i++){
		for(int j=0; j<grad_x.cols; j++){
			_gradient_magnitude.at<double>(i,j) =
					sqrt(pow(grad_x.at<double>(i,j),2)+pow(grad_y.at<double>(i,j),2));
			_gradient_direction.at<double>(i,j) =
					atan2(grad_y.at<double>(i,j), grad_x.at<double>(i,j));

		}
	}
}

void nwPlacardView::visualize_regions(std::vector<Region> regions){
	cout << regions.size() << endl;

	Mat im_vis;
	cvtColor(this->imgGray, im_vis, CV_GRAY2RGB);
	for(int i = 0; i < regions.size(); i++){
		cv::rectangle(im_vis, regions[i].bbox_.tl(), regions[i].bbox_.br(), Scalar(0, 255, 0), 3);
	}

	cv::namedWindow("regions", WINDOW_AUTOSIZE);
	cv::imshow("regions", im_vis);
	cv::waitKey(0);
}


/////////////////////////////////////////////
// Extract keypoints
//     SIFT: good for junctions
//     FAST: good for thin-stroke text
/////////////////////////////////////////////

vector<KeyPoint> nwPlacardView::get_sift_corners(Mat im_gray, int nOctaveLayers){

	vector<KeyPoint> corners;

	int nfeatures=0;
	double contrastThreshold=0.04; // default: 0.04
	double edgeThreshold=50; // default: 10
	double sigma=0.8; // default: 1.6, svt: 0.8

	int pyramid_level = nOctaveLayers;

	int maxTotalKeypoints = 2000;
	int gridRows=1;
	int gridCols=1;

	cv::Ptr<SiftFeatureDetector> detector =
			new SiftFeatureDetector(nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);

	cv::Ptr<PyramidAdaptedFeatureDetector> pyramid_detector =
			new cv::PyramidAdaptedFeatureDetector(detector, pyramid_level);

	cv::Ptr<GridAdaptedFeatureDetector> grid_detector =
			new cv::GridAdaptedFeatureDetector(pyramid_detector,
					maxTotalKeypoints, gridRows, gridCols);

	grid_detector->detect(im_gray, corners);

	return corners;
}

vector<KeyPoint> nwPlacardView::get_fast_corners(Mat im_gray, int kp_num){

	vector<KeyPoint> corners;

	int threshold = 25;
	bool nonmaxSuppression = true;

	int pyramid_level = 1;

	int maxTotalKeypoints = kp_num;
	int gridRows=1;
	int gridCols=1;

	Ptr<FeatureDetector> detector(
			new cv::DynamicAdaptedFeatureDetector(
					new FastAdjuster(threshold, nonmaxSuppression), 10, 100, 10));

	cv::Ptr<PyramidAdaptedFeatureDetector> pyramid_detector =
			new cv::PyramidAdaptedFeatureDetector(detector, pyramid_level);

	cv::Ptr<GridAdaptedFeatureDetector> grid_detector =
			new cv::GridAdaptedFeatureDetector(pyramid_detector,
					maxTotalKeypoints, gridRows, gridCols);

	grid_detector->detect(im_gray, corners);

	return corners;
}

void nwPlacardView::detect_regions(cv::Mat img,
		vector<Region> & regions_black, vector<Region> & regions_white){

	Mat grey, lab_img, gradient_magnitude, gradient_direction;
	gradient_magnitude = Mat_<double>(img.size());
	gradient_direction = Mat_<double>(img.size());
	this->get_gradient_maps( grey, gradient_magnitude, gradient_direction);
	cvtColor(img, grey, CV_BGR2GRAY);
	cvtColor(img, lab_img, CV_BGR2Lab);

	// parameters:
	// bool eight: 			[false]
	// int delta			[25], smaller will get more regions
	// double minArea,		[0.000008]
	// double maxArea,		[0.03]
	// double maxVariation	[1], smaller will get more regions
	// double minDiversity	[0.7]
	vector<Region> regions;
	// for black regions,
	::MSER mser_black(false, 40, 0.001, 1.0, 1, 0.7);
	// for white regions,
	::MSER mser_white(false, 4, 0.001, 1.0, 1, 0.7);

	// step = 1: detect black
	// step = 2: detect white
	for (int step =1; step<2; step++)
	{

		if (step == 1){
			mser_black((uchar*)grey.data, grey.cols, grey.rows, regions);
		}
		if (step == 2){
			grey = 255-grey;
			mser_white((uchar*)grey.data, grey.cols, grey.rows, regions);
		}

		for (int i=0; i<regions.size(); i++)
			regions[i].er_fill(grey);

		double max_stroke = 0;
		for (int i=regions.size()-1; i>=0; i--)
		{
			regions[i].extract_features(lab_img, grey, gradient_magnitude);

			if ( (regions.at(i).stroke_std_/regions.at(i).stroke_mean_ > 0.8)
					|| (regions.at(i).num_holes_>2)
					|| (regions.at(i).bbox_.width <=3)
					|| (regions.at(i).bbox_.height <=3)
					)
				regions.erase(regions.begin()+i);
			else
				max_stroke = max(max_stroke, regions[i].stroke_mean_);
		}

		// filter
		for (int i=0; i<regions.size(); i++){
			Rect r = regions[i].bbox_;
			float asp_ratio = (float)r.width / (float)r.height;
			float area_ratio = (float)regions[i].area_ / (float)r.area();

			if(r.width < 10){
				continue;
			}
			if(r.height < 10){
				continue;
			}

			if(asp_ratio > 2){
				continue;
			}
			if(asp_ratio < 0.5){
				continue;
			}


			// stroke shouldn't be too small
	//		if(regions_all[i].stroke_mean_ < this->erMinStrokeWidth){
	//			continue;
	//		}
	//		cout << "intensity std: " << regions_in[i].intensity_std_ << endl;


			// black
			if(step == 1){
				if(area_ratio < 0.3){
					continue;
				}
				regions_black.push_back(regions[i]);
			}

			// white
			if(step == 2){
				if(area_ratio < 0.4){
					continue;
				}

				//cout << "area ratio: " << area_ratio << endl;
				regions_white.push_back(regions[i]);
			}
		}

		regions.clear();
	}

}

void nwPlacardView::filter_regions(
		std::vector<Region> regions_black, std::vector<Region> regions_white,
		std::vector<Region> & regions_out){

	for(int i = 0; i < regions_black.size(); i++){

		Rect r1 = regions_black[i].bbox_; // child (black)
		int is_placard = 0;

		for(int j = 0; j < regions_white.size(); j++){

			// check r1 is inside r2
			Rect r2 = regions_white[j].bbox_; // parent
			Rect r_i = r1 & r2;

			//			if (r1.area() != r_i.area()){
			//	continue;
			//}

			// check rize ratio, shouldn't be too small or large
			//			float height_ratio = (float)r1.height / (float)r2.height;
			//			if(height_ratio < 0.15 || height_ratio > 0.4){
			//				continue;
			//			}

			// check black location; center of white should be inside black
			//			cv::Point c_w = cv::Point(r2.x + 0.5*r2.width, r2.y + 0.5*r2.height);
			//			if(!c_w.inside(r1)){
			//				continue;
			//			}

			// intensity diff of child and parent should be high
//			if(abs(regions_black[i].intensity_mean_ -
//					regions_white[j].intensity_mean_) < this->erBlackOnWhiteVar){
//				continue;
//			}

//			cout << "top left: (" << r1.x << ", " << r1.y << ")" << endl;
//			cout << "r1 intensity mead: " <<regions_filtered[i].intensity_mean_ << endl;
//			cout << "r1 intensity std : " <<regions_filtered[i].intensity_std_ << endl;
//			cout << "r2 intensity mead: " <<regions_filtered[j].intensity_mean_ << endl;
//			cout << "r2 intensity std : " <<regions_filtered[j].intensity_std_ << endl;
//			cout << "perimeter_    : " <<regions_filtered[i].perimeter_ << endl;
//			cout << "stroke_mean_  : " <<regions_filtered[i].stroke_mean_ << endl;
//			cout << "pixel size    : " <<regions_filtered[i].pixels_.size() << endl;

			is_placard = 1;
			continue;
		}
		is_placard = 1;

		if(is_placard == 1){
			regions_out.push_back(regions_black[i]);
			//cout << endl;
		}

	}
}

int nwPlacardView::match_template(cv::Mat im_region, cv::Mat im_templ){

	// 1. extract keypoints
	vector<KeyPoint> kps1, kps2;
	// 1.1 detect FAST and SIFT in multi-scale
	kps1 = this->get_fast_corners(im_region);
	kps2 = this->get_fast_corners(im_templ);

	if(kps1.size() == 0 || kps2.size() == 0){
		return 0;
	}

	// 1.2 try different features
	// FAST, STAR, SIFT, SURF, ORB, BRISK, MSER, GFTT, HARRIS, Dense, SimpleBlob
	//	Ptr<FeatureDetector> detector = FeatureDetector::create("FAST");
	//	detector->detect(im_region, kps1);
	//	detector->detect(im_templ, kps2);
	// 1.3 default
	//	cv::FastFeatureDetector detector(15);
	//	cv::SurfFeatureDetector detector(400);
	//	detector.detect(im_region, kps1);
	//	detector.detect(im_templ, kps2);

	// 2. extract descriptor
	// SIFT, SURF, BRIEF, BRISK, ORB, FREAK
	Ptr<DescriptorExtractor> descriptorExtractor = DescriptorExtractor::create("SIFT");

	Ptr<DescriptorMatcher> descriptorMatcher = DescriptorMatcher::create("BruteForce");

	Mat descriptors1, descriptors2;
	descriptorExtractor->compute(im_region, kps1, descriptors1);
	descriptorExtractor->compute(im_templ, kps2, descriptors2);

	vector<DMatch> matches, good_matches;
	descriptorMatcher->match( descriptors1, descriptors2, matches);
	double max_dist = 0;
	double min_dist = 100;
	for(int i = 0; i < descriptors1.rows; i++){
		double dist = matches[i].distance;
		if(dist < min_dist) min_dist = dist;
		if(dist > max_dist) max_dist = dist;
	}
	for(int i = 0; i < descriptors1.rows; i++){
		if(matches[i].distance <= max(3*min_dist, 0.02)){
			good_matches.push_back(matches[i]);
		}
	}

//	cout << "good matches: " << good_matches.size() << endl;
	Mat im_vis;
	cv::drawMatches(im_region, kps1, im_templ, kps2, good_matches, im_vis);
//
//	cv::namedWindow("matches", 0);
//	cv::imshow("matches", im_vis);
//	cv::waitKey(0);

	return good_matches.size();
}

int nwPlacardView::decode_circle(cv::Mat im_region){

	int gm = this->match_template(this->im_circle, im_region);

	if(gm < this->minGoodMatch){
		return 0;
	}else{
		return gm;
	}
}

int nwPlacardView::decode_triangle(cv::Mat im_region){

	int gm = this->match_template(this->im_triangle, im_region);

	if(gm < this->minGoodMatch){
		return 0;
	}else{
		return gm;
	}
}

int nwPlacardView::decode_cross(cv::Mat im_region){

	int gm = this->match_template(this->im_cross, im_region);

	if(gm < this->minGoodMatch){
		return 0;
	}else{
		return gm;
	}

}

void nwPlacardView::detect_placard_regions(
		cv::Mat im, std::vector<cv::Rect> &regions){

	vector<Region> regions_all, regions_black, regions_white,
		regions_black_f, regions_white_f, regions_filtered;
	this->detect_regions(im, regions_black, regions_white);

	//	this->visualize_regions(regions_black);
	//	this->visualize_regions(regions_white);

	this->filter_regions(regions_black, regions_white, regions_filtered);

	//	this->visualize_regions(regions_filtered);

	for (int i=0; i<regions_filtered.size(); i++){
		Rect r = regions_filtered[i].bbox_;

		int pad_size = 3;

		Rect r1 = Rect(r.x - pad_size, r.y - pad_size,
				r.width + pad_size * 2, r.height + pad_size * 2)
				& Rect(0, 0, im.cols, im.rows);

		regions.push_back(r1);
	}
}
