
#include "ColorObjDetector.hpp"

#include "PrintUtils.hpp"

#include "DebugImgUtils.hpp"
#include "TrackballHelper.hpp"
#include "MyImplOpenCV.hpp"
#include "CmpUtils.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/features2d.hpp>
#include "blob_detector.hpp"

#define DEBUG_IMG1(m) DEBUG_IMG(m)
#define DEBUG_IMG2(m) //DEBUG_IMG(m)
#define CALIB_HSV 1

#define FLIP 1

#define BLOB_MIN_AREA 20000
#define BLOB_MAX_AREA 2000000

const int max_value_H = 180;
const int max_value = 255;

// Threadsholded region color for S and V.
const Scalar cyclamen(161, 111, 245);

ColorObjDetector::ColorObjDetector(
	const string& cfg_fn,
	const string& img
)
{
	if(img == ""){
		fp = new CameraFrameProducer();
	}else{
		fp = new ImageFrameProducer(img);
	}
	
	s.read(cfg_fn);
	GET_DEFAULT_SETTING(s, start_H[0], 0);
	GET_DEFAULT_SETTING(s, stop_H[0], 0);
	GET_DEFAULT_SETTING(s, start_H[1], 0);
	GET_DEFAULT_SETTING(s, stop_H[1], 0);
	GET_DEFAULT_SETTING(s, low_S, 0);
	GET_DEFAULT_SETTING(s, high_S, max_value);
	GET_DEFAULT_SETTING(s, low_V, 0);
	GET_DEFAULT_SETTING(s, high_V, max_value);
	
	GET_DEFAULT_SETTING(s, blob_minArea, 1200);
	GET_DEFAULT_SETTING(s, blob_maxArea, 500000);
	GET_DEFAULT_SETTING(s, blob_minCircularity, 40);
	GET_DEFAULT_SETTING(s, blob_minConvexity, 50);
	
	namedWindow("Color Object Detection");

	TRACKBALL_WITH_CALLBACK(
		start_H[0],
		max_value_H,
		[&](int value){
			start_H[0] = min(stop_H[0], start_H[0]);
			UPDATE_TRACKBALL(start_H[0]);
			SET_AND_FLUSH_SETTING(s, start_H[0]);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		stop_H[0],
		max_value_H,
		[&](int value){
			stop_H[0] = max(stop_H[0], start_H[0]);
			UPDATE_TRACKBALL(stop_H[0]);
			SET_AND_FLUSH_SETTING(s, stop_H[0]);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		start_H[1],
		max_value_H,
		[&](int value){
			start_H[1] = min(stop_H[1], start_H[1]);
			UPDATE_TRACKBALL(start_H[1]);
			SET_AND_FLUSH_SETTING(s, start_H[1]);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		stop_H[1],
		max_value_H,
		[&](int value){
			stop_H[1] = max(stop_H[1], start_H[1]);
			UPDATE_TRACKBALL(stop_H[1]);
			SET_AND_FLUSH_SETTING(s, stop_H[1]);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		low_S,
		max_value,
		[&](int value){
			low_S = min(high_S, low_S);
			UPDATE_TRACKBALL(low_S);
			SET_AND_FLUSH_SETTING(s, low_S);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		high_S,
		max_value,
		[&](int value){
			high_S = max(high_S, low_S);
			UPDATE_TRACKBALL(high_S);
			SET_AND_FLUSH_SETTING(s, high_S);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		low_V,
		max_value,
		[&](int value){
			low_V = min(high_V, low_V);
			UPDATE_TRACKBALL(low_V);
			SET_AND_FLUSH_SETTING(s, low_V);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		high_V,
		max_value,
		[&](int value){
			high_V = max(high_V, low_V);
			UPDATE_TRACKBALL(high_V);
			SET_AND_FLUSH_SETTING(s, high_V);
			settings_changed();
		}
	);
	
#if 1
	TRACKBALL_WITH_CALLBACK(
		blob_minArea,
		BLOB_MIN_AREA,
		[&](int value){
			SET_AND_FLUSH_SETTING(s, blob_minArea);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		blob_maxArea,
		BLOB_MAX_AREA,
		[&](int value){
			SET_AND_FLUSH_SETTING(s, blob_maxArea);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		blob_minCircularity,
		100,
		[&](int value){
			SET_AND_FLUSH_SETTING(s, blob_minCircularity);
			settings_changed();
		}
	);
	TRACKBALL_WITH_CALLBACK(
		blob_minConvexity,
		100,
		[&](int value){
			SET_AND_FLUSH_SETTING(s, blob_minConvexity);
			settings_changed();
		}
	);
#endif
	contour_idx_percent_start = 0;
	TRACKBALL_WITH_CALLBACK(
		contour_idx_percent_start,
		100,
		[&](int value){
			SET_AND_FLUSH_SETTING(s, contour_idx_percent_start);
			settings_changed();
		}
	);
	contour_idx_percent_stop = 100;
	TRACKBALL_WITH_CALLBACK(
		contour_idx_percent_stop,
		100,
		[&](int value){
			SET_AND_FLUSH_SETTING(s, contour_idx_percent_stop);
			settings_changed();
		}
	);
}

void ColorObjDetector::process_image() {

	//cap >> src;
	src_orig = fp->getFrame();
	//DEBUG(mat_stats(src_orig));
	
#if 1
	// For easier visualization and to be more LPRS2_MAX1000 camera like.
	//static const Size prefered_size = Size(640, 480);
	// Little larger.
	//static const Size prefered_size = Size(800, 600);
	static const Size prefered_size = Size(1280, 960);
#if 0
	DEBUG(src_orig.rows);
	DEBUG(src_orig.cols);
	DEBUG(prefered_size.width);
	DEBUG(prefered_size.height);
#endif
	if(
		src_orig.rows > prefered_size.width ||
		src_orig.cols > prefered_size.height
	) {
		float ratio = min(
			float(prefered_size.width)/src_orig.rows,
			float(prefered_size.height)/src_orig.cols
		);
		resize(src_orig, src_resized, Size(), ratio, ratio);
	}else{
		src_resized = src_orig;
	}
#else
	src_resized = src_orig;
#endif

#if FLIP
	// Rotate 180 degrees.
	flip(src_resized, src, -1);
#else
	src = src_resized;
#endif

	DEBUG_IMG2(src);
	//DEBUG_IMG_CHANNELS(src, "BGR");
	
	cvtColor(src, hsv, COLOR_BGR2HSV);

	Mat tmp_threshold[2];
	inRange(
		hsv,
		Scalar(start_H[0], low_S, low_V),
		Scalar(stop_H[0], high_S, high_V),
		tmp_threshold[0]
	);
	inRange(
		hsv,
		Scalar(start_H[1], low_S, low_V),
		Scalar(stop_H[1], high_S, high_V),
		tmp_threshold[1]
	);
	//bitwise_or(tmp_threshold[0], tmp_threshold[1], threshold);
	threshold = tmp_threshold[0] | tmp_threshold[1];
	
	DEBUG_IMG2(threshold);
	
	
#if CALIB_HSV
	//DEBUG_IMG(src);
	DEBUG_IMG(threshold);
	//DEBUG_IMG_BGR(src);
	DEBUG_IMG_HSV(hsv);
#if 1
	vector<Mat> splited_hsv;
	split(hsv, splited_hsv);
	Mat h = splited_hsv[0];
	Mat s = splited_hsv[1];
	Mat v = splited_hsv[2];
	Mat lt_h;
	Mat lt_s;
	Mat lt_v;
	inRange(
		h,
		start_H[0],
		stop_H[0],
		tmp_threshold[0]
	);
	inRange(
		h,
		start_H[1],
		stop_H[1],
		tmp_threshold[1]
	);
	bitwise_or(tmp_threshold[0], tmp_threshold[1], lt_h);
	inRange(
		s,
		low_S,
		high_S,
		lt_s
	);
	inRange(
		v,
		low_V,
		high_V,
		lt_v
	);
	
	
	
	Mat l_h_hsv;
	Mat l_h;
	Mat ones(h.size(), CV_8UC1, Scalar::all(255));
	// Black is mask for H.
	Mat h_m = h & lt_h;
	Mat ones_m = ones & lt_h;
	merge(vector<Mat>({h_m, ones_m, ones_m}), l_h_hsv);
	cvtColor(l_h_hsv, l_h, COLOR_HSV2BGR);
	DEBUG_IMG(l_h);
	
	// Cyclamen is mask for S and V.
	Mat l_s;
	merge(
		vector<Mat>({
			(s & lt_s) | (~lt_s & cyclamen[0]),
			(s & lt_s) | (~lt_s & cyclamen[1]),
			(s & lt_s) | (~lt_s & cyclamen[2])
		}),
		l_s
	);
	DEBUG_IMG(l_s);
	
	Mat l_v;
	merge(
		vector<Mat>({
			(v & lt_v) | (~lt_v & cyclamen[0]),
			(v & lt_v) | (~lt_v & cyclamen[1]),
			(v & lt_v) | (~lt_v & cyclamen[2])
		}),
		l_v
	);
	DEBUG_IMG(l_v);
	
	// Blue where is h passing...
	// White is all 3 passing...
	// Black is non passing...
	Mat l_th_overlap;
	merge(
		vector<Mat>({
			lt_h,
			lt_s,
			lt_v
		}),
		l_th_overlap
	);
	DEBUG_IMG(l_th_overlap);
	
#endif
	Mat threshold3;
	merge(vector<Mat>({threshold, threshold, threshold}), threshold3);
	Mat thresholded_src;
	bitwise_and(threshold3, src, thresholded_src);
	Mat thresholded_hsv;
	//DEBUG_IMG(thresholded_src);
	cvtColor(thresholded_src, thresholded_hsv, COLOR_BGR2HSV);
	DEBUG_IMG_HSV(thresholded_hsv);
#endif

#if 0
	Mat hsv2;
	my_cv::cvtColor(src, hsv2, COLOR_BGR2HSV);
	DEBUG_IMG_HSV(hsv);
	DEBUG_IMG_HSV(hsv2);
	Scalar hsv_numPixDiff;
	Scalar hsv_sumPixDiff;
	Scalar hsv_maxPixDiff;
	comapreImgs(
		hsv,
		hsv2,
		hsv_numPixDiff,
		hsv_sumPixDiff,
		hsv_maxPixDiff
	);
	DEBUG(hsv_numPixDiff);
	DEBUG(hsv_sumPixDiff);
	DEBUG(hsv_maxPixDiff); //TODO Debug out, by maxLoc, that 179.
#endif
	
	threshold_with_keypoints = threshold;
	src_with_keypoints = src;
}

void ColorObjDetector::draw_marker(
	Point p,
	int id
) {
	static const Scalar colors[6] = {
		Scalar(255, 0, 0), // Blue.
		Scalar(0, 255, 0), // Green.
		Scalar(0, 0, 255), // Red.
		Scalar(255, 255, 0), // Cyan.
		Scalar(255, 0, 255), // Magenta.
		Scalar(0, 255, 255) // Yellow.
	};
	static const MarkerTypes markers[7] = {
		MARKER_CROSS,
		MARKER_TILTED_CROSS,
		MARKER_STAR,
		MARKER_DIAMOND,
		MARKER_SQUARE,
		MARKER_TRIANGLE_UP,
		MARKER_TRIANGLE_DOWN
	};
	MarkerTypes marker = markers[id % 7]; 
	Scalar color = colors[id % 6];
	
	//TODO Draw on some others images.
	drawMarker(
		src_with_keypoints,
		p,
		color,
		marker,
		30,
		3
	);
}

void ColorObjDetector::blob_detect() {
	
	
	// Just invert threshold.
	//TODO Use inv.
	//cv::threshold(threshold, inv_threshold, 100, 255, THRESH_BINARY_INV);
	bitwise_not(threshold, inv_threshold);
	erode(inv_threshold, eroded, Mat(), Point(-1, -1), 4);
	//DEBUG_IMG2(eroded);
	dilate(eroded, dilated, Mat(), Point(-1, -1), 4);
	//DEBUG_IMG2(dilated);
	medianBlur(dilated, blured, 5);
	//DEBUG_IMG2(blured);
	
	bitwise_not(blured, inv_blured);
	//DEBUG_IMG2(inv);
	for_detection_with_keypoints = blured;
	
	//Mat for_detection = blured;
	//Mat for_detection = inv_threshold; // Skip bluring.
	Mat for_detection = eroded;
	
	
	// For partially seen cherries: set white around.
	for(int y = 0; y < for_detection.rows; y++){
		for_detection.at<uchar>(Point(                   0, y)) = 255;
		for_detection.at<uchar>(Point(for_detection.cols-1, y)) = 255;
	}
	for(int x = 0; x < for_detection.cols; x++){
		for_detection.at<uchar>(Point(x,                    0)) = 255;
		for_detection.at<uchar>(Point(x, for_detection.rows-1)) = 255;
	}
	
	/*
	//TODO Full RGB and HSV histogram.
	DEBUG(mat_stats(blured));
	Mat hist;
	float range[] = { 0, 256 }; //the upper boundary is exclusive
	const float* histRange[] = { range };
	int histSize = 256;
	bool uniform = true, accumulate = false;
	calcHist(
		&blured, 1, 0, Mat(),
		hist, 1, &histSize,
		histRange,
		uniform, accumulate
	);
	DEBUG(hist);
	*/
	
	
	SimpleBlobDetector::Params params;
	
	// Image is already binarized. But need at least 2 images.
	params.minThreshold = 100;
	params.thresholdStep = 10;
	params.maxThreshold = 120;
	
	// Sometimes there is hole on center where algo sample color.
	params.filterByColor = true;
	
	params.filterByArea 
		= blob_minArea != BLOB_MIN_AREA && blob_maxArea != BLOB_MAX_AREA;
	params.minArea = blob_minArea;
	params.maxArea = blob_maxArea;
	
	params.filterByCircularity = blob_minCircularity != 0;
	params.minCircularity = float(blob_minCircularity)/100;
	
	params.filterByConvexity = blob_minConvexity != 0;
	params.minConvexity = float(blob_minConvexity)/100;
	
	params.filterByInertia = false;
	
	params.minDistBetweenBlobs = 0;
	//params.minRepeatability = 1;
	
#if 0
	auto detector = cv::SimpleBlobDetector::create(params);
	// Blob detector.
	detector->detect(for_detection, blob_keypoints);
#else
	BlobDetectorParams2 params2;
	params2.contour_idx_percent_start = contour_idx_percent_start;
	params2.contour_idx_percent_stop = contour_idx_percent_stop;
	detect_blobs(for_detection, blob_keypoints, params, params2);
#endif

	drawKeypoints(
		for_detection_with_keypoints,
		blob_keypoints,
		for_detection_with_keypoints,
		Scalar(255, 0, 0), // Blue.
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);
	drawKeypoints(
		src_with_keypoints,
		blob_keypoints,
		src_with_keypoints,
		Scalar(255, 0, 0), // Blue.
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);
	
	
#if 0
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(
		inv_blured,
		contours,
		hierarchy,
		RETR_CCOMP,
		CHAIN_APPROX_SIMPLE
	);
	
	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0;
	for(; idx >= 0; idx = hierarchy[idx][0]){
		//Scalar color(0, 255, 255); // Yellow.
		Scalar color( rand()&255, rand()&255, rand()&255 );
		drawContours(
			src_with_keypoints,
			contours,
			idx,
			color,
			FILLED,
			8,
			hierarchy
		);
	}
#endif
}

void ColorObjDetector::show_results() {
	DEBUG_IMG1(for_detection_with_keypoints);
	DEBUG_IMG1(src_with_keypoints);
}

bool ColorObjDetector::next_frame() {
	return fp->next_frame();
}

void ColorObjDetector::mean_detect() {

	// Zero
	//for_mean = inv_blured;
	for_mean = threshold;
	for_mean.at<uchar>(0, 0) = 1; // At least one different from 0.
	findNonZero(for_mean, loc);
	Mat m, s;
	meanStdDev(loc, m, s);
	mean_keypoints.clear();
	int non_zeros = loc.rows;
	if(non_zeros > 1000){
		mean_keypoints.push_back(
			KeyPoint(
				m.at<double>(0),
				m.at<double>(1),
				mean(s)[0]*2
			)
		);
	}
	
	drawKeypoints(
		threshold_with_keypoints,
		mean_keypoints,
		threshold_with_keypoints,
		Scalar(255, 255, 0), // Cyan.
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);
	DEBUG_IMG1(threshold_with_keypoints);
	drawKeypoints(
		src_with_keypoints,
		mean_keypoints,
		src_with_keypoints,
		Scalar(255, 255, 0), // Cyan.
		DrawMatchesFlags::DRAW_RICH_KEYPOINTS
	);
	DEBUG_IMG1(src_with_keypoints);
#if 1
	for(int i = 0; i < mean_keypoints.size(); i++){
		cout << "mean_keypoints[" << i << "]: "
			<< mean_keypoints[i].pt.x << "\t"
			<< mean_keypoints[i].pt.y << endl;
	}
#endif
}
