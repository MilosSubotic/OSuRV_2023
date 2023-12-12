
#include "blob_detector.hpp"

#include <opencv2/imgproc.hpp>

#define DEBUG_BLOB_DETECTOR 1

#if DEBUG_BLOB_DETECTOR
#endif
#include <opencv2/highgui.hpp>
#include "PrintUtils.hpp"


using namespace cv;

struct CV_EXPORTS Center
{
	Point2d location;
	double radius;
	double confidence;
};

static Point2f swap(Point2f p) {
	return Point2f(p.y, p.x);
}

static void find_blobs(
	const cv::SimpleBlobDetector::Params& params,
	const BlobDetectorParams2& params2,
	InputArray _binaryImage,
	std::vector<Center> &centers
) {
	Mat binaryImage = _binaryImage.getMat();
	centers.clear();

	std::vector< std::vector<Point> >contours;
	findContours(binaryImage, contours, RETR_LIST, CHAIN_APPROX_NONE);

	size_t N_contours = contours.size();
#if DEBUG_BLOB_DETECTOR
	Mat keypointsImage;
	cvtColor(binaryImage, keypointsImage, COLOR_GRAY2RGB);
	Mat contoursImage;
	cvtColor(binaryImage, contoursImage, COLOR_GRAY2RGB);
	size_t ci_start = size_t(floor(0.01*params2.contour_idx_percent_start*N_contours));
	size_t ci_stop = size_t(ceil(0.01*params2.contour_idx_percent_stop*N_contours));
	bool print_it = params2.contour_idx_percent_start == params2.contour_idx_percent_stop-1;
	// cherry_4_imgs/2022-06-11-002622.jpg
	//ci_start = 107; ci_stop = ci_start+1; // Black cherry
	//ci_start = 103; ci_stop = ci_start+1; // Specular on cherry.
	if(print_it){
		DEBUG(ci_start);
		DEBUG(ci_stop);
	}
	for(
		size_t contourIdx = ci_start;
		contourIdx < std::min(ci_stop, N_contours);
		contourIdx++
	){
		drawContours(contoursImage, contours, contourIdx, Scalar(0,255,0));
	}
	imshow("contours", contoursImage );
#endif

	for(size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++){
		std::vector<Point>& contour = contours[contourIdx];
#if DEBUG_BLOB_DETECTOR && 0
		DEBUG(contourIdx);
		cvtColor(binaryImage, contoursImage, COLOR_GRAY2RGB);
		drawContours(contoursImage, contours, contourIdx, Scalar(0,255,0));
		imshow("contour by contour", contoursImage );
		waitKey();
#endif

#define IF_IDX (ci_start <= contourIdx && contourIdx < ci_stop)
#if DEBUG_BLOB_DETECTOR && 0
#define CONTINUE if(IF_IDX){ cout << "filtered@" << __LINE__ << endl;} continue
#else
#define CONTINUE continue
#endif


		Center center;
		center.confidence = 1;
		Moments moms = moments(contour);
		if (params.filterByArea)
		{
			double area = moms.m00;
			if (area < params.minArea || area >= params.maxArea){
				CONTINUE;
			}
		}

		if (params.filterByCircularity)
		{
			double area = moms.m00;
			double perimeter = arcLength(contour, true);
			double ratio = 4 * CV_PI * area / (perimeter * perimeter);
			if (ratio < params.minCircularity || ratio >= params.maxCircularity){
				CONTINUE;
			}
		}

		if (params.filterByInertia)
		{
			double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
			const double eps = 1e-2;
			double ratio;
			if (denominator > eps)
			{
				double cosmin = (moms.mu20 - moms.mu02) / denominator;
				double sinmin = 2 * moms.mu11 / denominator;
				double cosmax = -cosmin;
				double sinmax = -sinmin;

				double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
				double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
				ratio = imin / imax;
			}
			else
			{
				ratio = 1;
			}

			if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio){
				CONTINUE;
			}

			center.confidence = ratio * ratio;
		}

		if (params.filterByConvexity){
			std::vector < Point > hull;
			convexHull(contour, hull);
			double area = contourArea(contour);
			double hullArea = contourArea(hull);
			if(fabs(hullArea) < DBL_EPSILON){
				CONTINUE;
			}
			double ratio = area / hullArea;
			if(ratio < params.minConvexity || ratio >= params.maxConvexity){
				CONTINUE;
			}
		}

		if(moms.m00 == 0.0){
			CONTINUE;
		}
		center.location = Point2d(moms.m10 / moms.m00, moms.m01 / moms.m00);

		if(params.filterByColor){
#if 0
			// In orig algo.
			uchar pix_inside = binaryImage.at<uchar>(
				cvRound(center.location.y),
				cvRound(center.location.x)
			);
			if(pix_inside != params.blobColor){
				CONTINUE;
			}
#else
			bool ok = false; 
			for(int i = 0; i < contour.size()-2; i++){
				if(contour.size() < i+2){
					break;
				}
				Point2f a = contour[i];
				Point2f b = contour[i+1];
				Point2f m = (a+b)/2;
				Point2f am = a-m;
				Point2f bm = b-m;
				Point2f c = m + Point2f(am.y, bm.x);
				Point2f d = m + Point2f(bm.y, am.x);
#if 0
				if(IF_IDX){
					cout << endl;
					DEBUG(i);
					DEBUG(a);
					DEBUG(b);
					DEBUG(m);
					DEBUG(am);
					DEBUG(bm);
					DEBUG(c);
					DEBUG(d);
					DEBUG(pointPolygonTest(contour, c, false));
					DEBUG(pointPolygonTest(contour, d, false));
					DEBUG((c == a));
					DEBUG((c == b));
					DEBUG((d == a));
					DEBUG((d == b));
					DEBUG(ok);
				}
#endif
				if(pointPolygonTest(contour, c, false) > 0){
					uchar pix_inside_c = binaryImage.at<uchar>(
						cvRound(c.y),
						cvRound(c.x)
					);
					if(pix_inside_c == params.blobColor){
						ok = true;
						break;
					}
				}
				if(pointPolygonTest(contour, d, false) > 0){
					uchar pix_inside_d = binaryImage.at<uchar>(
						cvRound(d.y),
						cvRound(d.x)
					);
					if(pix_inside_d == params.blobColor){
						ok = true;
						break;
					}
				}
			}
			if(!ok){
				CONTINUE;
			}
#endif
		}

		//compute blob radius
		{
			std::vector<double> dists;
			for (size_t pointIdx = 0; pointIdx < contour.size(); pointIdx++){
				Point2d pt = contour[pointIdx];
				dists.push_back(norm(center.location - pt));
			}
			std::sort(dists.begin(), dists.end());
			center.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
		}

		centers.push_back(center);


#if DEBUG_BLOB_DETECTOR
		circle(keypointsImage, center.location, 5, Scalar(0,0,255), 1);
		drawContours(keypointsImage, contours, contourIdx, Scalar(0,255,0), -1);
#endif
	}
#if DEBUG_BLOB_DETECTOR
	imshow("bk", keypointsImage);
	//waitKey();
#endif
}

void detect_blobs(
	cv::InputArray image,
	std::vector<cv::KeyPoint>& keypoints,
	const cv::SimpleBlobDetector::Params& params,
	const BlobDetectorParams2& params2
) {
	keypoints.clear();
	CV_Assert(params.minRepeatability != 0);
	Mat grayscaleImage;
	if (image.channels() == 3 || image.channels() == 4)
		cvtColor(image, grayscaleImage, COLOR_BGR2GRAY);
	else
		grayscaleImage = image.getMat();

	if (grayscaleImage.type() != CV_8UC1) {
		CV_Error(Error::StsUnsupportedFormat, "Blob detector only supports 8-bit images!");
	}

	std::vector < std::vector<Center> > centers;
	for (double thresh = params.minThreshold; thresh < params.maxThreshold; thresh += params.thresholdStep)
	{
		Mat binarizedImage;
		threshold(grayscaleImage, binarizedImage, thresh, 255, THRESH_BINARY);

		std::vector < Center > curCenters;
		find_blobs(params, params2, binarizedImage, curCenters);
		std::vector < std::vector<Center> > newCenters;
		for (size_t i = 0; i < curCenters.size(); i++)
		{
			bool isNew = true;
			for (size_t j = 0; j < centers.size(); j++)
			{
				double dist = norm(centers[j][ centers[j].size() / 2 ].location - curCenters[i].location);
				isNew = dist >= params.minDistBetweenBlobs && dist >= centers[j][ centers[j].size() / 2 ].radius && dist >= curCenters[i].radius;
				if (!isNew)
				{
					centers[j].push_back(curCenters[i]);

					size_t k = centers[j].size() - 1;
					while( k > 0 && curCenters[i].radius < centers[j][k-1].radius )
					{
						centers[j][k] = centers[j][k-1];
						k--;
					}
					centers[j][k] = curCenters[i];

					break;
				}
			}
			if (isNew)
				newCenters.push_back(std::vector<Center> (1, curCenters[i]));
		}
		std::copy(newCenters.begin(), newCenters.end(), std::back_inserter(centers));
	}

	for (size_t i = 0; i < centers.size(); i++)
	{
		if (centers[i].size() < params.minRepeatability){
			continue;
		}
		Point2d sumPoint(0, 0);
		double normalizer = 0;
		for (size_t j = 0; j < centers[i].size(); j++)
		{
			sumPoint += centers[i][j].confidence * centers[i][j].location;
			normalizer += centers[i][j].confidence;
		}
		sumPoint *= (1. / normalizer);
		KeyPoint kpt(sumPoint, (float)(centers[i][centers[i].size() / 2].radius) * 2.0f);
		keypoints.push_back(kpt);
	}

}
