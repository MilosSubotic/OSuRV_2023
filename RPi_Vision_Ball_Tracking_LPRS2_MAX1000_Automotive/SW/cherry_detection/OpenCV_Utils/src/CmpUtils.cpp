
#include "CmpUtils.hpp"
#include "DebugImgUtils.hpp"
#include <stdexcept>
using namespace std;


static double maxElement(InputArray m) {
	double minVal;
	double maxVal;
	Point minIdx;
	Point maxIdx;
	minMaxLoc(m, &minVal, &maxVal, &minIdx, &maxIdx);
	return maxVal;
}

void comapreImgs(
	InputArray i1,
	InputArray i2,
	Scalar& numPixDiff,
	Scalar& sumPixDiff,
	Scalar& maxPixDiff
) {
	Mat m1 = i1.getMat();
	Mat m2 = i2.getMat();
	if(m1.type() != CV_8UC3 && m2.type() != CV_8UC3){
		throw logic_error("Not implemented!");
	}
	Mat s1;
	Mat s2;
	m1.convertTo(s1, CV_16SC3);
	m2.convertTo(s2, CV_16SC3);
	//norm(m1 == m2, NORM_L1);
	Mat diff = abs(s1 - s2);
	Mat spl[3];
	split(diff, spl);
	DEBUG_IMG(spl[0]);
	
	numPixDiff = Scalar(
		countNonZero(spl[0]),
		countNonZero(spl[1]),
		countNonZero(spl[2])
	);
	sumPixDiff = Scalar(
		sum(spl[0])[0],
		sum(spl[1])[0],
		sum(spl[2])[0]
	);
	maxPixDiff = Scalar(
		maxElement(spl[0]),
		maxElement(spl[1]),
		maxElement(spl[2])
	);
}
