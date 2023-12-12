
#ifndef CMP_UTILS_HPP
#define CMP_UTILS_HPP

#include "opencv2/core.hpp"

using namespace cv;

void comapreImgs(
	InputArray i1,
	InputArray i2,
	Scalar& numPixDiff,
	Scalar& sumPixDiff,
	Scalar& maxPixDiff
);

#endif // CMP_UTILS_HPP
