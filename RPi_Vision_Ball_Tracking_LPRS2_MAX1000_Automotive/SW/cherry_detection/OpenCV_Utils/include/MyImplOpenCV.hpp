
#ifndef MY_IMPL_OPEN_CV_HPP
#define MY_IMPL_OPEN_CV_HPP

#include <opencv2/imgproc.hpp>

using namespace cv;

namespace my_cv {
	void cvtColor(
		InputArray src,
		OutputArray dst,
		int code,
		int dstCn = 0 
	);
};

#endif // MY_IMPL_OPEN_CV_HPP
