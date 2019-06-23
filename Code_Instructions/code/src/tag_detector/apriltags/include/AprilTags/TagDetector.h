#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "AprilTags/TagDetection.h"
#include "AprilTags/TagFamily.h"
#include "AprilTags/FloatImage.h"
#include "AprilTags/Edge.h"

namespace AprilTags {

class TagDetector {
public:
	
	const TagFamily thisTagFamily;

	//! Constructor
  // note: TagFamily is instantiated here from TagCodes
	TagDetector(const TagCodes& tagCodes) : thisTagFamily(tagCodes) {}

	std::vector<Edge> edges;
        std::vector<float> storage;
        float * tmin;
        float * tmax;
        float * mmin;
        float * mmax;

    	void init(int width, int height);
	std::vector<TagDetection> extractTags(const cv::Mat& image);
};

} // namespace

#endif
