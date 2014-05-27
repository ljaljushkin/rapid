#pragma once

#include <list>

#include <opencv2/core/core.hpp>

class MeanShift3D
{
public:
    MeanShift3D(
        const size_t MaxIter,
        const double distanceThreshold,
        const cv::Point3f windowSizes,
        const bool isIterCriteria = true);

    void execute(
        const std::list<cv::Mat>* setPoints, 
        cv::Mat& foundCenter) const;

private:
    size_t maxIter;
    double  distanceThreshold;
    cv::Point3f windowSizes;
    bool isIterCriteria;
};