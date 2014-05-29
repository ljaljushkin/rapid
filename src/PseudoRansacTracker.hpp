#pragma once

#include "Tracker.hpp"
#include "MeanShift3D.hpp"

//Default settings
#define Max_Iter 2000
#define DistanceThreshold 0.01

//sp - SolvePnP
//ms - MeanShift

class PseudoRansacTracker : public Tracker
{
public:
    PseudoRansacTracker(
        Model model,
        const bool isLogsEnabled,
        const cv::Point3f ms_windowSizes,
        const size_t ms_maxIter = Max_Iter,
        const double ms_eps = DistanceThreshold,
        const size_t sp_iter = Max_Iter
        );

    virtual void RunSolvePnP(
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<cv::Point3f> modelPoints3D,
        cv::Mat& out_rvec,
        cv::Mat& out_tvec) const;

    /*virtual void MyMeanShift3D(
        const std::list<cv::Mat>* setPoints, 
        cv::Mat& foundCenter) const;*/
private:
    size_t iter;
    MeanShift3D* meanShift3D;
};
