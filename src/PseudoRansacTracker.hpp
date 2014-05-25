#pragma once

#include "Tracker.hpp"

//void LineDistanceFunctor(const std::vector<DataType> &allData,
//                         const std::vector<ModelParametersType> &testModels,
//                         const double distanceThreshold,
//                         unsigned &out_bestModelIndex,
//                         std::vector<unsigned> &out_inlierIndices)
//{
//    //meanshift to find center_window
//        // int meanShift(InputArray probImage, Rect& window, TermCriteria criteria)
//        // probImage - testModels[i].rvec, testModels[i].tvec
//        // window - enter from cmd or xml. rotate_epsilon and translate_epsilon
//        // criteria - maxIter, AnotherEpsilon - EndCriteria
//    // to see how many (points)inliers in that window (inl_r + inl_t)??
//    // out_bestModelIndex - center of found window.
//}

//Default settings
#define Max_Iter 2000
#define DistanceThreshold 0.01

class PseudoRansacTracker : public Tracker
{
public:
    PseudoRansacTracker(
        Model model,
        bool isLogsEnabled,
        double distanceThreshold = DistanceThreshold,
        size_t maxIter = Max_Iter);

    virtual void RunSolvePnP(
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<cv::Point3f> modelPoints3D,
        cv::Mat& out_rvec,
        cv::Mat& out_tvec) const;
private:
    double distanceThreshold;
    size_t maxIter;
};
