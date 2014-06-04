#pragma once

#include "Tracker.hpp"

class CvRansacTracker : public Tracker
{
public:
    CvRansacTracker(
        Model model,
        bool isLogsEnabled,
        int iterationsCount = 100,
        float reprojectionError = 8.0,
        int minInliersCount = 100);

    virtual void RunSolvePnP(
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<cv::Point3f> modelPoints3D,
        cv::Mat& out_rvec,
        cv::Mat& out_tvec) const;

    virtual void FindInliers(
		const std::vector<cv::Point2f> foundBoxPoints2D,
		const std::list<cv::Point2d> projectedPoints,
		const float reprojectionError,
		std::vector<unsigned>& out_subset,
		double& out_sum_norm) const;
private:
    int _iterationsCount;
    float _reprojectionError;
    int _minInliersCount;
};