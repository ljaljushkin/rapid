#pragma once

#include "Tracker.hpp"
#include "MeanShift3D.hpp"
#include "Util.hpp"

//Default settings
#define Max_Iter 2000
#define DistanceThreshold 0.01
#define ReprojectionError 8

//sp - SolvePnP
//ms - MeanShift

class PseudoRansacTracker : public Tracker
{
public:
    PseudoRansacTracker(
        Model model,
        const bool isLogsEnabled,
        const cv::Point3f _ms_windowSizesR,
        const cv::Point3f _ms_windowSizesT,
        const int ms_maxIter = Max_Iter,
        const double _ms_epsR = DistanceThreshold,
        const double _ms_epsT = DistanceThreshold,
        const int sp_iter = Max_Iter,
		const float sp_reprojectionError = ReprojectionError
        );

    virtual void RunSolvePnP(
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<cv::Point3f> modelPoints3D,
        cv::Mat& out_rvec,
        cv::Mat& out_tvec) const;  

private:
	virtual void  OutputRvecAndTvec(
		const cv::Mat& out_rvec,
		const cv::Mat& out_tvec,
		std::ofstream& file) const;

	virtual void FindInliers(
		const std::vector<cv::Point2f> foundBoxPoints2D,
		const std::list<cv::Point2d> projectedPoints,
		const float reprojectionError,
		std::vector<unsigned>& out_subset,
		double& out_sum_norm) const;
private:
    int iter;
	float reprojectionError;
    MeanShift3D* meanShift3DRotate;
    MeanShift3D* meanShift3DTranslate;
	util::RandomGenerator* rng;
};
