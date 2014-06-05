#pragma once

#include "Util.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/internal.hpp"

class MyPnPSolver
{
public:
    void operator()( const cv::BlockedRange& r ) const
    {
        for( int i=r.begin(); i!=r.end(); ++i )
        {
            const unsigned int n = modelPoints3D.size();
            std::vector<unsigned> subset(4);

            cv::Mat rvec, tvec;
            std::vector<cv::Point3f> subModelPoints3D;
            std::vector<cv::Point2f> subFoundBoxPoints2D;

            rng->drawUniformSubset(n-1, 4, subset);

            util::getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);

            solvePnP(
                cv::Mat(subModelPoints3D),
                cv::Mat(subFoundBoxPoints2D),
                cameraMatrix,
                distortionCoefficients,
                rvec,
                tvec,
                false);

            out_rvecPool.push_back(cv::Point3f(cv::Mat(rvec - rotationVector)));
            out_tvecPool.push_back(cv::Point3f(cv::Mat(tvec - translateVector)));
        }
    }

    MyPnPSolver(
        const std::vector<cv::Point3f>& _modelPoints3D,
        const std::vector<cv::Point2f>& _foundBoxPoints2D,
        const cv::Mat& _cameraMatrix,
        const cv::Mat& _distortionCoefficients,
        const cv::Mat& _rotationVector,
        const cv::Mat& _translateVector,
        util::RandomGenerator* _rng,
        std::list<cv::Point3f>& _out_rvecPool,
        std::list<cv::Point3f>& _out_tvecPool)
        :   modelPoints3D(_modelPoints3D),
            foundBoxPoints2D(_foundBoxPoints2D),
            cameraMatrix(_cameraMatrix),
            distortionCoefficients(_distortionCoefficients),
            rotationVector(_rotationVector),
            translateVector(_translateVector),
            rng(_rng),
            out_rvecPool(_out_rvecPool),
            out_tvecPool(_out_tvecPool)
    {}

private:
    const std::vector<cv::Point3f>& modelPoints3D;
    const std::vector<cv::Point2f>& foundBoxPoints2D;

    const cv::Mat& cameraMatrix;
    const cv::Mat& distortionCoefficients;
    const cv::Mat& rotationVector;
    const cv::Mat& translateVector;

    util::RandomGenerator* rng;

public:
    std::list<cv::Point3f>& out_rvecPool;
    std::list<cv::Point3f>& out_tvecPool;
};