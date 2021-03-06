#include <cmath>
#include <iostream>
#include <fstream>

#include <opencv2/calib3d/calib3d.hpp>

#include "RAPIDTrackerExperiment.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;
using namespace cv;

RAPIDTrackerExperiment::RAPIDTrackerExperiment(
    Model& _model,
    bool _isLogsEnabled,
    unsigned int _k)
    :	RAPIDTracker(_model, _isLogsEnabled), k(_k)
{}

RAPIDTrackerExperiment::~RAPIDTrackerExperiment()
{}

void RAPIDTrackerExperiment::OutputRvecAndTvec(const Mat& rvec, const Mat& tvec, std::ofstream& file) const
{
    Mat delta_rvec = rvec - model.rotationVector;
    Mat delta_tvec = tvec - model.translateVector;
    if (isLogsEnabled)
    {
        //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
        cout << "---(SolvePnP) delta rotate vector" << endl << delta_rvec<< endl;
        cout << "---(SolvePnP) delta translate vector=" << endl << delta_tvec << endl << endl;
    }

    for(int i=0; i<3; i++)
        file << delta_rvec.at<double>(i, 0) << ", ";
    for(int i=0; i<2; i++)
        file << delta_tvec.at<double>(i, 0) << ", ";
    file << delta_tvec.at<double>(2, 0) << endl;
}

void RAPIDTrackerExperiment::RunSolvePnP(
    const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
    std::ofstream file, file_r;
    file.open ("../others/matlab_workspace/rvec_and_tvec.txt");
    file_r.open ("../others/matlab_workspace/ransac_rvec_and_tvec.txt");
    //file.open ("../others/matlab_workspace/rvec_and_tvec.txt", std::ios::app);

    std::vector<unsigned> subset(k);

    // Initialization of the first subset
    for (int i = 0; i != k; i++)
        subset[i] = i;

    const unsigned int n = model.controlPoints.size();

    int idx = 0;

    while(GenerateNextSubset(subset, n))
    {
        cout << idx++ << endl;
        if (idx == 1000)
            break;
        std::vector<Point3f> subModelPoints3D;
        std::vector<Point2f> subFoundBoxPoints2D;

        util::getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);
        util::printVector(subset);

        solvePnP(Mat(subModelPoints3D), Mat(subFoundBoxPoints2D), model.cameraMatrix,
            model.distortionCoefficients, out_rvec, out_tvec, false);

        OutputRvecAndTvec(out_rvec, out_tvec, file);
    }
    Mat out_rvec_r, out_tvec_r;
    solvePnPRansac(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,
        model.distortionCoefficients, out_rvec_r, out_tvec_r, false,
        10, 0.5, 1);

    OutputRvecAndTvec(out_rvec_r, out_tvec_r, file_r);

    solvePnP(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,
        model.distortionCoefficients, out_rvec, out_tvec, false);

    file.close();
    file_r.close();
}