#include "PseudoRansacTracker.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;

using namespace cv;

PseudoRansacTracker::PseudoRansacTracker(
    Model model,
    bool _isLogsEnabled,
    double _distanceThreshold,
    size_t _maxIter)
    :   Tracker(model, isLogsEnabled),
        distanceThreshold(_distanceThreshold),
        maxIter(_maxIter)
{ }

#define finalEps 0.01
#define meanShiftMaxCount 100
#define meanShiftEpsilon 100

#define translateWindowX 3
#define translateWindowY 3
#define translateWindowZ 3
#define rotateWindowX 1
#define rotateWindowY 1
#define rotateWindowZ 100

void PseudoRansacTracker::RunSolvePnP(
    const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
    //meanshift to find center_window
//        // int meanShift(InputArray probImage, Rect& window, TermCriteria criteria)
//        // probImage - testModels[i].rvec, testModels[i].tvec
//        // window - enter from cmd or xml. rotate_epsilon and translate_epsilon
//        // criteria - maxIter, AnotherEpsilon - EndCriteria
//    // to see how many (points)inliers in that window (inl_r + inl_t)??
//    // out_bestModelIndex - center of found window.

    Mat set;
    Rect window;
    TermCriteria criteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, meanShiftMaxCount, meanShiftEpsilon);
    cv::meanShift(set, window, criteria);

    const unsigned int n = model.controlPoints.size();
    std::vector<unsigned> subset(4);

    util::RandomGenerator rng;
	rng.drawUniformSubset(n, 4, subset);

    std::vector<Point3f> subModelPoints3D;
    std::vector<Point2f> subFoundBoxPoints2D;

	getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);

    solvePnP(
        Mat(subModelPoints3D),
        Mat(subFoundBoxPoints2D),
        model.cameraMatrix,
        model.distortionCoefficients,
        out_rvec,
        out_tvec,
        false);

    if (isLogsEnabled)
    {
        //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
        cout << "---(SolvePnP) delta rotate vector" << endl << out_rvec - model.rotationVector<< endl;
        cout << "---(SolvePnP) delta translate vector=" << endl << out_tvec - model.translateVector << endl << endl;
    }
}