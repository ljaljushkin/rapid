#include "time.h"

#include "opencv2/video/video.hpp"

#include "PseudoRansacTracker.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;

using namespace cv;

PseudoRansacTracker::PseudoRansacTracker(
    Model model,
    const bool _isLogsEnabled,
    const Point3f _ms_windowSizes,
    const size_t _ms_maxIter,
    const double _ms_eps,
    const size_t _iter)
    :   Tracker(model, isLogsEnabled),
        iter(_iter)
{ 
    meanShift3D = new MeanShift3D(_ms_maxIter, _ms_eps, _ms_windowSizes);
}

void PseudoRansacTracker::RunSolvePnP(
    const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
    std::list<Point3f> rvecPool;
    std::list<Point3f> tvecPool;

    std::vector<Point3f> subModelPoints3D;
    std::vector<Point2f> subFoundBoxPoints2D;

    const unsigned int n = model.controlPoints.size();
    std::vector<unsigned> subset(4);

    util::RandomGenerator rng(time(NULL));

    Mat rvec, tvec;
    for(int i=0; i < iter; i++)
    {
	    rng.drawUniformSubset(n, 4, subset);

	    getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);

        solvePnP(
            Mat(subModelPoints3D),
            Mat(subFoundBoxPoints2D),
            model.cameraMatrix,
            model.distortionCoefficients,
            rvec,
            tvec,
            false);

        rvecPool.push_back(Point3f(Mat(rvec - model.rotationVector)));
        tvecPool.push_back(Point3f(Mat(tvec - model.translateVector)));

        if (isLogsEnabled)
        {
            //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
            cout << "---(SolvePnP) delta rotate vector" << endl << rvec - model.rotationVector<< endl;
            cout << "---(SolvePnP) delta translate vector=" << endl << tvec - model.translateVector << endl << endl;
        }
    }

    meanShift3D->execute(&rvecPool, out_rvec);
    meanShift3D->execute(&tvecPool, out_tvec);

}