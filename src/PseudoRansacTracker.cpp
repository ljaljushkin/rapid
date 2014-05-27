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
    const double _ms_distanceThreshold,
    const size_t _iter)
    :   Tracker(model, isLogsEnabled),
        iter(_iter)
{ 
    meanShift3D = new MeanShift3D(_ms_maxIter, _ms_distanceThreshold, _ms_windowSizes);
}

//#define finalEps 0.01
//#define meanShiftMaxCount 100
//#define meanShiftEpsilon 100
//
//#define translateWindowX 3
//#define translateWindowY 3
//#define translateWindowZ 3
//#define rotateWindowX 1
//#define rotateWindowY 1
//#define rotateWindowZ 100

//void PseudoRansacTracker::MyMeanShift3D(const std::list<Mat>* setPoints, Mat& foundCenter) const
//{
//    std::list<Mat>::const_iterator setPointsIter = setPoints->begin();
//    foundCenter = setPointsIter->clone(); 
//    while(setPointsIter != setPoints->end())
//    {
//        //cout<<*setPointsIter<<endl;
//        //cout<<" x_d: "<<setPointsIter->at<double>(0,0)<<endl; 
//        //cout<<" y_d: "<<setPointsIter->at<double>(1,0)<<endl; 
//        //cout<<" z_d: "<<setPointsIter->at<double>(2,0)<<endl; 
//
//        cout<<" x_d: "<<setPointsIter->at<double>(0,0)<<endl;
//
//        setPointsIter++;
//    }
//    
//}

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

    /*std::vector<Point3f> rvecPool;
    std::vector<Point3f> tvecPool;*/

    std::list<Mat> rvecPool;
    std::list<Mat> tvecPool;

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

        rvecPool.push_back(rvec - model.rotationVector);
        tvecPool.push_back(tvec - model.translateVector);

        if (isLogsEnabled)
        {
            //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
            cout << "---(SolvePnP) delta rotate vector" << endl << rvec - model.rotationVector<< endl;
            cout << "---(SolvePnP) delta translate vector=" << endl << tvec - model.translateVector << endl << endl;
        }
    }

    // как передавать? по ссылке???
    //MyMeanShift3D(&rvecPool, out_rvec);
    //MyMeanShift3D(&tvecPool, out_tvec);
    meanShift3D->execute(&rvecPool, out_rvec);
    meanShift3D->execute(&tvecPool, out_tvec);

}