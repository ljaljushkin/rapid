#include <iostream>

#include "MeanShift3D.hpp"

using std::cout;
using std::endl;

using namespace cv;

MeanShift3D::MeanShift3D(
    const size_t _maxIter,
    const double _distanceThreshold,
    const cv::Point3f _windowSizes,
    const bool _isIterCriteria)
    :   maxIter(_maxIter),
        distanceThreshold(_distanceThreshold),
        windowSizes(_windowSizes),
        isIterCriteria(_isIterCriteria)
{}

void MeanShift3D::execute(
    const std::list<cv::Mat>* setPoints,
    cv::Mat& foundCenter) const
{
    std::list<Mat>::const_iterator setPointsIter = setPoints->begin();
    foundCenter = setPointsIter->clone(); 
    while(setPointsIter != setPoints->end())
    {
        cout<<*setPointsIter<<endl;
        cout<<" x_d: "<<setPointsIter->at<double>(0,0)<<endl; 
        cout<<" y_d: "<<setPointsIter->at<double>(1,0)<<endl; 
        cout<<" z_d: "<<setPointsIter->at<double>(2,0)<<endl; 

        //cout<<" x_d: "<<setPointsIter->at<double>(0,0)<<endl;

        setPointsIter++;
    }
}
