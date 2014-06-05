#include <iostream>

#include "MeanShift3D.hpp"

using std::cout;
using std::endl;

using namespace cv;

MeanShift3D::MeanShift3D(
    const int _maxIter,
    const double _eps,
    const cv::Point3f _windowSizes,
    const bool _isIterCriteria)
    :   maxIter(_maxIter),
    eps(_eps),
    windowSizes(_windowSizes),
    isIterCriteria(_isIterCriteria)
{}

void MeanShift3D::execute(
    const std::list<cv::Point3f>* setOfPoints,
    cv::Mat& foundCenter) const
{
    //cout<<"------------------------MeanShift was started"<<std::endl;
    float dx = windowSizes.x/2;
    float dy = windowSizes.y/2;
    float dz = windowSizes.z/2;

    std::list<Point3f>::const_iterator setOfPointsIter = setOfPoints->begin();
    //cout<<*setOfPointsIter<<std::endl;

    //Point3f center(*setOfPointsIter);
    Point3f center(0,0,0);
    //cout<<"first_center: "<<center<<std::endl;

    int countInside = 0, numIter = 0;

    while(numIter != maxIter)
    {
        //cout<<numIter<<" <--- Iter"<<std::endl;
        setOfPointsIter = setOfPoints->begin();
        countInside = 0;
        Point3f sum(0,0,0);

        while(setOfPointsIter != setOfPoints->end())
        {
            if( (setOfPointsIter->x >= center.x - dx) && (setOfPointsIter->x <= center.x + dx) &&
                (setOfPointsIter->y >= center.y - dy) && (setOfPointsIter->y <= center.y + dy) &&
                (setOfPointsIter->z >= center.z - dz) && (setOfPointsIter->z <= center.z + dz) )
            {
                sum += *setOfPointsIter;
                countInside++;
            }
            setOfPointsIter++;
        }

        //cout<<"countInside: "<<countInside<<std::endl;

        float nx = sum.x/countInside;
        float ny = sum.y/countInside;
        float nz = sum.z/countInside;

        if( (nx - center.x)*(nx - center.x) + (ny - center.y)*(ny - center.y) + (nz - center.z)*(nz - center.z) > eps*eps)
        {
            center.x = nx;
            center.y = ny;
            center.z = nz;
            //cout<<"new_center: "<<center<<std::endl;
        }
        else
        {
            break;
        }

        numIter++;
    }

    foundCenter = Mat::zeros(3, 1, CV_64F);
    //cout<<"iterCount: "<<numIter<<std::endl;
    foundCenter.at<double>(0, 0) = center.x;
    foundCenter.at<double>(1, 0) = center.y;
    foundCenter.at<double>(2, 0) = center.z;
    //cout<<"found_center: "<<foundCenter<<std::endl;
    //cout<<"MeanShift was finished------------------------"<<std::endl;
}