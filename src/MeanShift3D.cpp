#include <iostream>

#include "MeanShift3D.hpp"

using std::cout;
using std::endl;

using namespace cv;

MeanShift3D::MeanShift3D(
    const size_t _maxIter,
    const double _eps,
    const cv::Point3f _windowSizes,
    const bool _isIterCriteria)
    :   maxIter(_maxIter),
        eps(_eps),
        windowSizes(_windowSizes),
        isIterCriteria(_isIterCriteria)
{
    
    /*deltaBoundaries.push_back(Mat( _windowSizes.x/2,  _windowSizes.y/2,  _windowSizes.z/2));
    deltaBoundaries.push_back(Mat(-_windowSizes.x/2,  _windowSizes.y/2,  _windowSizes.z/2));
    deltaBoundaries.push_back(Mat( _windowSizes.x/2, -_windowSizes.y/2,  _windowSizes.z/2));
    deltaBoundaries.push_back(Mat(-_windowSizes.x/2, -_windowSizes.y/2,  _windowSizes.z/2));

    deltaBoundaries.push_back(Mat( _windowSizes.x/2,  _windowSizes.y/2, -_windowSizes.z/2));
    deltaBoundaries.push_back(Mat(-_windowSizes.x/2,  _windowSizes.y/2, -_windowSizes.z/2));
    deltaBoundaries.push_back(Mat( _windowSizes.x/2, -_windowSizes.y/2, -_windowSizes.z/2));
    deltaBoundaries.push_back(Mat(-_windowSizes.x/2, -_windowSizes.y/2, -_windowSizes.z/2));*/
}

void MeanShift3D::execute(
    const std::list<cv::Mat>* setPoints,
    cv::Mat& foundCenter) const
{
    double dx = windowSizes.x/2;
    double dy = windowSizes.y/2;
    double dz = windowSizes.z/2;

    std::list<Mat>::const_iterator setPointsIter = setPoints->begin();
    cout<<*setPointsIter<<std::endl;

    foundCenter = setPointsIter->clone(); // first point is the first center
    cout<<"first!"<<foundCenter<<std::endl;

    bool isConverged = false;
    int count = 0;

    double sumX = 0;
    double sumY = 0;
    double sumZ = 0;

    float cx = foundCenter.at<float>(0, 0);
    float cy = foundCenter.at<float>(0, 1);
    float cz = foundCenter.at<float>(0, 2);

    int coundInside = 0;

    while((count != maxIter) && (!isConverged))
    {
        setPointsIter = setPoints->begin();
        while(setPointsIter != setPoints->end())
        {
            float x = setPointsIter->at<float>(0,0);
            float y = setPointsIter->at<float>(0,1);
            float z = setPointsIter->at<float>(0,2);

            if( (x >= cx - dx) && (x <= cx + dx) && 
                (y >= cy - dy) && (y <= cy + dy) &&
                (z >= cz - dz) && (z <= cz + dz) )
            {
                sumX += x;
                sumY += y;
                sumZ += z;
                coundInside++;
            }
            
            /*cout<<*setPointsIter<<endl;
            cout<<" x_d: "<<setPointsIter->at<double>(0,0)<<endl; 
            cout<<" y_d: "<<setPointsIter->at<double>(1,0)<<endl; 
            cout<<" z_d: "<<setPointsIter->at<double>(2,0)<<endl; */
            //cout<<" x_d: "<<setPointsIter->at<double>(0,0)<<endl;

            setPointsIter++;
        }

        double nx = sumX/coundInside;
        double ny = sumY/coundInside;
        double nz = sumZ/coundInside;

        if( (nx - cx)*(nx - cx) + (ny - cy)*(ny - cy) + (nz - cz)*(nz - cz) > eps*eps*eps)
        {
            cx = nx;
            cy = ny;
            cz = nz;
        }
        else
        {
            isConverged = true;
            break;
        }

        count++;
    }

    foundCenter.at<float>(0, 0) = cx;
    foundCenter.at<float>(0, 1) = cy;
    foundCenter.at<float>(0, 2) = cz;
}
