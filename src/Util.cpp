#include "Util.hpp"

namespace util
{
    void getSubVectors(
        const std::vector<cv::Point3f> modelPoints3D,
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<unsigned> subset,
        std::vector<cv::Point3f> &out_subModelPoints3D,
        std::vector<cv::Point2f> &out_subFoundBoxPoints2D)
    {
        for (int i = 0; i < subset.size(); i++)
        {
            out_subModelPoints3D.push_back(modelPoints3D[subset[i]]);
            out_subFoundBoxPoints2D.push_back(foundBoxPoints2D[subset[i]]);
        }
    }

    void printVector(std::vector<unsigned>& vector)
    {
        std::vector<unsigned>::iterator Iter = vector.begin();
        while (Iter != vector.end())
        {
            std::cout<<*Iter<<" ";
            Iter++;
        }
        std::cout<<std::endl;
    }
}