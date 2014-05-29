#include <opencv2/core/core.hpp>
#include <gtest/gtest.h>

#include "MeanShift3D.hpp"

void FillData(std::list<cv::Mat>& setPoints)
{
    std::vector<cv::Point3f> vec;
    vec.push_back(cv::Point3f(2,0,0));
    cv::Mat mat = (cv::Mat)vec;

    std::vector<cv::Point3f> vec2;
    vec2.push_back(cv::Point3f(0,0,0));
    cv::Mat mat2 = (cv::Mat)vec2;

    std::cout<<"mat_only"<<mat2<<std::endl;

    /*setPoints->push_back(cv::Mat(1,1,0));
    setPoints->push_back(cv::Mat(2,2,0));
    setPoints->push_back(cv::Mat(0,2,0));*/

    //setPoints = new std::list<cv::Mat>;

    setPoints.push_back(mat.clone());
    setPoints.push_back(mat2.clone());

    std::list<cv::Mat>::iterator setPointsIter = setPoints.begin();
    std::cout<<"end fill"<<*setPointsIter<<std::endl;
}


class MeanShiftTest : public ::testing::Test
{
protected:
   void SetUp()
   {
       //setPoints = new std::list<cv::Mat>();
       FillData(setPoints);
       std::list<cv::Mat>::iterator setPointsIter = setPoints.begin(); 
       std::cout<<"after fill: "<<*setPointsIter<<std::endl;
       setPointsIter++;
       std::cout<<*setPointsIter<<std::endl;

       meanShift = new MeanShift3D(100, 0.01, cv::Point3f(4,4,0));
   }
protected:
   std::list<cv::Mat> setPoints;
   MeanShift3D* meanShift;
};

TEST_F(MeanShiftTest, Two2DPoints)
{
    cv::Mat center;

    std::list<cv::Mat>::const_iterator setPointsIter = setPoints.begin();
    std::cout<<*setPointsIter<<std::endl;

    meanShift->execute(&setPoints, center);

    EXPECT_EQ(1, center.at<float>(0,0)) << "center x isn't correct";
    EXPECT_EQ(0, center.at<float>(0,1)) << "center y isn't correct";
    EXPECT_EQ(0, center.at<float>(0,2)) << "center z isn't correct";
}
