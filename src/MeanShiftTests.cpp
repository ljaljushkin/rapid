#include <opencv2/core/core.hpp>
#include <gtest/gtest.h>

#include "MeanShift3D.hpp"

const char incorrectX [] = {"center x isn't correct"};
const char incorrectY [] = {"center y isn't correct"};
const char incorrectZ [] = {"center z isn't correct"};

void FillData_Two2DPoints(std::list<cv::Point3f>& setOfPoints)
{
    setOfPoints.push_back(cv::Point3f(2,0,0));
    setOfPoints.push_back(cv::Point3f(0,0,0));
    /*std::list<cv::Point3f>::iterator setOfPointsIter = setOfPoints.begin();
    std::cout<<"end fill"<<*setOfPointsIter<<std::endl;*/
}

void FillData_Four2DPoints(std::list<cv::Point3f>& setOfPoints)
{
    setOfPoints.push_back(cv::Point3f(2,0,0));
    setOfPoints.push_back(cv::Point3f(0,2,0));
    setOfPoints.push_back(cv::Point3f(2,2,0));
    setOfPoints.push_back(cv::Point3f(0,0,0));
}

void FillData_NumerousPointsOnPlane(std::list<cv::Point3f>& setOfPoints)
{
    setOfPoints.push_back(cv::Point3f(0, 0, 0));
    setOfPoints.push_back(cv::Point3f(2, 2, 0));
    setOfPoints.push_back(cv::Point3f(2, 3, 0));
    setOfPoints.push_back(cv::Point3f(2, 4, 0));
    setOfPoints.push_back(cv::Point3f(3, 2, 0));
    setOfPoints.push_back(cv::Point3f(3, 3, 0));
    setOfPoints.push_back(cv::Point3f(3, 4, 0));
    setOfPoints.push_back(cv::Point3f(4, 4, 0));
    setOfPoints.push_back(cv::Point3f(4, 3, 0));

    setOfPoints.push_back(cv::Point3f(2.5, 2.5, 0));
    setOfPoints.push_back(cv::Point3f(2.5, 4, 0));
    setOfPoints.push_back(cv::Point3f(3, 3.5, 0));
    setOfPoints.push_back(cv::Point3f(3.5, 4, 0));
}

class MeanShiftTest : public ::testing::Test
{
protected:
    void SetUp()
    {
        meanShift = new MeanShift3D(100, 0, cv::Point3f(4,4,0));
    }
protected:
    std::list<cv::Point3f> setOfPoints;
    MeanShift3D* meanShift;
};

TEST_F(MeanShiftTest, Two2DPoints)
{
    FillData_Two2DPoints(setOfPoints);
    cv::Mat center;

    meanShift->execute(&setOfPoints, center);

    EXPECT_EQ(1, center.at<float>(0,0)) << incorrectX;
    EXPECT_EQ(0, center.at<float>(1,0)) << incorrectY;
    EXPECT_EQ(0, center.at<float>(2,0)) << incorrectZ;
}

TEST_F(MeanShiftTest, Four2DPoints)
{
    FillData_Four2DPoints(setOfPoints);
    cv::Mat center(1, 3, CV_32F);

    meanShift->execute(&setOfPoints, center);

    EXPECT_EQ(1, center.at<float>(0,0)) << incorrectX;
    EXPECT_EQ(1, center.at<float>(1,0)) << incorrectY;
    EXPECT_EQ(0, center.at<float>(2,0)) << incorrectZ;
}

TEST_F(MeanShiftTest, NumerousPointsOnPlane)
{
    FillData_NumerousPointsOnPlane(setOfPoints);
    cv::Mat center(1, 3, CV_32F);

    meanShift->execute(&setOfPoints, center);

    EXPECT_LE(2, center.at<float>(0,0)) << incorrectX;
    EXPECT_LE(3, center.at<float>(1,0)) << incorrectY;
    EXPECT_EQ(0, center.at<float>(2,0)) << incorrectZ;
}