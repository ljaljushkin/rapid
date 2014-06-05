#pragma once

#include <cmath>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/flann/timer.h>

#include "Model.hpp"
#include "EdgeExtractor.hpp"

// Describes possible search directions among a row of pixels
enum Direction {HORIZONTAL, VERTICAL, UPWARD_DIAGONAL, DOWNWARD_DIAGONAL};

class Tracker : virtual protected EdgeExtractor
{
public:
    Tracker(Model _model, bool _isLogsEnabled) : model(_model)
    {
        isLogsEnabled = _isLogsEnabled;
    }
    double GetConvergenceMeasure(const Model& model1, const Model& model2, int normType) const;

    virtual cv::Mat	ExtractEdges(const cv::Mat& image) const;

    virtual void GetAndDrawPointsForSolvePnP(
        const cv::Mat& frame,
        std::vector<cv::Point2f>& out_foundBoxPoints2D,
        std::vector<cv::Point3f>& out_modelPoints3D);

    virtual void RunSolvePnP(
        const std::vector<cv::Point2f> foundBoxPoints2D,
        const std::vector<cv::Point3f> modelPoints3D,
        cv::Mat& out_rvec,
        cv::Mat& out_tvec) const;

    virtual Model ProcessFrame(const cv::Mat& frame);

    virtual void SetExtraImage(const cv::Mat& _extraImage)
    {
        extraImage = _extraImage.clone();
    }
protected:
    bool FindPoints(cv::Point2d controlPoint,
        cv::Point2d companionPoint,
        const cv::Mat& edges,
        cv::Point2d& foundPoint,
        cv::Point2d& foundPoint2);
protected:
    Model model;
    bool isLogsEnabled;
    cv::Mat extraImage;
};