#include "CvRansacTracker.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;

using namespace cv;

CvRansacTracker::CvRansacTracker(
    Model model,
    bool isLogsEnabled,
    int iterationsCount,
    float reprojectionError,
    int minInliersCount)
    :   Tracker(model, isLogsEnabled),
        _iterationsCount(iterationsCount),
        _reprojectionError(reprojectionError),
        _minInliersCount(minInliersCount)
{ }

void CvRansacTracker::FindInliers(
	const std::vector<Point2f> foundBoxPoints2D,
	const std::list<Point2d> projectedPoints,
	const float reprojectionError,
	std::vector<unsigned>& out_subset,
	double& out_sum_norm) const
{
	out_sum_norm = 0;
	std::list<Point2d>::const_iterator projectedPointsIter = projectedPoints.begin();
	for( unsigned i = 0; i < foundBoxPoints2D.size(); i++, projectedPointsIter++)
	{
		double curr_norm = norm((Point2f)*projectedPointsIter - foundBoxPoints2D[i]);
		out_sum_norm += curr_norm;

		if( curr_norm < reprojectionError)
		{
			out_subset.push_back(i);
		}
	}
}

void CvRansacTracker::RunSolvePnP(
    const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
    Mat rvec, tvec;
    solvePnPRansac(
        Mat(modelPoints3D),
        Mat(foundBoxPoints2D),
        model.cameraMatrix,
        model.distortionCoefficients,
        rvec,
        tvec,
        false,
        _iterationsCount,
        _reprojectionError,
        _minInliersCount);

    Model ransacModel(model);
	ransacModel.updatePose(rvec - ransacModel.rotationVector, tvec - ransacModel.translateVector);
	
    std::vector<unsigned> inliers;
    double curr_precision;
	FindInliers(foundBoxPoints2D, ransacModel.GetProjectedControlPoints(), _reprojectionError, inliers, curr_precision);

    cout << "ransac precision: " << curr_precision << endl;

    // --------- Finish SolvePnP

    std::vector<Point3f> subModelPoints3D;
	std::vector<Point2f> subFoundBoxPoints2D;
	util::getSubVectors(modelPoints3D, foundBoxPoints2D, inliers, subModelPoints3D, subFoundBoxPoints2D);
		
	solvePnP(
		Mat(subModelPoints3D),
		Mat(subFoundBoxPoints2D),
		model.cameraMatrix,
		model.distortionCoefficients,
		out_rvec,
		out_tvec,
		false);

    Model finalModel(model);
	ransacModel.updatePose(rvec - finalModel.rotationVector, tvec - finalModel.translateVector);
	
    inliers.clear();
	FindInliers(foundBoxPoints2D, finalModel.GetProjectedControlPoints(), _reprojectionError, inliers, curr_precision);
	
    cout << "final precision: " << curr_precision << endl;

    if (isLogsEnabled)
    {
        //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
        cout << "---(SolvePnP) delta rotate vector" << endl << out_rvec - model.rotationVector<< endl;
        cout << "---(SolvePnP) delta translate vector=" << endl << out_tvec - model.translateVector << endl << endl;
    }
}