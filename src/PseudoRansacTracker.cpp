#include <time.h>
#include <fstream>

#include "opencv2/video/video.hpp"

#include "PseudoRansacTracker.hpp"
#include "Util.hpp"

using std::cout;
using std::endl;

using namespace cv;

PseudoRansacTracker::PseudoRansacTracker(
    Model model,
    const bool _isLogsEnabled,
    const Point3f _ms_windowSizesR,
    const Point3f _ms_windowSizesT,
    const int _ms_maxIter,
    const double _ms_epsR,
    const double _ms_epsT,
    const int _iter,
	const float _reprojectionError)
    :   Tracker(model, _isLogsEnabled),
        iter(_iter),
		reprojectionError(_reprojectionError)
{ 
    meanShift3DRotate = new MeanShift3D(_ms_maxIter, _ms_epsR, _ms_windowSizesR);
    meanShift3DTranslate = new MeanShift3D(_ms_maxIter, _ms_epsT, _ms_windowSizesT);
	rng = new util::RandomGenerator(1992);
}

void PseudoRansacTracker::OutputRvecAndTvec(const Mat& rvec, const Mat& tvec, std::ofstream& file) const
{
	Mat delta_rvec = rvec - model.rotationVector;
	Mat delta_tvec = tvec - model.translateVector;
	if (isLogsEnabled)
	{
		//cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
		cout << "---(SolvePnP) delta rotate vector" << endl << delta_rvec<< endl;
		cout << "---(SolvePnP) delta translate vector=" << endl << delta_tvec << endl << endl;
	}

	for(int i=0; i<3; i++)
		file << delta_rvec.at<double>(i, 0) << ", ";
	for(int i=0; i<2; i++)
		file << delta_tvec.at<double>(i, 0) << ", ";
	file << delta_tvec.at<double>(2, 0) << endl;
}

void PseudoRansacTracker::FindInliers(
	const std::vector<Point2f> foundBoxPoints2D,
	const std::list<Point2d> projectedPoints,
	const float reprojectionError,
	std::vector<unsigned>& out_subset) const
{
	std::list<Point2d>::const_iterator projectedPointsIter = projectedPoints.begin();
	for( unsigned i = 0; i < foundBoxPoints2D.size(); i++, projectedPointsIter++)
	{
		if( norm((Point2f)*projectedPointsIter - foundBoxPoints2D[i]) < reprojectionError)
		{
			out_subset.push_back(i);
		}
	}
}

void PseudoRansacTracker::RunSolvePnP(
    const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
	std::ofstream file, file_r, file_m, file_i;
	file.open ("../others/matlab_workspace/mean_shift_rvec_and_tvec.txt");
	file_r.open ("../others/matlab_workspace/ransac_rvec_and_tvec.txt");
	file_m.open ("../others/matlab_workspace/center_rvec_and_tvec.txt");
	file_i.open ("../others/matlab_workspace/with_inliers_rvec_and_tvec.txt");

    std::list<Point3f> rvecPool;
    std::list<Point3f> tvecPool;

    const unsigned int n = model.controlPoints.size();
    std::vector<unsigned> subset(4);

    //util::RandomGenerator rng(time(NULL));

    Mat rvec, tvec;
    for(int i=0; i < iter; i++)
    {
		std::vector<Point3f> subModelPoints3D;
		std::vector<Point2f> subFoundBoxPoints2D;

	    rng->drawUniformSubset(n-1, 4, subset);

	    util::getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);

        solvePnP(
            Mat(subModelPoints3D),
            Mat(subFoundBoxPoints2D),
            model.cameraMatrix,
            model.distortionCoefficients,
            rvec,
            tvec,
            false);

		OutputRvecAndTvec(rvec, tvec, file);

        rvecPool.push_back(Point3f(Mat(rvec - model.rotationVector)));
        tvecPool.push_back(Point3f(Mat(tvec - model.translateVector)));

        if (isLogsEnabled)
        {
            //cout << "---(SolvePnP) rotate vector" << endl << rvec << endl << "---(SolvePnP) translate vector=" << endl << tvec << endl;
            cout << "---(SolvePnP) delta rotate vector" << endl << rvec - model.rotationVector<< endl;
            cout << "---(SolvePnP) delta translate vector=" << endl << tvec - model.translateVector << endl << endl;
        }
    }

	// SolvePnPRansac
	Mat out_rvec_r, out_tvec_r;
	solvePnPRansac(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,
		model.distortionCoefficients, out_rvec_r, out_tvec_r, false,
		100, 8, 20);
	OutputRvecAndTvec(out_rvec_r, out_tvec_r, file_r);

	Mat ransacExtraImage = extraImage.clone();
	Model ransacModel(model);
	ransacModel.updatePose(out_rvec_r - ransacModel.rotationVector, out_tvec_r - ransacModel.translateVector);
	Mat r_image = ransacModel.Outline(ransacExtraImage);
	imshow("Ransac", r_image);

	// MeanShift
	Mat out_rvec_m, out_tvec_m;
	meanShift3DRotate->execute(&rvecPool, out_rvec_m);
	meanShift3DTranslate->execute(&tvecPool, out_tvec_m);

	out_rvec_m += model.rotationVector;
	out_tvec_m += model.translateVector;
	OutputRvecAndTvec(out_rvec_m, out_tvec_m, file_m);

	Model meanShiftModel(model);
	meanShiftModel.updatePose(out_rvec_m - model.rotationVector, out_tvec_m - model.translateVector);
	Mat m_image = meanShiftModel.Outline(extraImage);
	imshow("meanShift", m_image);

	//Output vectors
	cout<<"^^^^^^ Rotate error: "<<endl<<abs(out_rvec_m - out_rvec_r)<<endl;
	cout<<"^^^^^^ Translate error: "<<endl<<abs(out_tvec_m - out_tvec_r)<<endl;

	cout<<"Ransac_Rotate ^^^^^^^^^^ : "<<endl<<abs(out_rvec_r - model.rotationVector)<<endl;
	cout<<"Ransac_Translate ^^^^^^^ : "<<endl<<abs(out_tvec_r -model .translateVector)<<endl;

	// Improvement PseudoRansac algorithm
	std::vector<unsigned> inliers;
	FindInliers(foundBoxPoints2D, meanShiftModel.GetProjectedControlPoints(), reprojectionError, inliers);
	cout<<"Found inliers --> ";
	util::printVector(inliers);

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
	OutputRvecAndTvec(out_rvec, out_tvec, file_i);

	cout<<"Finish diff-rotate with ransac: "<<endl<<abs(out_rvec - out_rvec_r)<<endl;
	cout<<"Finish diff-translate with ransac: "<<endl<<abs(out_tvec - out_tvec_r)<<endl;
}



