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
    const int _iter)
    :   Tracker(model, _isLogsEnabled),
        iter(_iter)
{ 
    meanShift3DRotate = new MeanShift3D(_ms_maxIter, _ms_epsR, _ms_windowSizesR);
    meanShift3DTranslate = new MeanShift3D(_ms_maxIter, _ms_epsT, _ms_windowSizesT);
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

void PseudoRansacTracker::RunSolvePnP(
    const std::vector<Point2f> foundBoxPoints2D,
    const std::vector<Point3f> modelPoints3D,
    Mat& out_rvec,
    Mat& out_tvec) const
{
	std::ofstream file, file_r, file_m;
	file.open ("../others/matlab_workspace/mean_shift_rvec_and_tvec.txt");
	file_r.open ("../others/matlab_workspace/ransac_rvec_and_tvec.txt");
	file_m.open ("../others/matlab_workspace/center_rvec_and_tvec.txt");

    std::list<Point3f> rvecPool;
    std::list<Point3f> tvecPool;

    std::vector<Point3f> subModelPoints3D;
    std::vector<Point2f> subFoundBoxPoints2D;

    const unsigned int n = model.controlPoints.size();
    std::vector<unsigned> subset(4);

    util::RandomGenerator rng(time(NULL));

    Mat rvec, tvec;
    for(int i=0; i < iter; i++)
    {
	    rng.drawUniformSubset(n-1, 4, subset);

	    getSubVectors(modelPoints3D, foundBoxPoints2D, subset, subModelPoints3D, subFoundBoxPoints2D);

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

	Mat out_rvec_m, out_tvec_m;
    meanShift3DRotate->execute(&rvecPool, out_rvec_m);
    meanShift3DTranslate->execute(&tvecPool, out_tvec_m);
	out_rvec_m += model.rotationVector;
	out_tvec_m += model.translateVector;
	OutputRvecAndTvec(out_rvec_m, out_tvec_m, file_m);

	
	solvePnPRansac(Mat(modelPoints3D), Mat(foundBoxPoints2D), model.cameraMatrix,
		model.distortionCoefficients, out_rvec, out_tvec, false,
		10, 0.5, 1);
	OutputRvecAndTvec(out_rvec, out_tvec, file_r);

}
