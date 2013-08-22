#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "Model.hpp"

Model::Model(const cv::Mat& _T,
             const cv::Mat* _cornerPoints,
             const int      _pointsPerEdge,
             const cv::Mat& _cameraMatrix,
             const cv::Mat& _distortionCoefficients,
             const cv::Mat& _rotationVector,
             const cv::Mat& _translateVector)
{
	T = _T.clone();

	for(int i = 0; i < 8; i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge   = _pointsPerEdge;
    rotationVector  = _rotationVector;
    translateVector = _translateVector;
	SetControlPoints();
}

Model::Model(const cv::Mat& _T,
             const std::vector<cv::Mat> _cornerPoints,
             const int      _pointsPerEdge,
             const cv::Mat& _cameraMatrix,
             const cv::Mat& _distortionCoefficients,
             const cv::Mat& _rotationVector,
             const cv::Mat& _translateVector)
{
	T = _T.clone();

	for(unsigned int i = 0; i < _cornerPoints.size(); i++)
		cornerPoints.push_back(_cornerPoints[i].clone());

	cameraMatrix = _cameraMatrix.clone();
	distortionCoefficients = _distortionCoefficients.clone();

	pointsPerEdge   = _pointsPerEdge;
    rotationVector  = _rotationVector;
    translateVector = _translateVector;
	SetControlPoints();
}

Model::~Model()
{
	std::list<cv::Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		controlPointsIter->release();
		controlPointsIter++;
	}

	std::list<cv::Mat>::iterator companionPointsIter = companionPoints.begin();
	while (companionPointsIter != companionPoints.end())
	{
		companionPointsIter->release();
		companionPointsIter++;
	}

	std::vector<cv::Mat>::iterator cornerPointsIter = cornerPoints.begin();
	while (cornerPointsIter != cornerPoints.end())
	{
		cornerPointsIter->release();
		cornerPointsIter++;
	}
}

// Projecting points manually. Parameters selection based on luck and attentiveness.
cv::Point2d Model::Project(const cv::Mat& _3DPoint, double scaleCoeff, const cv::Point2d &translateVector) const
{
	// projecting
	cv::Point2d point(_3DPoint.at<double>(0,0) / (_3DPoint.at<double>(0,2)),
			      _3DPoint.at<double>(0,1) / (_3DPoint.at<double>(0,2)));

	// scaling
	point *= scaleCoeff;

	// moving
	point += translateVector;

	return point;
}

// Manual projecting based on camera calibrating data.
cv::Point2d Model::ManualProject(const cv::Mat& _3DPoint) const
{
    cv::Mat Box3DPoint = _3DPoint.clone();
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);

    Box3DPoint = rotationMatrix * Box3DPoint.t() + translateVector;

    double x = Box3DPoint.at<double>(0,0) / Box3DPoint.at<double>(2,0);
	double y = Box3DPoint.at<double>(1,0) / Box3DPoint.at<double>(2,0);

	double rQuad = x*x+y*y;

	double x1 = x*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);
	double y1 = y*(1+distortionCoefficients.at<double>(0,0)*rQuad+distortionCoefficients.at<double>(1,0)*rQuad*rQuad + distortionCoefficients.at<double>(4,0)*rQuad*rQuad*rQuad);

	double u = x1*cameraMatrix.at<double>(0,0) + cameraMatrix.at<double>(0,2);
	double v = y1*cameraMatrix.at<double>(1,1) + cameraMatrix.at<double>(1,2);

	return cv::Point2d(u,v);
}


// Projecting using OpenCV function
cv::Point2d Model::Project(const cv::Mat& _3DPoint) const
{
	std::vector<cv::Point2d> projectedPoints;
	cv::projectPoints(_3DPoint, rotationVector, translateVector, cameraMatrix, distortionCoefficients, projectedPoints);
    return projectedPoints[0];
}

cv::Mat Model::Outline(const cv::Mat& source)
{
	cv::Mat result = source.clone();

	cv::Scalar whiteColor = cv::Scalar(cv::Scalar::all(255));

	// drawing edges
	cv::line(result, Project(cornerPoints[0]), Project(cornerPoints[1]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[1]), Project(cornerPoints[2]), whiteColor, 1, 8);
 	cv::line(result, Project(cornerPoints[2]), Project(cornerPoints[3]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[0]), Project(cornerPoints[4]), whiteColor, 1, 8);
 
 	cv::line(result, Project(cornerPoints[4]), Project(cornerPoints[5]), whiteColor, 1, 8);
 	cv::line(result, Project(cornerPoints[5]), Project(cornerPoints[6]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[6]), Project(cornerPoints[7]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[7]), Project(cornerPoints[4]), whiteColor, 1, 8);

 	cv::line(result, Project(cornerPoints[0]), Project(cornerPoints[3]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[1]), Project(cornerPoints[5]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[2]), Project(cornerPoints[6]), whiteColor, 1, 8);
	cv::line(result, Project(cornerPoints[3]), Project(cornerPoints[7]), whiteColor, 1, 8);

	// drawing cotrol points
	std::list<cv::Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		cv::circle(result, Project(*controlPointsIter), 4, cv::Scalar(cv::Scalar::all(255)));
		controlPointsIter++;
	}

	// drawing companion points
// 	std::list<cv::Mat>::iterator companionPointsIter = companionPoints.begin();
// 	while (companionPointsIter != companionPoints.end())
// 	{
// 		cv::circle(result, Project(T+*companionPointsIter), 5, Scalar(0,255,0) );
// 		companionPointsIter++;
// 	}

	return result;
}

// adds control points located on i-j edge
void Model::AddControlPointsFromTheEdge(int i, int j)
{
    const double offset = 0.15;

	const double companionPointsOffset = 0.3;

	const cv::Mat direction = cornerPoints[i] - cornerPoints[j];

    cv::Mat p, s;

    p = cornerPoints[i] - direction*offset;
    controlPoints.push_back(p);
	p.release();

	s = cornerPoints[i] - direction*companionPointsOffset*2;
	companionPoints.push_back(s);
	s.release();
	

    p = cornerPoints[i] - direction*(1 - offset);
    controlPoints.push_back(p);
	p.release();

	s = cornerPoints[i] - direction*(1-companionPointsOffset*2);
	companionPoints.push_back(s);
	s.release();


    for (int k = 1; k < pointsPerEdge-1; k++)
	{
		p = cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1));
		controlPoints.push_back(p);
		p.release();

		s = cornerPoints[i] - direction*(k / (double) (pointsPerEdge-1) + companionPointsOffset);
		companionPoints.push_back(s);
		s.release();
	}
}

void Model::SetControlPoints()
{
	// pointsPerEdge control point correspond to every edge
	//AddControlPointsFromTheEdge(1, 2);
	AddControlPointsFromTheEdge(0, 1);
 	//AddControlPointsFromTheEdge(2, 3);
	//AddControlPointsFromTheEdge(0, 4);

	//AddControlPointsFromTheEdge(4, 5);
	AddControlPointsFromTheEdge(5, 6);
	AddControlPointsFromTheEdge(6, 7);
	//AddControlPointsFromTheEdge(7, 4);

	AddControlPointsFromTheEdge(0, 3);
	AddControlPointsFromTheEdge(1, 5);
	//AddControlPointsFromTheEdge(2, 6);
	AddControlPointsFromTheEdge(3, 7);
}

void Model::RotateAndTranslate(const cv::Mat &rotationVector, const cv::Mat &translateVector)
{

}

void Model::updatePose(const cv::Mat &solution)
{
	cv::Mat angle,distanse;
	angle=cv::Mat(solution,cv::Range(0,3),cv::Range(0,1));
	distanse=cv::Mat(solution,cv::Range(3,6),cv::Range(0,1));

//	std::cout<<"angle= "<<endl<<angle<<endl;
//	std::cout<<"distanse= "<<endl<<distanse<<endl;
//	std::cout<<"T= "<<endl<<T<<endl;

	T+=distanse.t();
//	std::cout<<"new T= "<<endl<<T<<endl;

//		cv::Mat test = (Mat_<double>(3,1) <<  0,0,1);
//		cv::Mat tesl = (Mat_<double>(3,1) <<  0,1,0);
//		std::cout<<"crossProduct!!"<<test.cross(tesl)<<endl;

	std::list<cv::Mat>::iterator controlPointsIter = controlPoints.begin();
	while (controlPointsIter != controlPoints.end())
	{
		//std::cout<<"Point= "<<(*controlPointsIter)<<endl;
		(*controlPointsIter)+=angle.t().cross(*controlPointsIter) + distanse.t();
		//std::cout<<"newPoint= "<<(*controlPointsIter)<<endl;
		controlPointsIter++;
	}

	std::list<cv::Mat>::iterator companionPointsIter = companionPoints.begin();
	while (companionPointsIter != companionPoints.end())
	{
		(*companionPointsIter)+=angle.t().cross(*companionPointsIter) + distanse.t();
		companionPointsIter++;
	}

	std::vector<cv::Mat>::iterator cornerPointsIter = cornerPoints.begin();
	while (cornerPointsIter != cornerPoints.end())
	{
		(*cornerPointsIter)+=angle.t().cross(*cornerPointsIter) + distanse.t();
		cornerPointsIter++;
	}

}
