/* program to calculate relative position and orientation between two cameras 
 * basic concept from epipolar geometery of stereo vision
 */
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>


#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
	//2d image points 1 
	ifstream csvImage1("../input_data/single_1.txt");

	vector<Point2f> image_points1;

	float u, v;

	while (!csvImage1.eof())
	{
		csvImage1 >> u >> v;
		image_points1.push_back(Point2f(u, v));
	}

	cout << " image points1 : " << endl << image_points1 << endl;

	csvImage1.close();

	cout << endl;

	//2d image points 2 
	ifstream csvImage2("../input_data/single_2.txt");

	vector<Point2f> image_points2;

	float l, m;

	while (!csvImage2.eof())
	{
		csvImage2 >> l >> m;
		image_points2.push_back(Point2f(l, m));
	}

	cout << " image points2 : " << endl << image_points2 << endl;

	csvImage2.close();

	cout << endl;


	// Camera internals
	// fx 0  cx 
	// 0  fy cy
	// 0  0   1

	ifstream csvIntrinsic("../input_data/camera_intrinsic.txt");

	double arrayIntrinsic[3][3];

	for (int row = 0; row < 3; row++)
	{
		for (int col = 0; col < 3; col++)
		{
			csvIntrinsic >> arrayIntrinsic[row][col];

		}

	}

	Mat camera_matrix = Mat(3, 3, CV_64F, arrayIntrinsic);
	cout << "Camera Matrix " << endl << camera_matrix << endl;

	csvIntrinsic.close();
	cout << endl;

	//distortion coefficient
	ifstream csvDist_coeffs("../input_data/dist_coeffs.txt");

	double arrayDist_coeffs[5][1];

	for (int row = 0; row < 5; row++)
	{
		for (int col = 0; col < 1; col++)
		{
			csvDist_coeffs >> arrayDist_coeffs[row][col];

		}

	}

	Mat dist_coeffs = Mat(5, 1, CV_64F, arrayDist_coeffs);
	csvDist_coeffs.close();


	//outputing stereo relations

	// initialize Mat object
	Mat H, E, F, rotation_vector, translation_vector, mask;

	ofstream csvHomography("../output_data/homography_matrix.txt");

	H = findHomography(image_points1, image_points2, RANSAC, 3, mask, 2000, 0.995);

	// outputing homography matrix into text file
	csvHomography << setprecision(10) << H << endl;

	csvHomography.close();

	cout << "input mask to get essential and fundamental matrix: \n" << mask << endl;

	cout << endl;

	cout << "Homography matrix: \n" << H << endl;

	cout << endl;

	// essential matrix
	ofstream csvEssential("../output_data/camera_essential.txt");

	E = findEssentialMat(image_points1, image_points2, camera_matrix, RANSAC, 0.995, 3, mask);

	// outputing essential matrix into text file
	csvEssential << setprecision(10) << E << endl;

	csvEssential.close();

	cout << "Essential Matrix: \n" << E << endl;

	cout << endl;

	// fundamental matrix
	ofstream csvFundamental("../output_data/camera_fundamental.txt");

	F = findFundamentalMat(image_points1, image_points2, FM_RANSAC, 3, 0.99, mask);

	// outputing fundamental matrix into text file
	csvFundamental << setprecision(10) << F << endl;

	csvFundamental.close();

	cout << "Fundamental Matrix: \n" << F << endl;

	cout << endl;

	ofstream csvRelativeRotation("../output_data/relative_rotation.txt");
	ofstream csvRelativeTranslation("../output_data/relative_translation.txt");

	// Solver for rotation and translation
	recoverPose(E, image_points1, image_points2, camera_matrix, rotation_vector, translation_vector, mask);

	Mat relative_rotation(3, 1, CV_64F);

	// find rotation in euler form using rodrigues method
	Rodrigues(rotation_vector, relative_rotation);

	double phi = 3.14159;

	// outputing relative translation and rotation into text file
	csvRelativeRotation << setprecision(10) << (relative_rotation) * 180 / phi << endl;
	csvRelativeTranslation << setprecision(10) << translation_vector << endl;

	csvRelativeRotation.close();
	csvRelativeTranslation.close();

	cout << "Relative rotation (in degree):  " << endl << (relative_rotation) * 180 / phi << "\n" << endl;
	cout << "Relative translation (in mm):" << endl << translation_vector << "\n" << endl;


	system("pause");
	return 0;

}