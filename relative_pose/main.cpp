/* program to calculate relative position and orientation between two cameras
 * basic concept from epipolar geometery of stereo vision
 * All input data is in .txt file format
 */
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <iomanip>

 // opencv preprocessors to get two cameras relation
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


using namespace std;
using namespace cv;

// Function prototyping

// 2D image points extractor from text file data.
vector<Point2d> image_points(ifstream& csvImage);

int main(int argc, char **argv)
{

	// 2D image points 1 input in pixel
	ifstream im1;

	im1.open("../input_data/single_1.txt");

	vector<Point2d> image1 = image_points(im1);

	im1.close();

	cout << " image points : " << endl << image1 << endl;

	cout << endl;

	//2d image points 2 input in pixel
	ifstream im2;

	im2.open("../input_data/single_2.txt");

	vector<Point2d> image2 = image_points(im2);

	im2.close();

	cout << " image points 2 : " << endl << image2 << endl;

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


	// stereo relations

	// initialize Mat object for homography, essential, and fundamental matrix
	Mat H, E, F;

	// initialize Mat object for rotation, translation, and mask matrix
	Mat rotation_vector, translation_vector, mask;

	// outputing homography matrix into text file
	ofstream csvHomography("../output_data/homography_matrix.txt");

	H = findHomography(image1, image2, RANSAC, 3, mask, 2000, 0.995);

	csvHomography << setprecision(10) << H << endl;

	csvHomography.close();

	cout << "input mask to get essential and fundamental matrix: \n" << mask << endl;

	cout << endl;

	cout << "Homography matrix: \n" << H << endl;

	cout << endl;

	// essential matrix
	ofstream csvEssential("../output_data/camera_essential.txt");

	E = findEssentialMat(image1, image2, camera_matrix, RANSAC, 0.995, 3, mask);

	// outputing essential matrix into text file
	csvEssential << setprecision(10) << E << endl;

	csvEssential.close();

	cout << "Essential Matrix: \n" << E << endl;

	cout << endl;

	// fundamental matrix
	ofstream csvFundamental("../output_data/camera_fundamental.txt");

	F = findFundamentalMat(image1, image2, FM_RANSAC, 3, 0.99, mask);

	// outputing fundamental matrix into text file
	csvFundamental << setprecision(10) << F << endl;

	csvFundamental.close();

	cout << "Fundamental Matrix: \n" << F << endl;

	cout << endl;

	ofstream csvRelativeRotation("../output_data/relative_rotation.txt");
	ofstream csvRelativeTranslation("../output_data/relative_translation.txt");

	// Solver for for finding relative rotation and translation between two cameras
	recoverPose(E, image1, image2, camera_matrix, rotation_vector, translation_vector, mask);

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

//2D image points extractor from text file data.
vector<Point2d> image_points(ifstream& csvImage)
{
	vector<Point2d> imagePoints;

	double u, v;

	while (!csvImage.eof())
	{
		csvImage >> u >> v;
		imagePoints.push_back(Point2d(u, v));
	}


	return imagePoints;
}