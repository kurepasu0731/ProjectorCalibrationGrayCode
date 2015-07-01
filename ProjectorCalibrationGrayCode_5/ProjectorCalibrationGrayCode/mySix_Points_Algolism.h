#pragma once

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;

bool six_points_algolism(vector<Point3d>& P, vector<Point2d>& p, Mat& cameraMat){
	cameraMat = Mat(3, 4, CV_64FC1, Scalar::all(0));
	Mat A(12,12,CV_64FC1, Scalar::all(0));
	Mat At(12,12,CV_64FC1, Scalar::all(0));
	Mat W(12,12,CV_64FC1, Scalar::all(0));

	//行列Aの作成
	for(int i=0; i<6; i++){
		A.at<double>(2*i,0) = P[i].x;			A.at<double>(2*i,1) = P[i].y;		   A.at<double>(2*i,2) = P[i].z;
		A.at<double>(2*i,3) = 1.0;				A.at<double>(2*i,4) = 0.0;			   A.at<double>(2*i,5) = 0.0;
		A.at<double>(2*i,6) = 0.0;				A.at<double>(2*i,7) = 0.0;			   A.at<double>(2*i,8) = -1*P[i].x*p[i].x;
		A.at<double>(2*i,9) = -1*P[i].y*p[i].x; A.at<double>(2*i,10)=-1*P[i].z*p[i].x; A.at<double>(2*i,11) = -1*p[i].x;

		A.at<double>(2*i+1,0) = 0.0;			  A.at<double>(2*i+1,1) = 0.0;			   A.at<double>(2*i+1,2) = 0.0;
		A.at<double>(2*i+1,3) = 0.0;			  A.at<double>(2*i+1,4) = P[i].x;		   A.at<double>(2*i+1,5) =P[i].y;
		A.at<double>(2*i+1,6) =  P[i].z;		  A.at<double>(2*i+1,7) = 1.0;			   A.at<double>(2*i+1,8) = -1*P[i].x*p[i].y;
		A.at<double>(2*i+1,9) = -1*P[i].y*p[i].y; A.at<double>(2*i+1,10)=-1*P[i].z*p[i].y; A.at<double>(2*i+1,11) = -1*p[i].y;
	}
	//std::cout << "A = " << A << std::endl;
	transpose(A, At);
	W = At * A;


	//固有値・固有ベクトルを求める
	cv::Mat eigenValues, eigenVectors;
	cv::eigen(W, eigenValues, eigenVectors);
	//最小固有値に対応する固有ベクトルの取得(固有値は降順に格納されている)
	Mat min_eigenVectors(1,12,CV_64FC1, Scalar::all(0));
	for(int i=0; i<12; i++){
		min_eigenVectors.at<double>(0,i) = eigenVectors.at<double>(11,i);
	}

	//カメラMatrixの取得
	for(int i=0; i<12; i++){
		cameraMat.at<double>(i/4,i%4) = min_eigenVectors.at<double>(0,i);
	}

	/*std::cout << "matrix A" << std::endl << A << std::endl << std::endl;
	std::cout << "matrix At" << std::endl << At << std::endl << std::endl;
	std::cout << "matrix W" << std::endl << W << std::endl << std::endl;
	std::cout << "eigen Values of W" << std::endl << eigenValues << std::endl << std::endl;
	std::cout << "eigen vectors of W" << std::endl << eigenVectors << std::endl << std::endl;
	std::cout << "minimum eigen vectors of W" << std::endl << min_eigenVectors << std::endl << std::endl;*/
	return true;
}

//100(num)点バージョン　
bool six_points_algolism_2(vector<Point3d>& P, vector<Point2d>& p, Mat& cameraMat, int num){
	cameraMat = Mat(3, 4, CV_64FC1, Scalar::all(0));
	Mat A(num*2,12,CV_64FC1, Scalar::all(0));
	Mat At(12,num*2,CV_64FC1, Scalar::all(0));
	Mat W(12,12,CV_64FC1, Scalar::all(0));

	//行列Aの作成
	for(int i=0; i<num; i++){
		A.at<double>(2*i,0) = P[i].x;			A.at<double>(2*i,1) = P[i].y;		   A.at<double>(2*i,2) = P[i].z;
		A.at<double>(2*i,3) = 1.0;				A.at<double>(2*i,4) = 0.0;			   A.at<double>(2*i,5) = 0.0;
		A.at<double>(2*i,6) = 0.0;				A.at<double>(2*i,7) = 0.0;			   A.at<double>(2*i,8) = -1*P[i].x*p[i].x;
		A.at<double>(2*i,9) = -1*P[i].y*p[i].x; A.at<double>(2*i,10)=-1*P[i].z*p[i].x; A.at<double>(2*i,11) = -1*p[i].x;

		A.at<double>(2*i+1,0) = 0.0;			  A.at<double>(2*i+1,1) = 0.0;			   A.at<double>(2*i+1,2) = 0.0;
		A.at<double>(2*i+1,3) = 0.0;			  A.at<double>(2*i+1,4) = P[i].x;		   A.at<double>(2*i+1,5) =P[i].y;
		A.at<double>(2*i+1,6) =  P[i].z;		  A.at<double>(2*i+1,7) = 1.0;			   A.at<double>(2*i+1,8) = -1*P[i].x*p[i].y;
		A.at<double>(2*i+1,9) = -1*P[i].y*p[i].y; A.at<double>(2*i+1,10)=-1*P[i].z*p[i].y; A.at<double>(2*i+1,11) = -1*p[i].y;
	}
	//std::cout << "A = " << A << std::endl;
	transpose(A, At);
	W = At * A;


	//固有値・固有ベクトルを求める
	cv::Mat eigenValues, eigenVectors;
	cv::eigen(W, eigenValues, eigenVectors);
	//最小固有値に対応する固有ベクトルの取得(固有値は降順に格納されている)
	Mat min_eigenVectors(1,12,CV_64FC1, Scalar::all(0));
	for(int i=0; i<12; i++){
		min_eigenVectors.at<double>(0,i) = eigenVectors.at<double>(11,i);
	}

	//カメラMatrixの取得
	for(int i=0; i<12; i++){
		cameraMat.at<double>(i/4,i%4) = min_eigenVectors.at<double>(0,i);
	}

	/*std::cout << "matrix A" << std::endl << A << std::endl << std::endl;
	std::cout << "matrix At" << std::endl << At << std::endl << std::endl;
	std::cout << "matrix W" << std::endl << W << std::endl << std::endl;
	std::cout << "eigen Values of W" << std::endl << eigenValues << std::endl << std::endl;
	std::cout << "eigen vectors of W" << std::endl << eigenVectors << std::endl << std::endl;
	std::cout << "minimum eigen vectors of W" << std::endl << min_eigenVectors << std::endl << std::endl;*/
	return true;
}