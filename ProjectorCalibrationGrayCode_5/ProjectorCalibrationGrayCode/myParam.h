#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>


using namespace cv;
using namespace std;

typedef struct {
	Size imageSize;
	Mat perspectiveMat;		//“§‹“Š‰es—ñ
	Mat internalMat;		//“à•”s—ñ
	Mat translationalMat;	//•Àis—ñ
	Mat rotationMat;		//‰ñ“]s—ñ
	Mat distCoffs;			//˜c‚İŒW”
	double alpha;
	double beta;
} Params;


Params loadParams(const string& filename){
	Params re;

	FileStorage fs(filename, FileStorage::READ);
	FileNode node(fs.fs, NULL);

	cv::read(node["perspective_matrix"], re.perspectiveMat);
	cv::read(node["internal_matrix"], re.internalMat);
	cv::read(node["translational_matrix"], re.translationalMat);
	cv::read(node["rotation_matrix"], re.rotationMat);
	cv::read(node["distortion_coefficients"], re.distCoffs);

	re.imageSize.width = node["image_width"];
	re.imageSize.height = node["image_height"];

	re.alpha = node["alpha"];
	re.beta = node["beta"];

	return re;
}

void saveCameraParams(const string& filename, Params param){
	if(param.distCoffs.data == NULL) param.distCoffs = Mat::zeros(1, 5, CV_64F);
	if(param.internalMat.data == NULL) param.internalMat = Mat::zeros(3,3,CV_64F);
	if(param.perspectiveMat.data == NULL) param.perspectiveMat = Mat::zeros(3,4,CV_64F);
	if(param.rotationMat.data == NULL) param.rotationMat = Mat::zeros(3,3,CV_64F);
	if(param.translationalMat.data == NULL) param.translationalMat = Mat::zeros(3,1,CV_64F);


	FileStorage fs(filename, FileStorage::WRITE);
	
	write(fs, "image_width", param.imageSize.width);
	write(fs, "image_height", param.imageSize.height);
	write(fs, "alpha", param.alpha);
	write(fs, "beta", param.beta);

	fs << "internal_matrix" << param.internalMat;
	fs << "perspective_matrix" << param.perspectiveMat;
	fs << "translational_matrix" << param.translationalMat;
	fs << "rotation_matrix" << param.rotationMat;
	fs << "distortion_coefficients" << param.distCoffs;
}
