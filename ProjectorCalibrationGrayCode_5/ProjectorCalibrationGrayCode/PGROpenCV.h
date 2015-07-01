#ifndef PGROPENCV_H
#define PGROPENCV_H

#pragma once

// Flycapture2のライブラリを読込む
#pragma once
#ifdef _DEBUG
#pragma comment(lib,"FlyCapture2d.lib")
#pragma comment(lib,"FlyCapture2GUId.lib")
#else _REREASE
#pragma comment(lib,"FlyCapture2.lib")
#pragma comment(lib,"FlyCapture2GUI.lib")
#endif

#include <FlyCapture2.h>
#include <opencv2\opencv.hpp>
#include <cstdio>
#include <iostream>

class TPGROpenCV
{
private:
	FlyCapture2::Error			fc2Error;
	FlyCapture2::BusManager		fc2BusMgr;
	FlyCapture2::PGRGuid		fc2Guid;
	FlyCapture2::CameraInfo		fc2CamInfo;
	FlyCapture2::Camera			fc2Cam;
	FlyCapture2::Property		fc2Prop;
	FlyCapture2::PixelFormat	fc2PixelFormat;

	FlyCapture2::ColorProcessingAlgorithm fc2CPA;
	unsigned int				numCameras;

	float						Shutter_high;
	float						Shutter_middle1;
	float						Shutter_middle2;
	float						Shutter_middle3;
	float						Shutter_low;
	float						Shutter_LC; //補正中のシャッタースピード
	float						Gain;
	float						Framerate;
	unsigned int				Wb_Red;
	unsigned int				Wb_Blue;
	cv::Mat						fc2Mat;

public:
	TPGROpenCV();
	~TPGROpenCV();
	int init(  FlyCapture2::PixelFormat _format = FlyCapture2::PIXEL_FORMAT_BGR, int ColorProcessingAlgorithm = FlyCapture2::ColorProcessingAlgorithm::HQ_LINEAR );
	void PrintBuildInfo();
	void PrintError( FlyCapture2::Error error );
	void PrintCameraInfo( FlyCapture2::CameraInfo* pCamInfo );
	void InitCameraParameter();
	int PixelFormatInOpenCV();
	int start();
	int queryFrame();
	int stop();
	int release();
	std::string windowNameCamera;

	float getShutter_h(){ return Shutter_high; };
	float getShutter_m1(){ return Shutter_middle1; };
	float getShutter_m2(){ return Shutter_middle2; };
	float getShutter_m3(){ return Shutter_middle3; };
	float getShutter_l(){ return Shutter_low; };
	float getShutter_LC(){ return Shutter_LC; };

	void setShutterSpeed(float shutterSpeed);
	void setGain(float gain);
	void setWhiteBalance(int r, int b);
	void setPixelFormat( FlyCapture2::PixelFormat format );
	void setColorProcessingAlgorithm( FlyCapture2::ColorProcessingAlgorithm algorithm );

	float getShutterSpeed();
	float getGain();
	void getWhiteBalance(int &r, int &b);

	void showCapImg(cv::Mat cap = cv::Mat());
	cv::Mat getVideo(){ return fc2Mat; };
};

#endif