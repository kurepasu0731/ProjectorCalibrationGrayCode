#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <stdint.h>
#include <memory>
#include <fstream>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <kinect.h>

#include "ComPtr.h"
#include "Header.h"


using namespace cv;
using namespace std;

#define ERROR_CHECK( ret )  \
	if ( (ret) != S_OK ) {    \
	std::stringstream ss;	\
	ss << "failed " #ret " " << std::hex << ret << std::endl;			\
	throw std::runtime_error( ss.str().c_str() );			\
	}

class myKinect
{

public:
	myKinect();
	~myKinect();
	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;
	int infraredWidth;
	int infraredHeight;

	IKinectSensor* kinect;
	IColorFrameReader* colorFrameReader;
	unsigned int colorBytesPerPixel;
	vector<BYTE> colorBuffer;
	unsigned int bufferSize; 
	cv::Mat rawBuffer; //0~8000のdepthデータ
	IDepthFrameReader* depthFrameReader;
	vector<UINT16> depthBuffer;
	IInfraredFrameReader* infraredFrameReader;
	vector<UINT16> infraredBuffer;

	int minDepth;
	int maxDepth;
	UINT16 minDepthReliableDistance;
	UINT16 maxDepthReliableDistance;

	ICoordinateMapper *coordinateMapper;
	vector<ColorSpacePoint> colorPoints;
	vector<DepthSpacePoint> depthPoints;
	vector<CameraSpacePoint> cameraPoints;

	CameraIntrinsics depthcameraIntrinsics;

	//深度→視差に変換するパラメータ
	double dc1;
	double dc2;

	Mat colorImage;
	Mat colorImage_half;
	Mat depthImage;
	Mat infraredImage;
	Mat coordinatedImage; //MapDepthFrameToColorSpace
	Mat coordinatedImage2; //MapColorFrameToDepthSpace

	std::string colorWinName;
	std::string depthWinName;
	std::string infraredWinName;
	std::string coordinatedWinName;
	std::string coordinatedWinName2;

	int image_idx; //撮影枚数
	std::string outdir; //保存フォルダ

	void init();
	void initColorFrame();
	void initDepthFrame();
	void initIRFrame();

	void updateColorFrame();
	void updateDepthFrame();
	void updateIRFrame();

	void multiUpdateColorFrame_in(myKinect* kinect);

	void coordinateColorDepth();
	void coordinateDepthColor();

	void coordinateColorToCamera(Point camPro[PROJECTOR_HEIGHT][PROJECTOR_WIDTH], Point3f proWorld[PROJECTOR_HEIGHT][PROJECTOR_WIDTH], int* pointnum);

	bool isValidColorFrameRange(float x, float y);
	bool isValidDepthFrameRange(float x, float y);
	bool isValidDepthRange(int index);

	void draw();

	void swap_endiannes(uint16_t *out,const uint16_t *in, int size);
	void saveDepth_PMG();
	void saveDepth_JPG();
	void saveColor();
	void saveIR();

	double getRawDepthValue(UINT16 depthmillimeter);

	void getDepthCameraIntrinsics(CameraIntrinsics cameraIntrinsics);

};