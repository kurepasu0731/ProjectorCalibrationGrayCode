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
	cv::Mat rawBuffer; //0~8000��depth�f�[�^
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

	//�[�x�������ɕϊ�����p�����[�^
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

	int image_idx; //�B�e����
	std::string outdir; //�ۑ��t�H���_


	//**�������֘A**/
	//Depth�̈ړ����ϖ@�ɂ�鐸�x����
    int nFrame;      // �ړ����ς��s���t���[���� 30�Ȃ�30FPS�Ȃ̂łP�b��  100�t���[���ʂ��ǍD
    double Kd; // �ړ����ς����Z���g�킸��Z�ŋ��߂邽�߂̌W��
    int Dx;          // ��ʃC���[�W�̐���������f��
    int Dy;          // ��ʃC���[�W�̐���������f��
    int dByte;         // XRGB�`���Ȃ̂łS�o�C�g
    int dPixels; // 1�t���[�����̉�f��
    int ptr;                 // �ړ����ς��s���ׂ̃f�[�^�i�[�|�C���^

	//vector<UINT16> DataIn;       // Kinect����f�v�X���擾���邽�߂̃o�b�t�@�i�P�t���[�����j[dPixels] = depthBuffer
    vector<UINT16> nDepthBuffer; // nFrame���̃f�v�X�o�b�t�@ [dPixels * nFrame]
    long *Sum; // nFrame���̈ړ����Z�l���i�[����ׂ̃o�b�t�@[dPixels]

	//���ό��ʂ�Depth
	unsigned short* aveDepthData;
	
	//�������f�[�^���邩�ǂ����̃t���O
	bool hasSmooth;

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

	//�[�x������
	void frame_smoothing();

	void FIFOFilter();


};