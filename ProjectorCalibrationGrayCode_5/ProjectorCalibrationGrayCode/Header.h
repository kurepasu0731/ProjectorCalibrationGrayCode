#ifndef HEADER_H
#define HEADER_H

#pragma once

#include <Windows.h>
#include <opencv2\opencv.hpp>


// OpenCV�̃o�[�W�������擾����pragma���Ń��C�u������ǂݍ���
#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

#ifdef _DEBUG
#define CV_EXT_STR "d.lib"
#else _REREASE
#define CV_EXT_STR ".lib"
#endif

#pragma comment(lib,"opencv_core" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_imgproc" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_highgui" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_video" CV_VERSION_STR CV_EXT_STR)
#pragma comment(lib,"opencv_features2d" CV_VERSION_STR CV_EXT_STR)


#define PROJECTOR_WIDTH 1280
#define PROJECTOR_HEIGHT 800
//#define CAMERA_WIDTH 1600
//#define CAMERA_HEIGHT 1200
#define CAMERA_WIDTH 1920
#define CAMERA_HEIGHT 1080
#define DISPLAY_NUMBER 2
#define DISPLAY_WIDTH 1680
#define DISPLAY_HEIGHT 0

//using namespace std;

namespace Projection{

	typedef struct disp_prop{
		int index;
		int x,y,width,height;
	} Disp_Prop;

	static int dispCount=-1;
	static std::vector<Disp_Prop> Disps_Prop;

	//�f�B�X�v���C�̏�����
	inline BOOL CALLBACK DispEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData ) {
		Disp_Prop di;
		di.index = dispCount++;
		di.x = lprcMonitor->left;
		di.y = lprcMonitor->top;
		di.width = lprcMonitor->right - di.x;
		di.height = lprcMonitor->bottom - di.y;
		Disps_Prop.push_back(di);

		return TRUE; // TRUE�͒T���p���CFALSE�ŏI��
	}

	//�f�B�X�v���C���o
	inline void SearchDisplay(void) {
		// ��x�������s����
		if (dispCount == -1) {
			dispCount = 0;
			Disps_Prop = std::vector<Disp_Prop>();
			EnumDisplayMonitors(NULL, NULL, DispEnumProc, 0);
			Sleep(200);
		}
	}

	//�v���W�F�N�V����
	inline void MySetFullScrean(const int num, const char *windowname){

		HWND windowHandle = ::FindWindowA(NULL, windowname);

		SearchDisplay();

		if (NULL != windowHandle) {

			//-�E�B���h�E�X�^�C���ύX�i���j���[�o�[�Ȃ��A�őO�ʁj-
			SetWindowLongPtr(windowHandle,  GWL_STYLE, WS_POPUP);
			SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

			//-�ő剻����-
			ShowWindow(windowHandle, SW_MAXIMIZE);
			cv::setWindowProperty(windowname, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN );

			//-�f�B�X�v���C���w��-
			Disp_Prop di = Disps_Prop[num];

			//-�N���C�A���g�̈���f�B�X�v���[�ɍ��킹��-
			SetWindowPos(windowHandle, NULL, di.x, di.y, di.width, di.height, SWP_FRAMECHANGED | SWP_NOZORDER);
		}
	}
}

#endif