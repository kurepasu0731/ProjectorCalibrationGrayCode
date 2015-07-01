#ifndef HEADER_H
#define HEADER_H

#pragma once

#include <Windows.h>
#include <opencv2\opencv.hpp>


// OpenCVのバージョンを取得してpragma文でライブラリを読み込む
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

	//ディスプレイの情報入手
	inline BOOL CALLBACK DispEnumProc(HMONITOR hMonitor, HDC hdcMonitor, LPRECT lprcMonitor, LPARAM dwData ) {
		Disp_Prop di;
		di.index = dispCount++;
		di.x = lprcMonitor->left;
		di.y = lprcMonitor->top;
		di.width = lprcMonitor->right - di.x;
		di.height = lprcMonitor->bottom - di.y;
		Disps_Prop.push_back(di);

		return TRUE; // TRUEは探索継続，FALSEで終了
	}

	//ディスプレイ検出
	inline void SearchDisplay(void) {
		// 一度だけ実行する
		if (dispCount == -1) {
			dispCount = 0;
			Disps_Prop = std::vector<Disp_Prop>();
			EnumDisplayMonitors(NULL, NULL, DispEnumProc, 0);
			Sleep(200);
		}
	}

	//プロジェクション
	inline void MySetFullScrean(const int num, const char *windowname){

		HWND windowHandle = ::FindWindowA(NULL, windowname);

		SearchDisplay();

		if (NULL != windowHandle) {

			//-ウィンドウスタイル変更（メニューバーなし、最前面）-
			SetWindowLongPtr(windowHandle,  GWL_STYLE, WS_POPUP);
			SetWindowLongPtr(windowHandle, GWL_EXSTYLE, WS_EX_TOPMOST);

			//-最大化する-
			ShowWindow(windowHandle, SW_MAXIMIZE);
			cv::setWindowProperty(windowname, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN );

			//-ディスプレイを指定-
			Disp_Prop di = Disps_Prop[num];

			//-クライアント領域をディスプレーに合わせる-
			SetWindowPos(windowHandle, NULL, di.x, di.y, di.width, di.height, SWP_FRAMECHANGED | SWP_NOZORDER);
		}
	}
}

#endif