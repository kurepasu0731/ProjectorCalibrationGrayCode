#ifndef SQUARE_H
#define SQUARE_H

//#include "main.h"
#include <cmath>
#include <random>
#include <fstream>
#include <filesystem>

#include <opencv2\opencv.hpp>

/**
@brief 幾何補正用の画像を作成するクラス
*/
class SQUARE{
public:
	SQUARE(std::string windowName = "内接リサイズ")
	{
		this->windowName = windowName;
		pixels = 0;
		debug = false;
	}

	~SQUARE()
	{
	}

	std::string windowName;
	cv::Mat mask;	//!< 使用するマスク画像（setMaskで読み込む）
	int Xa, Ya;	//!< 始点の座標
	int Xb, Yb;	//!< 終点の座標

	/// 整数に丸める
	int round(double d)
	{
		return (int)(d+0.5);
	}

	// マスク画像を読み込む
	void setMask(std::string filename, double showSizeRate = 0.5);
	// ウィンドウ名を設定
	void setWindowName(std::string windowName);
	// 自動的に座標を決定する
	int autoSelectSquare(int minPixels, cv::Size size, cv::Point shift = cv::Point(0, 0), double precision = 0.005, bool savePoint = true);
	void randomSelectSquare(cv::Size size, double precision = 0.005, int num = 100);
	// 結果画像を表示
	void drawResult(bool savePoint = true);
	// カーソルキーで矩形をの位置を調節する
	void adjustSquare(cv::Size size, std::string searchDir = ".", std::string saveDir = ".");
	// ファイルから座標を読み込む
	bool loadData();
	// 画像を保存する
	void saveImage(cv::Mat &src, std::string saveName);
	// マウスのコールバック関数
	void onMouse( int event, int x, int y, int flag, void* param);
	// リサイズした画像を位置合わせ＋マスク適用（結果確認用）
	void imageShiftAdaptMask(cv::Mat &src, cv::Mat &dst, cv::Point center = cv::Point(0, 0));
	// リサイズした画像を位置合わせ
	void imageShift(cv::Mat &src, cv::Mat &dst, cv::Point center = cv::Point(0, 0));
	// 指定した矩形のサイズにリサイズ
	cv::Point resize(cv::Mat &src, cv::Mat &dst, cv::Point start, cv::Point end);

private:
	double showSizeRate;	//!< マスク画像をディスプレイに表示するときのサイズの割合
	int orgWidth, orgHeight;	//!< 読み込み時のマスク画像のサイズ（リサイズしてしまうので値を保存しておく）
	int pixels;	//!< 矩形内のピクセル数
	bool debug;	//!< デバッグ用画像の表示

	// 下位ディレクトリ内の全ファイルの名前を取得
	std::vector<std::string> getAllFilename(std::string searchDir);
};

#endif