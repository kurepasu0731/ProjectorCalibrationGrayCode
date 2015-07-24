#ifndef GRAYCODE_H
#define GRAYCODE_H

#pragma once

// GrayCodeを用いた幾何補正

// グレイコードは，縦，横別に作成し，後で合成
// パターン画像はビット演算を用いて作成（文字列char型は使わない）
// パターン画像は1枚ずつ書き込んで保存
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>  // 文字列ストリーム
#include <direct.h> // create dir
#include <math.h>
#include <random>
#include "myKinect.h"

typedef struct {
	Size imageSize;
	Mat perspectiveMat;		//透視投影行列
	Mat internalMat;		//内部行列
	Mat translationalMat;	//並進行列
	Mat rotationMat;		//回転行列
	Mat distCoffs;			//歪み係数
	double alpha;
	double beta;
} Params;

/** 
@brief GrayCodeを用いた幾何補正<br>
対応付けられなかった画素には近傍の画素をコピーして補間<br>
最終的なプロジェクタとカメラを対応付けた配列はc->CamProに格納される。
*/
class GRAYCODE{
public:
	// プロジェクタ解像度
	static const int PRJ_WIDTH = PROJECTOR_WIDTH;
	static const int PRJ_HEIGHT = PROJECTOR_HEIGHT;
	static const int CMR_WIDTH = CAMERA_WIDTH;
	static const int CMR_HEIGHT = CAMERA_HEIGHT;
	static const int PRJ_X = DISPLAY_WIDTH;
	static const int PRJ_Y = DISPLAY_HEIGHT;

	// グレイコード作成に必要な構造体
	typedef struct _Graycode {
		int graycode[PRJ_HEIGHT][PRJ_WIDTH];  // グレイコード（プロジェクタ解像度[高さ][幅]）
		unsigned int h_bit, w_bit;     // 高さ，幅の必要ビット数
		unsigned int all_bit;          // 合計ビット数（h_bit + w_bit）
	} Graycode;

	// プロジェクタ - カメラ対応に必要な構造体
	typedef struct _correspondence {
		int graycode[CMR_HEIGHT][CMR_WIDTH];  // 2値化コードを復元したものをカメラ画素に格納
		cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH];  // プロジェクタ画素に対するカメラ対応画素
		std::map<int, cv::Point> *code_map;
		Graycode g;
	} correspondence;

	correspondence *c; //プロジェクタ-カメラ間対応

	cv::Point3f ProWorld[PRJ_HEIGHT][PRJ_WIDTH]; //プロジェクタ-3次元座標対応
	int validPointNum; //有効な対応点数
	Params pro_param; //プロジェクタパラメータ

	//全対応点
	vector<Point2d> points_pro;//プロジェクタ画像
	vector<Point3d> points_3d;//三次元座標

	//正対応点
	vector<Point2d> g_imagePointInlierSet;
	vector<Point3d> g_worldPointInlierSet;

	//最初にランダムに100点取ったやつ
	vector<Point2d> random_calib_2d_points;
	vector<Point3d> random_calib_3d_points;

	//再投影用画像
	Mat reProjectImage;
	void reProjectPoints(int radius);
	Point2f ProjectTo(Params pro_param, Point3f world_point);

		//myParam.h
	Params loadParams(const string& filename);
	void saveCameraParams(const string& filename, Params param);

//	//mySixPoints_Algolism.h
//	bool six_points_algolism(vector<Point3d>& P, vector<Point2d>& p, Mat& cameraMat);

	GRAYCODE(myKinect *k);
	~GRAYCODE();
	// パターンコード投影 & 撮影
	void code_projection();
	// 2値化
	void make_thresh();
	// 初期化
	void makeCorrespondence();
	// プロジェクタ-3次元座標間対応付けを行うインターフェース
	void makeCorrespondenceToWorld();
	//点を互換変換するインターフェース
	void GRAYCODE::PushPoints();
	//パラメータ推定開始
	void calcParameters();
	//最適化
	void OptimizeParameters();

	//// 画像変形・処理
	//// カメラ撮影領域からプロジェクタ投影領域を切り出し
	void transport_camera_projector(cv::Mat &src, cv::Mat &dst);
	//// 入力画像をカメラ撮影領域に変形
	void transport_projector_camera(cv::Mat &src, cv::Mat &dst);

	//対応点を形式で保存
	void save_g_worldPointInlierSet();
	void load_g_worldPointInlierSet();

private:
	// ウィンドウネーム
	char* GC;
	char* MP;
	float SHUTTER;	// シャッター速度
	double delay;

	Graycode *g;

	myKinect *kinect;

	/// グレイコードの作成関連
	// カメラの初期化
	void initCamera();
	// グレイコード作成
	void initGraycode();
	// パターンコード画像作成
	void makeGraycodeImage();
	// ディレクトリの作成
	void createDirs();
	/// 二値化関連
	// カメラ撮影画像を読み込む関数
	void loadCam(cv::Mat &mat, int div_bin, bool flag, bool pattern);
	// 最終的に使用するマスクを生成する関数
	void makeMask(cv::Mat &mask);
	// グレイコードの画像を利用してマスクを生成する関数
	// ポジとネガの差分を取ってMASK_THRESH以上の輝度のピクセルを白にする
	void makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue = 25);
	// 2値化処理関数 
	void thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value );

	//パラメータ推定関連
	//6 points algorithm
	void six_points_calibration(vector<Point3d>& points_3d, vector<Point2d>& points_2d, Mat& dst);
	//RANSACバージョン
	void six_points_calibration_2(vector<Point3d>& points_3d, vector<Point2d>& points_2d, Mat& dst);
	//ランダムに100点を抽出
	void get_hundred_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P);
	//互換変換
	void changePoints(vector<Point2d>& points_pro, vector<Point3d>& points_3d);
	//ランダムに6点を抽出
	void get_six_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P);
	//元画像の特徴点と、再計算した特徴点の誤差を求める
	double inspection_error_value(Mat& cameraMat, vector<Point3d>& P, Size& imageSize, vector<Point2d>& groundTruth, bool isDraw);
	//平面フィッティング
	double PlaneFit(vector<Point3d>& points, Mat& X, double* D);

	/// その他
	// プロジェクタ - カメラ構造体初期化
	void initCorrespondence();
	// 2値化コード復元
	void code_restore();
	//// 画素補間手法
	// 隣接する画素から持ってくる
	cv::Point getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH]);
	// 画素補間
	void interpolation();
};


#endif