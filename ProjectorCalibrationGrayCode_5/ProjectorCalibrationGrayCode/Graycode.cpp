#pragma once

#include "Graycode.h"
#include "myGauss_Newton.h"
//#include "myParam.h"
#include "mySix_Points_Algolism.h"


GRAYCODE::GRAYCODE(myKinect *k)
{
	kinect = k;
	GC = "Graycode";
	MP = "Measure";
	delay = 200;
	g = new Graycode();
	c = new correspondence();
	validPointNum = 0;
	c->code_map = new std::map<int, cv::Point>();
	// 構造体の初期化
	c->g.h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	c->g.w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	c->g.all_bit = c->g.h_bit + c->g.w_bit;
	createDirs();
}

GRAYCODE::~GRAYCODE()
{
}

void GRAYCODE::createDirs()
{
	_mkdir("./GrayCodeImage");
	// グレイコード撮影画像
	_mkdir("./GrayCodeImage/CaptureImage");
	// グレイコード生画像
	_mkdir("./GrayCodeImage/ProjectionGrayCode");
	// グレイコード撮影画像の二値化した画像
	_mkdir("./GrayCodeImage/ThresholdImage");
}

/***************************
** グレイコードの作成関連 **
****************************/

// ビット数の計算とグレイコードの作成
void GRAYCODE::initGraycode()
{
	int bin_code_h[PRJ_HEIGHT];  // 2進コード（縦）
	int bin_code_w[PRJ_WIDTH];   // 2進コード（横）
	int graycode_h[PRJ_HEIGHT];  // グレイコード（縦）
	int graycode_w[PRJ_WIDTH];   // グレイコード（横）
	//int *graycode_h =  new int[c->g.h_bit];  // グレイコード（縦）
	//int *graycode_w =  new int[c->g.w_bit];  // グレイコード（横）

	/***** 2進コード作成 *****/
	// 行について
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		bin_code_h[y] = y + 1;
	// 列について
	for( int x = 0; x < PRJ_WIDTH; x++ )
		bin_code_w[x] = x + 1;

	/***** グレイコード作成 *****/
	// 行について
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		graycode_h[y] = bin_code_h[y] ^ ( bin_code_h[y] >> 1 );
	// 列について
	for( int x = 0; x < PRJ_WIDTH; x++ )
		graycode_w[x] = bin_code_w[x] ^ ( bin_code_w[x] >> 1 );
	// 行列を合わせる（行 + 列）
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ )
			c->g.graycode[y][x] = ( graycode_h[y] << c->g.w_bit) | graycode_w[x];
	}
}

// パターンコード画像作成（一度作ればプロジェクタの解像度が変わらない限り作り直す必要はない）
void GRAYCODE::makeGraycodeImage()
{
	std::cout << "投影用グレイコード作成中" << std::endl;
	//initGraycode();
	cv::Mat posi_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	cv::Mat nega_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	int bit = c->g.all_bit-1;
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit];  // 書式付入出力
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];  // 書式付入出力

	// ポジパターンコード画像作成
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {  // 最上位ビットから順に抽出し，そのビットが0だった時
					posi_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					posi_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					posi_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					posi_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// 連番でファイル名を保存（文字列ストリーム）
		Filename_posi[z] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_posi[z].str(), posi_img);
		Filename_posi[z] << std::endl;
	}

	// ネガパターンコード画像作成
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 1 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 0;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 0;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 0;  // R
				}else if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {
					nega_img.at<cv::Vec3b>( y, x )[0] = 255;  // B
					nega_img.at<cv::Vec3b>( y, x )[1] = 255;  // G
					nega_img.at<cv::Vec3b>( y, x )[2] = 255;  // R
				}
			}
		}
		// 連番でファイル名を保存（文字列ストリーム）
		Filename_nega[z] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_nega[z].str(), nega_img);
		Filename_nega[z] << std::endl;
	}

	delete[] Filename_posi;
	delete[] Filename_nega;
}

// パターンコード投影 & 撮影
void GRAYCODE::code_projection()
{
	// 定数
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	Graycode *g = new Graycode();
	//TPGROpenCV	pgrOpenCV;

	//初期化&カメラ起動
	initGraycode();
	////pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR );
	//pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	//pgrOpenCV.start();

	cv::Mat *posi_img = new cv::Mat[c->g.all_bit];  // ポジパターン用
	cv::Mat *nega_img = new cv::Mat[c->g.all_bit];  // ネガパターン用

	// 書式付入出力（グレイコード読み込み用）
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];
	// 書式付入出力（撮影画像書き込み用）
	std::stringstream *Filename_posi_cam = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega_cam = new std::stringstream[c->g.all_bit];

	// 連番でファイル名を読み込む（文字列ストリーム）
	std::cout << "投影用グレイコード画像読み込み中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		Filename_posi[i] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		// 読み込み
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;
		// 読み込む枚数が足りなかったらグレイコード画像を作り直す
		if(posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)：投影用のグレイコード画像が不足しています。" << std::endl;
			std::cout << "ERROR(2)：グレイコード画像を作成します。" << std::endl;
			makeGraycodeImage();
			code_projection();
			return;
		}
	}

	/***** グレイコード投影 & 撮影 *****/
	/*  全画面表示用ウィンドウの作成  */
	cv::namedWindow(GC, 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, GC);

	// ポジパターン投影 & 撮影
	//start capturing

	std::cout << "ポジパターン撮影中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// 投影
		cv::imshow(GC, posi_img[i]);
		// 遅延待ち
		cv::waitKey(2.0*delay);
		// 撮影
		//pgrOpenCV.queryFrame();
		kinect->updateColorFrame();
		// 撮影画像をMat型に格納
		//cv::Mat cap = pgrOpenCV.getVideo();
		cv::Mat cap = kinect->colorImage;
		//kinectだから左右反転
		flip(cap, cap, 1);
		// 撮影の様子をチェック
		//pgrOpenCV.showCapImg(cap);
		imshow("colorImage", cap);

		// ポジパターン撮影結果を保存
		// 横縞
		if(i < c->g.h_bit)
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << POSI << ".bmp"; 
		// 縦縞
		else
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << POSI << ".bmp"; 
		// 保存
		//cv::imwrite(Filename_posi_cam[i].str(), pgrOpenCV.getVideo());
		cv::imwrite(Filename_posi_cam[i].str(), cap);
		Filename_posi_cam[i] << std::endl;
	}

	// ネガパターン投影 & 撮影
	std::cout << "ネガパターン撮影中" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// 投影
		cv::imshow(GC, nega_img[i]);
		// 遅延待ち
		cv::waitKey(2*delay);
		// 撮影
		//pgrOpenCV.queryFrame();
		kinect->updateColorFrame();
		// 撮影画像をMat型に格納
		//cv::Mat cap = pgrOpenCV.getVideo();
		cv::Mat cap = kinect->colorImage;
		//kinectだから左右反転
		flip(cap, cap, 1);
		// 撮影の様子をチェック
		//pgrOpenCV.showCapImg(cap);
		imshow("colorImage", cap);
		// ポジパターン撮影結果を保持
		// 横縞
		if(i < c->g.h_bit)
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << NEGA << ".bmp"; 
		// 縦縞
		else
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << NEGA << ".bmp"; 
		////Filename_nega_cam[i] << "./output/Camera_nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		//cv::imwrite(Filename_nega_cam[i].str(), pgrOpenCV.getVideo());
		cv::imwrite(Filename_nega_cam[i].str(), kinect->colorImage);
		Filename_nega_cam[i] << std::endl;
	}
	/***** 投影 & 撮影終了 *****/


	cv::Mat src = cv::imread("./Penguins.jpg",1);
	cv::imshow(GC, src);
	cv::waitKey(2*delay);
	//pgrOpenCV.queryFrame();
	kinect->updateColorFrame();
	//cv::imwrite("./cap.jpg", pgrOpenCV.getVideo());
	cv::imwrite("./cap.jpg", kinect->colorImage);

	// カメラ終了処理
	//pgrOpenCV.stop();

	/**** 終了 *****/

	// メモリの開放
	delete[] posi_img;
	delete[] nega_img;
	delete[] Filename_posi;
	delete[] Filename_nega;
	delete[] Filename_posi_cam;
	delete[] Filename_nega_cam;
}


/***************
** 二値化関連 **
****************/

// カメラ撮影画像を読み込む関数
void GRAYCODE::loadCam(cv::Mat &mat, int div_bin, bool vh, bool pn)
{
	char buf[256];
	sprintf_s(buf, "./GrayCodeImage/CaptureImage/CameraImg%d_%02d_%d.bmp", vh, div_bin, pn);
	mat = cv::imread(buf, 0);
}

// マスクを作成するインタフェース
void GRAYCODE::makeMask(cv::Mat &mask)
{
	cv::Mat posi_img;
	cv::Mat nega_img;

	// マスク画像生成
	cv::Mat mask_vert, mask_hor;
	static int useImageNumber = 6;
	// y方向のグレイコード画像読み込み
	loadCam(posi_img, useImageNumber, 0, 1);
	loadCam(nega_img, useImageNumber, 0, 0);

	// 仮のマスク画像Y生成
	makeMaskFromCam(posi_img, nega_img, mask_vert);

	// x方向のグレイコード画像読み込み
	loadCam(posi_img, useImageNumber, 1, 1);
	loadCam(nega_img, useImageNumber, 1, 0);

	// 仮のマスク画像X生成
	makeMaskFromCam(posi_img, nega_img, mask_hor);

	// XとYのORを取る
	// マスク外はどちらも黒なので黒
	// マスク内は（理論的には）必ず一方が白でもう一方が黒なので、白になる
	// 実際はごま塩ノイズが残ってしまう
	cv::bitwise_or(mask_vert, mask_hor, mask);

	// 残ったごま塩ノイズを除去（白ゴマか黒ゴマかで適用順が逆になる）
	dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

	cv::imwrite("./GrayCodeImage/mask.bmp", mask);
}

// グレイコードの画像を利用してマスクを生成する関数
// ポジとネガの差分を取ってthresholdValue以上の輝度のピクセルを白にする
void GRAYCODE::makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue)
{
	result = cv::Mat::zeros(cv::Size(CMR_WIDTH,CMR_HEIGHT), CV_8UC1);

	for(int j=0; j<CMR_HEIGHT; j++){
		for(int i=0; i<CMR_WIDTH; i++){
			int posi_i = posi.at<uchar>(j, i);
			int nega_i = nega.at<uchar>(j, i);

			if (abs(posi_i - nega_i) > thresholdValue){
				result.at<uchar>(j, i) = 255;
			}else{
				result.at<uchar>(j, i) = 0;
			}
		}
	}
}

// 撮影画像の2値化をするインタフェース
void GRAYCODE::make_thresh()
{
	cv::Mat posi_img;
	cv::Mat nega_img;
	cv::Mat Geometric_thresh_img;  // 2値化された画像
	cv::Mat mask;

	// マスクを生成
	makeMask(mask);

	int h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	int w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	int all_bit = h_bit + w_bit;

	std::cout << "二値化開始" << std::endl;
	// 連番でファイル名を読み込む
	for( int i = 0; i < h_bit; i++ ) {
		// 読み込み
		char buf[256];
		// ポジパターン読み込み
		loadCam(posi_img, i+1, 0, 1);
		// ネガパターン読み込み
		loadCam(nega_img, i+1, 0, 0);

		// 2値化
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// マスクを適用して2値化
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::imwrite(buf, masked_img);

		std::cout << i << ", ";
	}
	for( int i = 0; i < w_bit; i++ ) {
		// 読み込み
		char buf[256];
		// ポジパターン読み込み
		loadCam(posi_img, i+1, 1, 1);
		// ネガパターン読み込み
		loadCam(nega_img, i+1, 1, 0);

		// 2値化
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// マスクを適用して2値化
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i+h_bit);
		cv::imwrite(buf, masked_img);

		std::cout << i+h_bit << ", ";
	}
	std::cout << std::endl;
	std::cout << "二値化終了" << std::endl;
}

// 実際の2値化処理 
void GRAYCODE::thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value )
{
	thresh_img = cv::Mat(posi.rows, posi.cols, CV_8UC1);
	for( int y = 0; y < posi.rows; y++ ) {
		for(int x = 0; x < posi.cols; x++ ) {
			int posi_pixel = posi.at<uchar>( y, x );
			int nega_pixel = nega.at<uchar>( y, x );

			// thresh_valueより大きいかどうかで二値化
			if( posi_pixel - nega_pixel >= thresh_value )
				thresh_img.at<uchar>( y, x ) = 255;
			else
				thresh_img.at<uchar>( y, x ) = 0;
		}
	}
}

/***********************************
** プロジェクタとカメラの対応付け **
************************************/

// 2値化コード復元
void GRAYCODE::code_restore()
{
	// 2値化コード復元
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		char buf[256];
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::Mat a = cv::imread(buf, 0);

		for( int y = 0; y < CMR_HEIGHT; y++ ) {
			for( int x = 0; x < CMR_WIDTH; x++ ) {
				if( a.at<uchar>( y, x ) == 255)
					c->graycode[y][x] = ( 1 << (c->g.all_bit-i-1) ) | c->graycode[y][x]; 
			}
		}
	}

	// 連想配列でグレイコードの値の場所に座標を格納
	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ) {
			int a = c->graycode[y][x];
			if( a != 0 )
				(*c->code_map)[a] = cv::Point(x, y);
		}
	}

	// 0番目は使わない
	(*c->code_map)[0] = cv::Point(-1, -1);

	// プロジェクタとカメラの対応付け
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			// グレイコード取得
			int a = c->g.graycode[y][x];
			// map内に存在しないコード（カメラで撮影が上手くいかなかった部分）の場所にはエラー値-1を格納
			if ( (*c->code_map).find(a) == (*c->code_map).end() ) {
				c->CamPro[y][x] = cv::Point(-1, -1);
			}
			// 存在する場合は、対応するグレイコードの座標を格納
			else {
				c->CamPro[y][x] = (*c->code_map)[a];
			}
		}
	}
}

// 隣接する画素から持ってくる
cv::Point GRAYCODE::getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH])
{
	const int MAX_RADIUS = 100;

	for (int radius = 1; radius <= MAX_RADIUS; radius++){

		for (int j = -radius; j <= radius; j++) {
			for (int i = -radius; i <= radius; i++) {
				int yj = j + y;
				int xi = i + x;

				if (0 <= yj && yj < PRJ_HEIGHT && 0 <= xi && xi < PRJ_WIDTH) {
					if ((yj > y) || (yj == y && xi > x)) {
						if (c->CamPro[yj][xi].x != -1) {
							return c->CamPro[yj][xi];
						}
					}
				}
			}
		}

	}

	return cv::Point(-1, -1);
}

// 画素補間
void GRAYCODE::interpolation()
{
	for (int y = 0; y < PRJ_HEIGHT; y++) {
		for (int x = 0; x < PRJ_WIDTH; x++) {
			if (c->CamPro[y][x].x == -1) {
				c->CamPro[y][x] = getInterpolatedPoint2(y, x, c->CamPro);
			}
		}
	}
}

// プロジェクタ - カメラ構造体初期化
void GRAYCODE::initCorrespondence()
{
	initGraycode();

	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ){
			c->graycode[y][x] = 0;
		}
	}
}

// 対応付けを行うインターフェース
void GRAYCODE::makeCorrespondence()
{
	initCorrespondence();
	code_restore();
	//	補完処理on off
	//interpolation();
}


// プロジェクタ-3次元座標間対応付けを行う
void GRAYCODE::makeCorrespondenceToWorld()
{
	//最新のフレーム情報を使ってCoordinateMapper
	kinect->updateColorFrame();
	kinect->updateDepthFrame();
	kinect->coordinateColorToCamera(c->CamPro, ProWorld, &validPointNum);

	//デバッグ用
	cout << "対応点取得終了...\n" << endl;
	cout << "総点数：" << validPointNum << "点\n" << endl;
}

//対応点を用いてパラメータ推定(6Points->最適化)
void GRAYCODE::calcParameters(){

	//キャリブレーション(6 points)
	std::cout << "*---------------------------------------------*" << std::endl;
	std::cout << "*       calibration by 6 points algrithm      *" << std::endl;

//	//互換変換
//	changePoints(points_pro, points_3d);

	//six_points_calibration(points_3d, points_pro, pro_param.perspectiveMat);
	six_points_calibration_2(points_3d, points_pro, pro_param.perspectiveMat);

	//Matのサイズが取れているか
	std::cout << "perspectiveMat = " << pro_param.perspectiveMat.size() << std::endl;

	//結果の表示
	std::cout << "*                result = " << 
		inspection_error_value(pro_param.perspectiveMat, points_3d, Size(800, 600), points_pro, false)
		<<std::endl; //Kinectの色画像サイズ：1920×1080

	//結果の保存
	pro_param.alpha = 1.0;
	pro_param.beta = 0.0;
	saveCameraParams("../projector_paramater_6points.xml", pro_param);
	std::cout << "***********************************************" << std::endl;

}

void GRAYCODE::OptimizeParameters()
{
	//キャリブレーション（パラメータ数12の最適化）
	std::cout << "*---------------------------------------------*" << std::endl;
	std::cout << "*         calibration by optimization         *" << std::endl;
	//↓対応点全部突っ込むとすごく時間が掛かるので、RANSAC的な感じで削減する必要あり
//	getper::get_perspectiveMat(points_pro, points_3d, pro_param.perspectiveMat, pro_param.perspectiveMat);
	getper::get_perspectiveMat(g_imagePointInlierSet, g_worldPointInlierSet, pro_param.perspectiveMat, pro_param.perspectiveMat);
	//結果の表示
	std::cout << "*                result = " << 
//		inspection_error_value(pro_param.perspectiveMat, points_3d, Size(800, 600), points_pro, false)
		inspection_error_value(pro_param.perspectiveMat, g_worldPointInlierSet, Size(800, 600), g_imagePointInlierSet, false)
		<<std::endl;
	//結果の保存
	pro_param.alpha = 1.0;
	pro_param.beta = 0.0;
	saveCameraParams("../projector_paramater_optimize.xml", pro_param);
	std::cout << "***********************************************" << std::endl;
}

/***********************************
*** 6Points関連************
************************************/

//互換変換するインターフェース
void GRAYCODE::PushPoints()
{
	validPointNum = 0;//初期化
	//互換変換と配列クリア
	changePoints(points_pro, points_3d);
}

//互換変換
void GRAYCODE::changePoints(vector<Point2d>& points_pro, vector<Point3d>& points_3d)
{
	for(int y = 0; y < PRJ_HEIGHT; y++)
	{
		for(int x = 0; x < PRJ_WIDTH; x++)
		{
			if(ProWorld[y][x].z != -1){
				Point2d point2d(x, y);
				Point3d point3d(ProWorld[y][x].x, ProWorld[y][x].y, ProWorld[y][x].z);

				points_pro.push_back(point2d);
				points_3d.push_back(point3d);

				//点をプッシュしたのでクリア
				c->CamPro[y][x].x = 0;
				c->CamPro[y][x].y = 0;

				ProWorld[y][x].x = 0.0f;
				ProWorld[y][x].y = 0.0f;
				ProWorld[y][x].z = 0.0f;

			}
		}
	}
}

//6 points algorithm
void GRAYCODE::six_points_calibration(vector<Point3d>& points_3d, vector<Point2d>& points_2d, Mat& dst){
	double min_ave=100000;

	for(int i=0; i<10; i++){
		vector<Point2d> calib_2d_points;
		vector<Point3d> calib_3d_points;
		Mat pm;
		double ave;
		//points_3dに値が正しく入っていない
		//std::cout << "points_3d = " << points_3d << std::endl;

		get_six_points(calib_2d_points, calib_3d_points, points_2d, points_3d);
		//std::cout << "calib_2d_points = " << calib_2d_points << std::endl;
		//calib_3d_pointsに値が正しく入っていない
		//std::cout << "calib_3d_points = " << calib_3d_points << std::endl;


		six_points_algolism(calib_3d_points, calib_2d_points, pm);
		//std::cout << "pmの値：" << pm.at<double>(0,0) << std::endl;
		//std::cout << "pmのサイズ：" << pm.size() << std::endl;
		ave = inspection_error_value(pm, points_3d, Size(512, 424), points_2d, false);
		//std::cout << "ave = " << ave << std::endl;
		if(ave < min_ave){
			dst = pm;
			//std::cout << "dst.size = " << dst.size() << std::endl;
		}
	}
}

//ランダムに6点を抽出
void GRAYCODE::get_six_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P){
	int i=0;
	srand(time(NULL));    /* 乱数の初期化 */ //rand()<32767
	Vector<int> exists;
	while(i <= 100){
		int maxValue = (int)src_p.size();
		int random = rand();
		int v = rand() % maxValue;
		bool e2=false;
		for(int s=0; s<i; s++){
			if(exists[s] == v) e2 = true; 
		}
		if(!e2){
			exists.push_back(v);
			calib_P.push_back(src_P[v]);
			calib_p.push_back(src_p[v]);
			i++;
		}
	}
}

//元画像の特徴点と、再計算した特徴点の誤差を求める
double GRAYCODE::inspection_error_value(Mat& cameraMat, vector<Point3d>& P, Size& imageSize, vector<Point2d>& groundTruth, bool isDraw){
	Mat image(imageSize, CV_8UC3, Scalar::all(255));
	//std::cout << cameraMat.cols << ", " << cameraMat.rows << std::endl;
	if(cameraMat.cols != 4 || cameraMat.rows != 3){
		std::cout << "error : camera Matrix is not 3x4 Matrix" << std::endl;
		return 0.0;
	}
	vector<Point2d> p;
	for(int i=0; i<(int)P.size(); i++){

		double x = (cameraMat.at<double>(0,0)*P[i].x + cameraMat.at<double>(0,1)*P[i].y + cameraMat.at<double>(0,2)*P[i].z + cameraMat.at<double>(0,3))
			/ (cameraMat.at<double>(2,0)*P[i].x + cameraMat.at<double>(2,1)*P[i].y + cameraMat.at<double>(2,2)*P[i].z + cameraMat.at<double>(2,3));
		double y = (cameraMat.at<double>(1,0)*P[i].x + cameraMat.at<double>(1,1)*P[i].y + cameraMat.at<double>(1,2)*P[i].z + cameraMat.at<double>(1,3))
			/ (cameraMat.at<double>(2,0)*P[i].x + cameraMat.at<double>(2,1)*P[i].y + cameraMat.at<double>(2,2)*P[i].z + cameraMat.at<double>(2,3));
		//std::cout << "x = " << x << ", y = " << y << std::endl;

		p.push_back(Point2d(x,y));
	}

	double sum = 0.0;
	for(int i=0; i<(int)p.size(); i++){
		double error = pow(pow(groundTruth[i].x-p[i].x,2.0)+pow(groundTruth[i].y-p[i].y,2.0),0.5);
		sum = sum + error;
		if(isDraw == true){
		//再投影
		//circle(cretex::projectorImage, groundTruth[i], cretex::RADIUS, cv::Scalar(255,0,0), -1, CV_AA); //正解 赤
		//circle(cretex::projectorImage, p[i], cretex::RADIUS, cv::Scalar(0,0,255), -1, CV_AA); //再投影 青
		}
	}
	//std::cout << "sum = " << sum << std::endl;
	//	std::cout << "return = " << sum/(int)p.size() << std::endl;

	return sum/(int)p.size();
}

void GRAYCODE::reProjectPoints(int radius)
{
	reProjectImage = cv::Mat::zeros(cv::Size(PROJECTOR_WIDTH,PROJECTOR_HEIGHT), CV_8UC3);

	for(int i = 0; i < g_worldPointInlierSet.size(); i++)
	{
		//再投影
		Point2f reprojectPoint = ProjectTo(pro_param, g_worldPointInlierSet[i]);
		//描画
		circle(reProjectImage, g_imagePointInlierSet[i], radius, cv::Scalar(255,0,0), -1, CV_AA); //正解 赤
		circle(reProjectImage, reprojectPoint, radius, cv::Scalar(0,0,255), -1, CV_AA); //再投影 青
	}

	for(int i = 0; i < random_calib_2d_points.size(); i++)
	{
		//ランダムに100点取ったやつ
		circle(reProjectImage, random_calib_2d_points[i], radius+2, cv::Scalar(0,255,0), -1, CV_AA); //緑
	}
}

Point2f GRAYCODE::ProjectTo(Params pro_param, Point3f world_point)
{
	//①
	cv::Mat wp = (cv::Mat_<double>(4,1) << world_point.x, world_point.y, world_point.z, 1.0);

	Mat mult_result = pro_param.perspectiveMat * wp;
	Point2f dst;
	dst.x = mult_result.at<double>(0,0) / mult_result.at<double>(2,0);
	dst.y = mult_result.at<double>(1,0) / mult_result.at<double>(2,0);

	////②
	//Mat cameraMat = pro_param.perspectiveMat;
	//dst.x = (cameraMat.at<double>(0,0)*world_point.x + cameraMat.at<double>(0,1)*world_point.y + cameraMat.at<double>(0,2)*world_point.z + cameraMat.at<double>(0,3))
	//	/ (cameraMat.at<double>(2,0)*world_point.x + cameraMat.at<double>(2,1)*world_point.y + cameraMat.at<double>(2,2)*world_point.z + cameraMat.at<double>(2,3));
	//dst.y = (cameraMat.at<double>(1,0)*world_point.x + cameraMat.at<double>(1,1)*world_point.y + cameraMat.at<double>(1,2)*world_point.z + cameraMat.at<double>(1,3))
	//	/ (cameraMat.at<double>(2,0)*world_point.x + cameraMat.at<double>(2,1)*world_point.y + cameraMat.at<double>(2,2)*world_point.z + cameraMat.at<double>(2,3));

	 return dst;
}


//RoomAliveを参考にRANSACを使ってみる
void GRAYCODE::six_points_calibration_2(vector<Point3d>& points_3d, vector<Point2d>& points_2d, Mat& dst){

	cout << "use RANSAC" << endl;

	double min_error=100000;

	int numCompletedFits = 0;

	for(int i=0; (numCompletedFits < 4) && (i < 10); i++){
		cout << "RANSAC iteration " << i << endl;

		vector<Point2d> calib_2d_points;
		vector<Point3d> calib_3d_points;
		Mat pm;
		double error;
		bool nonCoplanar = false; //同一平面上でないかどうか
		int nTries = 0;

		//1000点を非同一平面になるように取ってくる
		while(!nonCoplanar)
		{
			get_hundred_points(calib_2d_points, calib_3d_points, points_2d, points_3d);

			Mat X;
			double D = 0;
			double ssdToPlane = PlaneFit(calib_3d_points, X, &D);
			int numOutliers = 0;
			for(int p = 0; p < calib_3d_points.size(); p++)
			{
				double distanceFromPlane = X.at<double>(0,0) * calib_3d_points[p].x 
					+ X.at<double>(1,0) * calib_3d_points[p].y + X.at<double>(2,0) * calib_3d_points[p].z;
				if(abs(distanceFromPlane) > 0.1f)
					numOutliers++;
			}
			nonCoplanar = (numOutliers > calib_3d_points.size() * 0.10f);
			if(!nonCoplanar)
			{
				cout << "points are coplanar: try" << nTries << endl;
				calib_2d_points.clear();
				calib_3d_points.clear();
			}
			if(nTries++ > 1000)
			{
				cout << "over 1000 times" << endl;
				break;
			}
		}

		//6pointsAlgorithmを100pointsでやるバージョン
		six_points_algolism_2(calib_3d_points, calib_2d_points, pm, 100);
//		error = inspection_error_value(pm, points_3d, Size(512, 424), points_2d, false);
		error = inspection_error_value(pm, calib_3d_points, Size(512, 424), calib_2d_points, false);
		cout << "error =" << error << endl;

		//正対応を抽出
		bool eniughInliers = true;
		double sumError = 0;
		int poinsInSum = 0;

		vector<Point2d> imagePointInlierSet;
		vector<Point3d> worldPointInlierSet;
		for(int i=0; i<(int)points_3d.size(); i++){

			double x = (pm.at<double>(0,0)*points_3d[i].x + pm.at<double>(0,1)*points_3d[i].y + pm.at<double>(0,2)*points_3d[i].z + pm.at<double>(0,3))
				/ (pm.at<double>(2,0)*points_3d[i].x + pm.at<double>(2,1)*points_3d[i].y + pm.at<double>(2,2)*points_3d[i].z + pm.at<double>(2,3));
			double y = (pm.at<double>(1,0)*points_3d[i].x + pm.at<double>(1,1)*points_3d[i].y + pm.at<double>(1,2)*points_3d[i].z + pm.at<double>(1,3))
				/ (pm.at<double>(2,0)*points_3d[i].x + pm.at<double>(2,1)*points_3d[i].y + pm.at<double>(2,2)*points_3d[i].z + pm.at<double>(2,3));

			 Point2d p(x,y);
 			 double thisError = pow(pow(points_2d[i].x-p.x,2.0)+pow(points_2d[i].y-p.y,2.0),0.5);
			 if(thisError < 1.0f) //再投影誤差が許容範囲内ならば正対応点とする
			 {
				 worldPointInlierSet.push_back(points_3d[i]);
				 imagePointInlierSet.push_back(points_2d[i]);
			 }
			 sumError += thisError * thisError;
			 poinsInSum++;
		}

		eniughInliers = eniughInliers && (worldPointInlierSet.size() > 1000);
		cout << "Inlier points: " << worldPointInlierSet.size() << endl;

		if(eniughInliers)
		{
//****もういっかいランダムに非平面で点を取得
		//vector<Point2d> calib_2d_points_inlier;
		//vector<Point3d> calib_3d_points_inlier;
		//Mat pm_inlier;
		//bool nonCoplanar_inlier = false; //同一平面上でないかどうか
		//int nTries_inlier = 0;

		////100点を非同一平面になるように取ってくる
		//while(!nonCoplanar_inlier)
		//{
		//	get_hundred_points(calib_2d_points_inlier, calib_3d_points_inlier, imagePointInlierSet, worldPointInlierSet);
		//
		//	Mat X;
		//	double D = 0;
		//	double ssdToPlane = PlaneFit(calib_3d_points_inlier, X, &D);
		//	int numOutliers_inlier = 0;
		//	for(int p = 0; p < calib_3d_points_inlier.size(); p++)
		//	{
		//		double distanceFromPlane = X.at<double>(0,0) * calib_3d_points[p].x 
		//			+ X.at<double>(1,0) * calib_3d_points[p].y + X.at<double>(2,0) * calib_3d_points[p].z;
		//		if(abs(distanceFromPlane) > 0.1f)
		//			numOutliers_inlier++;
		//	}
		//	nonCoplanar_inlier = (numOutliers_inlier > calib_3d_points_inlier.size() * 0.10f);
		//	if(!nonCoplanar_inlier)
		//	{
		//		cout << "2: points are coplanar: try" << nTries_inlier << endl;
		//		calib_2d_points_inlier.clear();
		//		calib_3d_points_inlier.clear();
		//	}
		//	if(nTries_inlier++ > 1000)
		//	{
		//		cout << "2: over 1000 times" << endl;
		//		break;
		//	}
		//}
//****
			//正対応点すべてで6points
			six_points_algolism_2(worldPointInlierSet, imagePointInlierSet, pm, worldPointInlierSet.size());
			double error2 = inspection_error_value(pm, worldPointInlierSet, Size(512, 424), imagePointInlierSet, false);
			cout << "error with inliers =" << error2 << endl;
			cout << "projector matrix = \n" << pm << endl;

			if(error < min_error){
				min_error = error;
				dst = pm;
				g_imagePointInlierSet = imagePointInlierSet;
				g_worldPointInlierSet = worldPointInlierSet;
				random_calib_2d_points = calib_2d_points;
				random_calib_3d_points = calib_3d_points;
				numCompletedFits++;
			}

			////100点バージョン
			////ここでもランダムに非平面で点を取ってこないとダメ
			//six_points_algolism_2(calib_3d_points_inlier, calib_2d_points_inlier, pm_inlier);
			//double error2 = inspection_error_value(pm_inlier, worldPointInlierSet, Size(512, 424), imagePointInlierSet, false);
			//cout << "error with inliers =" << error2 << endl;
			//cout << "projector matrix = \n" << pm_inlier << endl;

			//if(error < min_error){
			//	min_error = error;
			//	dst = pm_inlier;
			//	g_imagePointInlierSet = imagePointInlierSet;
			//	g_worldPointInlierSet = worldPointInlierSet;
			//	random_calib_2d_points = calib_2d_points;
			//	random_calib_3d_points = calib_3d_points;
			//	numCompletedFits++;
			//}
		}
	}

	if(numCompletedFits == 0)
		cout << "Unable to successfully calibrate projector" << endl;
}

//ランダムに1000点を抽出
void GRAYCODE::get_hundred_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P){
	int i=0;
	//srand(time(NULL));    /* 乱数の初期化 */ 
	std::random_device rnd;//rand() < 32767 std::random_device < 0xffffffff=4294967295
	Vector<int> exists;
	while(i <= 100){
		int maxValue = (int)src_p.size();
		//int v = rand() % maxValue;
		int v = rnd() % maxValue;
		bool e2=false;
		for(int s=0; s<i; s++){
			if(exists[s] == v) e2 = true; 
		}
		if(!e2){
			exists.push_back(v);
			calib_P.push_back(src_P[v]);
			calib_p.push_back(src_p[v]);
			i++;
		}
	}
}

//pointsを平面フィッティング
double GRAYCODE::PlaneFit(vector<Point3d>& points, Mat& X, double* D)
{
	X = Mat(3, 1, CV_64F, Scalar::all(0));
	Mat mu(3, 1, CV_64F, Scalar::all(0));

	for(int i = 0; i < points.size(); i++){
		Mat p_mat = (cv::Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z);
		mu += p_mat;
	}
	mu *= (1 / (float)points.size());

	Mat A(3, 3, CV_64F, Scalar::all(0));
	Mat pc(3, 1, CV_64F, Scalar::all(0));
	Mat M(3, 3, CV_64F, Scalar::all(0));

	for(int i = 0; i < points.size(); i++)
	{
		Mat p_mat = (cv::Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z);
		pc = p_mat - mu;
		//Outer(直積？)
		for (int i = 0; i < M.rows; i++)
			for (int j = 0; j < M.cols; j++)
				M.at<double>(i,j) = pc.at<double>(i,0) * pc.at<double>(j,0);
				//M.data[i * M.cols + j] = pc.data[i] * pc.data[j];
		A += M;
	}
	Mat V(3, 3, CV_64F, Scalar::all(0)); //Aの固有ベクトル
	Mat d(3, 1, CV_64F, Scalar::all(0));//Aの固有値
	eigen(A, d, V); //cvの関数

	double minEigenvalue = DBL_MAX;
	int minEigenvaluei = 0;
	for(int i = 0; i < 3; i++)
	{
		if(d.at<double>(i) < minEigenvalue)
		{
			minEigenvalue = d.at<double>(i,0);
			minEigenvaluei = i;
		}
	}
		//最小固有値のベクトルだけXのその列にコピー
		//X.data[0] = V.at<double>(0,minEigenvaluei);
		//X.data[1] = V.at<double>(1,minEigenvaluei);
		//X.data[2] = V.at<double>(2,minEigenvaluei);
		X.at<double>(0,0) = V.at<double>(0,minEigenvaluei);
		X.at<double>(1,0) = V.at<double>(1,minEigenvaluei);
		X.at<double>(2,0) = V.at<double>(2,minEigenvaluei);

		*D = -(X.at<double>(0,0) * mu.at<double>(0,0) + X.at<double>(1,0) * mu.at<double>(1,0) + X.at<double>(2,0) * mu.at<double>(2,0));

		return minEigenvalue;
}

/***********************************
** 対応点保存 **
************************************/
void GRAYCODE::save_g_worldPointInlierSet()
{
	FileStorage fs("../g_worldPointInlierSet.xml", FileStorage::WRITE);

	write(fs, "points", g_worldPointInlierSet);

	cout << "points saved." << endl;

}

void GRAYCODE::load_g_worldPointInlierSet()
{
	FileStorage fs("../g_worldPointInlierSet.xml", FileStorage::READ);
	FileNode node(fs.fs, NULL);

	read(node["points"], g_worldPointInlierSet);

	cout << "loaded." << endl;

}

/***********************************
** 深度平滑化 **
************************************/

void GRAYCODE::call_smoothing()
{
	kinect->frame_smoothing();
}


/***********************************
************ myParam.h **************
************************************/
Params GRAYCODE::loadParams(const string& filename){
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

void GRAYCODE::saveCameraParams(const string& filename, Params param){
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




/***********************************
** その他（用途不明な過去の遺物） **
************************************/

// 画像変形・処理
// カメラ撮影領域からプロジェクタ投影領域を切り出し
void GRAYCODE::transport_camera_projector(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // リサイズした画像
	resize( src, src_resize, cv::Size(CMR_WIDTH, CMR_HEIGHT) );

	dst = cv::Mat( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // 幾何補正された画像（投影画像）

	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				if(src_resize.at<uchar>( p.y, 3*p.x ) != 0 && src_resize.at<uchar>( p.y, 3*p.x+1 ) != 0 && src_resize.at<uchar>( p.y, 3*p.x+2 ) != 0){
					//printf("x:%d, y:%d, p.x:%d, p.y:%d\n", x, y, p.x, p.y);
					dst.at<uchar>( y, 3*x ) = src_resize.at<uchar>( p.y, 3*p.x );      // B
					dst.at<uchar>( y, 3*x+1 ) = src_resize.at<uchar>( p.y, 3*p.x+1 );  // G
					dst.at<uchar>( y, 3*x+2 ) = src_resize.at<uchar>( p.y, 3*p.x+2 );  // R
				}
			}
		}
	}
}

// 入力画像をカメラ撮影領域に変形
void GRAYCODE::transport_projector_camera(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // リサイズした画像
	resize( src, src_resize, cv::Size(PRJ_WIDTH, PRJ_HEIGHT) );

	dst = cv::Mat( CMR_HEIGHT, CMR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // 幾何補正された画像（投影画像）

	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			cv::Point p = c->CamPro[y][x];
			if( p.x != -1 ) {
				//printf("x:%d, y:%d, p.x:%d, p.y:%d\n", x, y, p.x, p.y);
				dst.at<uchar>( p.y, 3*p.x ) = src_resize.at<uchar>( y, 3*x );      // B
				dst.at<uchar>( p.y, 3*p.x+1 ) = src_resize.at<uchar>( y, 3*x+1 );  // G
				dst.at<uchar>( p.y, 3*p.x+2 ) = src_resize.at<uchar>( y, 3*x+2 );  // R
			}
		}
	}
}