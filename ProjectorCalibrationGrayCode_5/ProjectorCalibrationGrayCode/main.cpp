/**
@file main.cpp
@mainpage
輝度補正を行うプログラム<br>
@section 簡単な処理の流れ
- 事前にガンマ値をRGBごとに測定しておく（一度測定すれば、プロジェクタかカメラを変えない限り一定）
- グレイコードを使ってプロジェクタとカメラの各画素を対応付け
- マスク（プロジェクタ投影領域）に合わせて投影する画像を変換（動画には非対応）
- 対応付けた関係を輝度補正用のクラスに渡す
- 最小輝度画像と最大輝度画像を撮影（ダイナミックレンジ計算用）
- 応答関数を推定
- 応答関数を使って読み込んだ画像の目標画像を作成

以降は、補正方法によって異なる。<br>
@section 応答関数を変化させない場合
- カメラ撮影画像が目標画像に近くなるように投影映像を変換
毎フレーム目標画像を変えることで動画対応可能<br>
その場合、補正中にカメラは不要<br>
事前に計測したガンマ値が精確なほど補正が綺麗に行われる。

@section 応答関数を変化させる場合
- カメラ撮影画像から反射率（応答関数の推定に必要なパラメータ）を計測
- LMSアルゴリズムを使って徐々に真の反射率（真の応答関数）へ近づけていく
ある程度ループが進むと反射率は収束する。<br>
動きに変化が少ない動画であれば反射率の変化は少ないため、そのまま動画対応が可能<br>
動きが大きい場合やシーンの切り替え時などは残像が残ってしまう。<br>
応答関数を真の値に近づけていくため、ガンマ値はある程度大雑把でも自然と修正されて上手くいく。

*/

#pragma once

#include "Header.h"
#include "Graycode.h"


#define MASK_ADDRESS "./GrayCodeImage/mask.bmp"
#define IMAGE_DIRECTORY "./UseImage"
#define SAVE_DIRECTORY "./UseImage/resize"
#define IMAGE_WIDTH (1280)
#define IMAGE_HEIGHT (800)
#define RADIUS (2)
#define GRAYCODE_NUM (5) //グレイコードを繰り返し投影する回数

int main()
{
	myKinect *mykinect = new myKinect();
	GRAYCODE *gc = new GRAYCODE(mykinect);

	cv::Mat src = cv::imread("./cap.jpg",1);
	cv::Mat src2 = cv::imread("./geo.bmp",1); //幾何補正後の見えたいカメラ画像(square.cpp/.hで作る)
	cv::Mat dst;

	printf("0：グレイコード投影&二値化\n");
	printf("1：対応付け\n");
	printf("2：パラメータ推定(6 points algolism)\n");
	printf("3：パラメータ推定(optimization)\n");
	printf("4: GrayCode幾何補正\n");
	printf("5: 点再投影\n");
	printf("6: GrayCode 連続投影開始\n");
	printf("w：待機時に白画像を投影するかしないか\n");
	printf("\n");

	mykinect->init();

	static bool prjWhite = true;

	// キー入力受付用の無限ループ
	while(true){
		printf("====================\n");
		printf("数字を入力してください....\n");
		int command;

		// 白い画像を全画面で投影（撮影環境を確認しやすくするため）
		//メインループ
		while(true){
			// trueで白を投影、falseで通常のディスプレイを表示
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// 何かのキーが入力されたらループを抜ける
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			mykinect->updateColorFrame();
			mykinect->updateDepthFrame();

			//画像表示
			mykinect->draw();
		}

		cv::destroyWindow("white_black");

		// 条件分岐
		switch (command){
		case '0':
			gc->code_projection();
			gc->make_thresh();
			break;

		case '1':
			gc->makeCorrespondence(); //プロジェクタ-カメラ対応付け
			gc->makeCorrespondenceToWorld();//プロジェクタ-3次元座標対応付け
			gc->PushPoints(); //点をプッシュ&配列クリア
			break;

		case '2':
			gc->calcParameters();//パラメータ推定開始
			break;

		case '3':
			gc->OptimizeParameters();//最適化
			break;

		case '4':
			//幾何対応付けで確認
			gc->makeCorrespondence();
			gc->transport_camera_projector(src2,dst);
			cv::imwrite("./penguin2.jpg",dst);
			cv::namedWindow("geometry", 0);
			Projection::MySetFullScrean(DISPLAY_NUMBER, "geometry");
			cv::imshow("geometry", dst);
			cv::waitKey(1000);
			mykinect->updateColorFrame();
			cv::imwrite("./cap2.jpg", mykinect->colorImage );
			cv::waitKey(0);

			break;

		case '5':
			//推定に使った点を再投影
			cv::namedWindow("reProjection", 0);
			gc->reProjectPoints(RADIUS);
			//フルスクリーン表示
			Projection::MySetFullScrean(DISPLAY_NUMBER, "reProjection");
			cv::imshow("reProjection", gc->reProjectImage);
			waitKey(0);
			break;

		case '6':
			cout << GRAYCODE_NUM << "回投影します。\n毎投影後、any keyで次に進めてください." << endl;
			int current = 0;
			int key = 0;
			while(current < GRAYCODE_NUM)
			{
				current++;
				//GrayCode投影、二値化
				gc->code_projection();
				gc->make_thresh();
				//対応付け
				cout << "対応付け開始." << endl;

				gc->makeCorrespondence(); //プロジェクタ-カメラ対応付け
				gc->makeCorrespondenceToWorld();//プロジェクタ-3次元座標対応付け
				gc->PushPoints(); //点をプッシュ&配列クリア

				//ここまでの取得点数
				cout << current <<  "回目までの取得点数：" << gc->points_3d.size() << "点" << endl;

				cout << "対応付け終了.\n次の準備ができたら何かキーを押してください..." << endl;
				waitKey(0);

			}
			break;

		case 'w':
			prjWhite = !prjWhite;
			break;

		default:
			exit(0);
			break;
		}
		printf("\n");
		cv::destroyAllWindows();
	}
}
