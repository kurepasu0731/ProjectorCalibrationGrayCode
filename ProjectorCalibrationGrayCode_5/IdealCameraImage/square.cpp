#include "square.h"


/**
@brief ウィンドウ名を設定する
*/
void SQUARE::setWindowName(std::string windowName){
	this->windowName = windowName;
}

/**
@brief マスク画像を設定する関数
@param filename マスク画像（二値画像）のパス
@param _showSizeRate ディスプレイに表示するときの大きさ
*/
void SQUARE::setMask(std::string filename, double showSizeRate)
{
	mask = cv::imread(filename, 0);
	orgWidth = mask.cols;
	orgHeight = mask.rows;
	this->showSizeRate = showSizeRate;
	cv::resize(mask, mask, cv::Size(), showSizeRate, showSizeRate, cv::INTER_AREA);

	// Xa<Xb, Ya<Ybになってないとエラーが出るので適当に初期値入れておく
	Xa = 0;
	Xb = 10;
	Ya = 0;
	Yb = 10;
}

/**
@brief カーソルキーで矩形の位置を調節するメインの関数
@param size 変換する画像のサイズ（アスペクト比を維持するために必要）
@param searchDir 変換する画像が入っているディレクトリ名
*/
void SQUARE::adjustSquare(cv::Size size, std::string searchDir, std::string saveDir)
{
	std::cout << "カーソルキーで矩形の位置を調節してEnterを押して下さい。" << std::endl;
	std::cout << "前回の座標を使う場合はカーソルキーを押さずにEnterを押して下さい。" << std::endl;
	std::cout << "r：ランダム自動探索" << std::endl;
	std::cout << "q：終了します。" << std::endl;
	std::cout << "S：指定したディレクトリ内の画像を一括変換して保存します。" << std::endl;

	cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
	cv::imshow(windowName, mask);
	autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(0, 0), 0.01, false);
	//randomSelectSquare(cv::Size(size.width, size.height), 0.0005, 500);

	// 矩形の中心位置
	int centX = round( (Xa + (Xb - Xa)*0.5) * showSizeRate );
	int centY = round( (Ya + (Yb - Ya)*0.5) * showSizeRate );
	int x = round( centX - mask.cols*0.5 );
	int y = round( centY - mask.rows*0.5 );

	// カーソルキー押下時に動くピクセル数
	int step = 5;
	// 終了フラグ
	bool exit = false;

	while(true){
		switch(cv::waitKey(0)){
		case 'q':
		case 'Q':
			return;

		case 2490368:	// カーソルキー「↑」
			y -= step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "リサイズ後解像度：" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 2621440:	// カーソルキー「↓」
			y += step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "リサイズ後解像度：" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 2555904:	// カーソルキー「→」
			x += step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "リサイズ後解像度：" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 2424832:	// カーソルキー「←」
			x -= step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "リサイズ後解像度：" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 'r':	// 最大矩形推定を追加で行う（現在より小さくなることはない）
			randomSelectSquare(cv::Size(size.width, size.height), 0.001, 500);

			// 矩形の中心位置
			centX = round( (Xa + (Xb - Xa)*0.5) * showSizeRate );
			centY = round( (Ya + (Yb - Ya)*0.5) * showSizeRate );
			x = round( centX - mask.cols*0.5 );
			y = round( centY - mask.rows*0.5 );
			break;

		case 'c':	// 中心で矩形推定
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(0, 0));
			std::cout << "リサイズ後解像度：" << Xb-Xa << " x " << Yb-Ya << std::endl;

			// 矩形の中心位置
			x = 0;
			y = 0;
			break;

		case 'd':	// デバッグ画像表示
			debug = !debug;
			randomSelectSquare(cv::Size(size.width, size.height), 0.001, 500);
			// 矩形の中心位置
			centX = round( (Xa + (Xb - Xa)*0.5) * showSizeRate );
			centY = round( (Ya + (Yb - Ya)*0.5) * showSizeRate );
			x = round( centX - mask.cols*0.5 );
			y = round( centY - mask.rows*0.5 );
			break;

		case 'S':
			{
				// 保存した座標を取得
				loadData();
				// IMAGE_DIRECTORY内の画像ファイル名をすべて取得
				std::vector<std::string> file_list = getAllFilename(searchDir);
				int i = 0;

				// IMAGE_DIRECTORY内の画像をすべて変換
				std::cout << "変換中" << std::endl;
				for (auto &path : file_list) {
					i++;
					// 画像読み込み
					char fileName[128];
					sprintf_s(fileName, "%s/%s", searchDir.c_str(), path.c_str());
					cv::Mat src = cv::imread(fileName, 1);
					// 変換して保存
					char saveName[128];
					sprintf_s(saveName, "%s/sample%02d.png", saveDir.c_str(), i);
					saveImage(src, saveName);
					std::cout.fill('0');
					std::cout.width(2);
					std::cout << i << ":\t" << fileName << std::endl;
				}

				// マスク用の画像作成
				cv::Mat src = cv::Mat(size.height, size.width, CV_8UC3, cv::Scalar(255, 255, 0));
				saveImage(src, "./CalculationData/RectMask.bmp");

				std::cout << "complete!" << std::endl;
				exit = true;
			}
			break;

		default:
			cv::Mat src = cv::Mat(size.height, size.width, CV_8UC3, cv::Scalar(255, 255, 0));
			saveImage(src, "./CalculationData/RectMask.bmp");
			std::cout << "矩形領域の指定完了" << std::endl;
			exit = true;
			break;
		}

		// whileループを抜ける
		if(exit)
			break;
	}
	cv::destroyWindow(windowName);
}

/**
@brief 中心位置をランダムに決めて矩形推定を何度も行い、最大サイズになる矩形を探す。
@param size 変換する画像のサイズ（アスペクト比を維持するために必要）
@param shift 中心位置をずらす画素数。
@param precision 精度。値が小さいほど細かくリサイズされる。
@param savePoint trueで座標を保存する。
@param num 最大矩形の推定を行う回数
*/
void SQUARE::randomSelectSquare(cv::Size size, double precision, int num){
	// 乱数生成
	std::random_device rd;
	std::mt19937 mt(rd());
	double randomSizeRateX = 0.3;
	double randomSizeRateY = 0.3;
	std::uniform_int_distribution<int> randX(-round(mask.cols*randomSizeRateX), round(mask.cols*randomSizeRateX));
	std::uniform_int_distribution<int> randY(-round(mask.rows*randomSizeRateY), round(mask.rows*randomSizeRateY));

	cv::Mat random;
	// ランダムの範囲の視覚化
	if(debug){
		random = mask.clone();
		cv::rectangle(random, cv::Rect(mask.cols/2-round(mask.cols*randomSizeRateX), mask.rows/2-round(mask.rows*randomSizeRateY),
			round(mask.cols*randomSizeRateX*2), round(mask.rows*randomSizeRateY*2)), cv::Scalar(230, 230, 230), -1);
	}

	// 最初の矩形内の白の画素数を数える
	pixels = 0;
	for (int j = round(Ya*showSizeRate); j < round(Yb*showSizeRate); j++)
	{
		for (int i = round(Xa*showSizeRate); i < round(Xb*showSizeRate); i++)
		{
			int white = mask.data[j*mask.step + i*mask.elemSize()];
			if(white == 255){
				pixels++;
			}
		}
	}

	// 矩形推定のループ
	for (int j = 0; j < num; j++)
	{
		// ランダムに座標を決定
		int x = randX(mt);
		int y = randY(mt);

		// マスク外になったらやり直し（height*rate=表示用にリサイズした画像サイズ -> *0.5=その画像の真ん中, +y=真ん中から移動した距離）
		uchar Y = mask.data[round(mask.rows*0.5+y)*mask.step + round(mask.cols*0.5+x)*mask.elemSize()];
		if(Y == 0){
			j--;
			continue;
		}

		// 座標にマーク
		if(debug)
			cv::circle(random, cv::Point(round(mask.cols*0.5+x), round(mask.rows*0.5+y)), 2, cv::Scalar(128, 128, 128), -1, CV_AA);

		// 最大となる領域の始点と終点を再帰的に探索
		int tmp = autoSelectSquare(pixels, size, cv::Point(x, y), precision, false);
		if(pixels < tmp)
			pixels = tmp;
	}
	drawResult(true);
	std::cout << "リサイズ後解像度：" << Xb-Xa << " x " << Yb-Ya << std::endl;

	// デバッグ用の画像
	if(debug){
		char debugWindow[] = "デバッグ：ランダムに座標をとった画像";
		cv::namedWindow(debugWindow, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
		cv::imshow(debugWindow, random);
		cv::imwrite("./ランダム.png", random);
	}
}

/**
@brief マスクの領域内で最大となる矩形の左上と右下の座標を求める関数<br>
デフォルトでは画像の中心をマスクの中心として求める
@param minPixels 求まった矩形内のピクセル数がこの値以下の場合は無視する
@param size 変換する画像のサイズ（アスペクト比を維持するために必要）
@param shift 中心位置をずらす画素数。
@param precision 精度。値が小さいほど細かくリサイズされる。
@param savePoint trueで座標を保存する。
@return 矩形内のピクセル数
*/
int SQUARE::autoSelectSquare(int minPixels, cv::Size size, cv::Point shift, double precision, bool savePoint){
	// エラー処理
	if( (abs(shift.x) >= round(mask.cols*0.5)) || (abs(shift.y) >= round(mask.rows*0.5)) )
		return minPixels;

	// 四隅に黒が含まれない最大サイズの矩形を探索
	cv::Mat rect;
	if(debug){
		rect = mask.clone();
		cv::cvtColor(rect, rect, cv::COLOR_GRAY2BGR);
	}

	// エラーが出ないように「tXa<tXb, tYa<tYb」となる適当な値を入れる
	int tXa = 0;
	int tXb = 0;
	int tYa = 10;
	int tYb = 10;
	int i = 0;
	while(true){
		i++;
		// 小さい矩形を定義
		int sWidth = round(size.width * precision * i);
		int sHeight = round(size.height * precision * i);
		// ウィンドウサイズ0だとエラーが出るので小さすぎる場合は飛ばす
		// 大きい値にするほどループ回数が減るので若干速くなる
		if( (sWidth < 10) | (sHeight < 10) ){
			continue;
		}

		cv::Mat roi(mask, cv::Rect(mask.cols/2-sWidth/2+shift.x, mask.rows/2-sHeight/2+shift.y, sWidth, sHeight));

		// 全部の矩形を描画した画像を作成
		if(debug){
			cv::rectangle(rect, cv::Rect(mask.cols/2-sWidth/2+shift.x, mask.rows/2-sHeight/2+shift.y, sWidth, sHeight), cv::Scalar(0, 0, 255), 1);
			cv::circle(rect, cv::Point(mask.cols/2+shift.x, mask.rows/2+shift.y), 2, cv::Scalar(0, 255, 0), -1, CV_AA);
		}

		// 矩形の四隅に1ヶ所でも黒が含まれていたら探索終了
		int leftUp = roi.at<uchar>(0, 0);
		int rightUp = roi.at<uchar>(0, roi.cols-1);
		int rightDown = roi.at<uchar>(roi.rows-1, roi.cols-1);
		int leftDown = roi.at<uchar>(roi.rows-1, 0);
		if( (leftUp == 0) || (rightUp == 0) || (rightDown == 0) || (leftDown == 0) )
			break;

		// 始点と終点の座標を代入
		//Xa = round( (mask.cols/2 - sWidth/2 - shift.x) / showSizeRate );
		//Ya = round( (mask.rows/2 - sHeight/2 - shift.y) / showSizeRate );
		//Xb = round( (mask.cols/2 + sWidth/2 - shift.x) / showSizeRate );
		//Yb = round( (mask.rows/2 + sHeight/2 - shift.y) / showSizeRate );

		// 一時的な始点と終点の座標を代入
		tXa = round( (mask.cols/2 - sWidth/2 + shift.x) );
		tYa = round( (mask.rows/2 - sHeight/2 + shift.y) );
		tXb = round( (mask.cols/2 + sWidth/2 + shift.x) );
		tYb = round( (mask.rows/2 + sHeight/2 + shift.y) );
	}

	// 矩形内の白の画素数を数える
	int truePixels = 0;
	for (int j = tYa; j < tYb; j++)
	{
		for (int i = tXa; i < tXb; i++)
		{
			int white = mask.data[j*mask.step + i*mask.elemSize()];
			if(white == 255){
				truePixels++;
			}
		}
	}

	// 前回より多かった場合は座標を更新
	if(minPixels < truePixels){
		Xa = round( tXa / showSizeRate );
		Ya = round( tYa / showSizeRate );
		Xb = round( tXb / showSizeRate );
		Yb = round( tYb / showSizeRate );
		drawResult(savePoint);
		// デバッグ用の画像
		if(debug){
			char debugWindow[] = "デバッグ：徐々に大きくなる矩形画像";
			cv::namedWindow(debugWindow, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
			cv::imshow(debugWindow, rect);
			cv::imwrite("./回.png", rect);
		}
	}

	return truePixels;
}

/**
@brief 結果の画像を表示して値を保存する。
@param savePoint trueで座標を保存する。
*/
void SQUARE::drawResult(bool savePoint){
	// ディスプレイ表示用
	cv::Mat kakunin = mask.clone();
	cv::cvtColor(kakunin, kakunin, cv::COLOR_GRAY2BGR);
	cv::rectangle(kakunin, cv::Point2d(Xa*showSizeRate, Ya*showSizeRate), cv::Point2d(Xb*showSizeRate, Yb*showSizeRate), cv::Scalar(255, 255, 0), -1);
	// 中心
	cv::circle(kakunin, cv::Point2d(mask.cols/2, mask.rows/2), 4, cv::Scalar(160, 160, 200), -1);
	// 矩形の中心
	cv::circle(kakunin, cv::Point2d(Xa*showSizeRate+(Xb-Xa)/2*showSizeRate, Ya*showSizeRate+(Yb-Ya)/2*showSizeRate), 4, cv::Scalar(0, 0, 200), -1);
	cv::imshow(windowName, kakunin);

	// 結果を保存
	if(savePoint){
		// リサイズするとカーソルで連続的に動かすとき重くなる
		//cv::resize(kakunin, kakunin, cv::Size(), 1.0/showSizeRate, 1.0/showSizeRate);
		cv::imwrite("./SaveImage/RectMask（確認用）.png", kakunin);
		std::ofstream ofs("./CalculationData/Rect_value.txt");
		ofs << Xa << "\t" << Ya << std::endl;
		ofs << Xb << "\t" << Yb << std::endl;
		ofs.close();
	}
}

/**
@brief ファイルからデータを読み込む関数
*/
bool SQUARE::loadData()
{
	std::ifstream ifs("./CalculationData/Rect_value.txt");

	if(ifs.fail()){
		std::cout << "LOAD ERROR：ファイルが読み込めませんでした" << std::endl;
		return false;
	}

	int col = 1;	// エラー文で使用
	char buf[256];
	float x[2], y[2];

	// 1行ずつ読み込み
	while(ifs.getline(buf, 256)){
		// コメントと空行は読み飛ばす
		if( (strncmp(buf, "//", 2) == 0) || (strncmp(buf, "#", 1) == 0) || (strcmp(buf, "\0") == 0) ){
			col++;
			continue;
		}
		// 値が2つ書かれている行以外が読み込まれたらエラー
		if(sscanf_s(buf, "%f %f", &x[col-1], &y[col-1]) != 2){
			std::cout << "DATA LOAD ERROR(1)：" << col << " 行目の書式が間違っています。" << std::endl;
			break;
		}
		col++;
	}
	ifs.close();

	Xa = round(x[0]);
	Ya = round(y[0]);
	Xb = round(x[1]);
	Yb = round(y[1]);

	return true;
}

/**
@brief リサイズした画像を保存する関数
@param src 保存する画像
@param saveName 保存後のファイル名
*/
void SQUARE::saveImage(cv::Mat &src, std::string saveName)
{
	cv::Mat dst, save;
	// 矩形サイズにリサイズ
	cv::Point center = resize(src, dst, cv::Point(Xa, Ya), cv::Point(Xb, Yb));
	// 矩形中心に並進移動
	imageShift(dst, save, center);
	// マスクサイズにリサイズ
	cv::resize(save, save, cv::Size(orgWidth, orgHeight));
	cv::imwrite(saveName, save);
}

/**
@brief 画像を並進移動させる
@param src 移動する画像
@param dst 移動後の画像
@param center 中心からずらす距離
*/
void SQUARE::imageShift(cv::Mat &src, cv::Mat &dst, cv::Point center)
{
	// 上一列と左一列が黒い入力画像を作成
	cv::Mat tmp(cv::Size(src.cols+1, src.rows+1), CV_8UC3, cv::Scalar(0));
	cv::getRectSubPix(src, cv::Size(src.cols, src.rows), cv::Point(src.cols/2+1, src.rows/2+1), tmp);
	src = tmp.clone();
	cv::line(src, cv::Point(0, 0), cv::Point(src.cols-1, 0), cv::Scalar(0, 0, 0), 1);
	cv::line(src, cv::Point(0, 0), cv::Point(0, src.rows-1), cv::Scalar(0, 0, 0), 1);

	// 黒背景と合成
	cv::Mat back = cv::Mat(orgHeight, orgWidth, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat roi(back, cv::Rect(0, 0, src.cols, src.rows));
	src.copyTo(roi);

	// 中心位置の補正
	int offsetX = back.cols/2 - center.x;
	int offsetY = back.rows/2 - center.y;

	// 並進移動
	cv::getRectSubPix(back, cv::Size(back.cols, back.rows), cv::Point(src.cols/2 + offsetX, src.rows/2 + offsetY), dst);
}

/**
@brief 画像を並進移動させてマスクを適用する（結果確認用）
@param src 移動する画像
@param dst 移動後の画像
@param center 中心からずらす距離
*/
void SQUARE::imageShiftAdaptMask(cv::Mat &src, cv::Mat &dst, cv::Point center)
{
	// 上一列と左一列が白い入力画像を作成
	cv::Mat tmp(cv::Size(src.cols+1, src.rows+1), CV_8UC3, cv::Scalar(255));
	cv::getRectSubPix(src, cv::Size(src.cols, src.rows), cv::Point(src.cols/2+1, src.rows/2+1), tmp);
	src = tmp.clone();
	cv::line(src, cv::Point(0, 0), cv::Point(src.cols-1, 0), cv::Scalar(255, 255, 255), 1);
	cv::line(src, cv::Point(0, 0), cv::Point(0, src.rows-1), cv::Scalar(255, 255, 255), 1);

	// 白背景と合成
	cv::Mat back = cv::Mat(orgHeight, orgWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat roi(back, cv::Rect(0, 0, src.cols, src.rows));
	src.copyTo(roi);

	// 中心位置の補正
	int offsetX = back.cols/2 - center.x;
	int offsetY = back.rows/2 - center.y;

	// 並進移動
	cv::getRectSubPix(back, cv::Size(back.cols, back.rows), cv::Point(src.cols/2 + offsetX, src.rows/2 + offsetY), dst);

	// マスクを適用
	cv::Mat _mask;
	cv::resize(mask, _mask, cv::Size(), 1.0/showSizeRate, 1.0/showSizeRate, cv::INTER_AREA);
	tmp = _mask.clone();
	dst.copyTo(tmp, _mask);
	dst = tmp.clone();
}

/**
@brief 対角上の2点を使ってリサイズを行う
@param src リサイズする画像
@param dst リサイズ後の画像
@param start 左上の座標
@param end 右下の座標
@return 矩形の中心座標
*/
cv::Point SQUARE::resize(cv::Mat &src, cv::Mat &dst, cv::Point start, cv::Point end)
{
	double sqWidth = end.x - start.x;
	double sqHeight = end.y - start.y;
	cv::Point center;
	center.x = round(start.x + sqWidth/2);
	center.y = round(start.y + sqHeight/2);
	double r1 = abs( sqWidth / src.cols );
	double r2 = abs( sqHeight / src.rows );

	// 指定した矩形内に収まるように、縮小率の大きい方を使う
	double r = r2;
	if(r1 < r2)
		r = r1;
	// エラー処理
	if(r == 0)
		r = 0.01;

	cv::resize(src, dst, cv::Size(), r, r, cv::INTER_AREA);
	return center;
}

/**
@brief 下位ディレクトリ内の全ファイルの名前を取得（一括変換用）
*/
std::vector<std::string> SQUARE::getAllFilename(std::string searchDir)
{
	std::vector<std::string> file_list;

	// カレントディレクトリ以下のファイル名を取得する
	// 再帰的にファイル名を取得する場合は、std::tr2::sys::recursive_directory_iteratorを使う
	for (std::tr2::sys::directory_iterator it(searchDir), end; it != end; ++it) {
		// 画像ファイルだけ取得
		std::string ext = it->path().extension();
		if(ext == ".jpg" ||
			ext == ".JPG" ||
			ext == ".png" ||
			ext == ".PNG" ||
			ext == ".bmp" ||
			ext == ".BMP"
			){
				file_list.push_back(it->path());
		}
	}

	// 取得したファイル名をすべて表示する
	//for (auto &path : file_list) {
	//	std::cout << path << std::endl;
	//}
	return file_list;
}

/**
@brief setMouseCallbackで設定したコールバック関数内で呼ぶ関数<br>
マウスで直に矩形を指定する
@warning 自動で出来るようにしたので不要になったけど一応残しておく
*/
void SQUARE::onMouse( int event, int x, int y, int flag, void* param)
{
	static int ptSize = 4;
	static bool isLButton = false;
	cv::Mat referenceMat = mask.clone();
	cv::cvtColor(referenceMat, referenceMat, cv::COLOR_GRAY2BGR);
	static int count = 0;
	switch (event){
	case cv::EVENT_LBUTTONDOWN:
		// クリックした座標に点を描画
		isLButton = true;
		Xa = x*2;
		Ya = y*2;
		cv::circle(referenceMat, cv::Point(Xa/2, Ya/2), ptSize, cv::Scalar(0, 0, 200), -1);
		cv::imshow(windowName, referenceMat);
		break;

	case cv::EVENT_MOUSEMOVE:
		// ドラッグ中に矩形などを描画
		if(isLButton){
			// 始点、終点、中心の座標
			cv::circle(referenceMat, cv::Point(Xa/2, Ya/2), ptSize, cv::Scalar(0, 0, 200), -1);
			cv::circle(referenceMat, cv::Point(x, y), ptSize, cv::Scalar(0, 0, 200), -1);
			cv::circle(referenceMat, cv::Point(x-(x-Xa/2)/2, y-(y-Ya/2)/2), ptSize, cv::Scalar(0, 0, 200), -1);
			// 矩形
			cv::rectangle(referenceMat, cv::Point(Xa/2, Ya/2), cv::Point(x, y), cv::Scalar(0, 0, 200));
			cv::imshow(windowName, referenceMat);
		}
		break;

	case cv::EVENT_LBUTTONUP:
		// リサイズと指定した座標の保存を行う
		isLButton = false;
		Xb = x*2;
		Yb = y*2;
		cv::Mat src = cv::Mat(orgHeight, orgWidth, CV_8UC3, cv::Scalar(255, 255, 0));
		cv::Mat dst, kakunin, save;
		// Xa, Ya, Xb, Yb, centerを保存しておけば一括でリサイズ＆位置合わせできる
		cv::Point center = resize(src, dst, cv::Point(Xa, Ya), cv::Point(Xb, Yb));
		imageShiftAdaptMask(dst, kakunin, center);
		cv::imwrite("resize.bmp", kakunin);
		cv::resize(kakunin, kakunin, cv::Size(), showSizeRate, showSizeRate, cv::INTER_AREA);

		// 始点、終点、中心の座標
		cv::circle(kakunin, cv::Point(Xa/2, Ya/2), ptSize, cv::Scalar(0, 0, 200), -1);
		cv::circle(kakunin, cv::Point(Xb/2, Yb/2), ptSize, cv::Scalar(0, 0, 200), -1);
		cv::circle(kakunin, cv::Point(Xb/2-(Xb/2-Xa/2)/2, Yb/2-(Yb/2-Ya/2)/2), ptSize, cv::Scalar(0, 0, 200), -1);
		// 矩形
		cv::rectangle(kakunin, cv::Point(Xa/2, Ya/2), cv::Point(Xb/2, Yb/2), cv::Scalar(0, 0, 200));
		cv::imshow(windowName, kakunin);
		imageShift(dst, kakunin, center);
		cv::imwrite("resize2.bmp", kakunin);

		// 結果を保存
		std::ofstream ofs("./CalculationData/Rect_value.txt");
		ofs << Xa << "\t" << Ya << std::endl;
		ofs << Xb << "\t" << Yb << std::endl;
		ofs.close();
		break;
	}
}
