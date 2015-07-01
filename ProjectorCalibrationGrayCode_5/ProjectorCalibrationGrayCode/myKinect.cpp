#pragma once

#include "myKinect.h"

myKinect::myKinect()
{
	kinect = nullptr;
	colorFrameReader = nullptr;
	depthFrameReader = nullptr;
	infraredFrameReader = nullptr;

	colorBytesPerPixel = 4;

	dc1 = -0.0030711016;
	dc2 = 3.3309495161;

	colorWinName = "color window";
	depthWinName = "depth window";
	infraredWinName = "infrared window";
	coordinatedWinName = "MapDepthFrameToColorSpace coordinated window";
	coordinatedWinName2 = "MapColorFrameToDepthSpace coordinated window";

	image_idx = 0;
	outdir = "./capture";
}

//デストラクタ
myKinect::~myKinect()
{
}

void myKinect::init(){
	try{
		ERROR_CHECK(GetDefaultKinectSensor(&kinect));
		ERROR_CHECK(kinect->Open());
		BOOLEAN isOpen = false;
		ERROR_CHECK(kinect->get_IsOpen(&isOpen));
		if(!isOpen) throw runtime_error("Kinect cannot open.");

		kinect->get_CoordinateMapper(&coordinateMapper);

		initColorFrame();
		initDepthFrame();
		initIRFrame();

		namedWindow(colorWinName);
		namedWindow(depthWinName);
/*
		namedWindow(infraredWinName);
		namedWindow(coordinatedWinName);
		namedWindow(coordinatedWinName2);
*/
		getDepthCameraIntrinsics(depthcameraIntrinsics);

	}catch(exception& ex){
		cout << ex.what() << endl;
	}
}

void myKinect::initIRFrame(){
	ComPtr<IInfraredFrameSource> infraredFrameSource;
	ERROR_CHECK( kinect->get_InfraredFrameSource( &infraredFrameSource ) );
	ERROR_CHECK( infraredFrameSource->OpenReader( &infraredFrameReader ) );
 
	// 赤外線画像のサイズを取得する
	ComPtr<IFrameDescription> infraredFrameDescription;
	ERROR_CHECK( infraredFrameSource->get_FrameDescription( &infraredFrameDescription ) );
	ERROR_CHECK( infraredFrameDescription->get_Width( &infraredWidth ) );
	ERROR_CHECK( infraredFrameDescription->get_Height( &infraredHeight ) );
 
	// バッファーを作成する
	infraredBuffer.resize( infraredWidth * infraredHeight );
}

void myKinect::initColorFrame(){
	ComPtr<IColorFrameSource> colorFrameSource;
	ERROR_CHECK(kinect->get_ColorFrameSource(&colorFrameSource));
	ERROR_CHECK(colorFrameSource->OpenReader(&colorFrameReader));

	ComPtr<IFrameDescription> colorFrameDescription;
	ERROR_CHECK( colorFrameSource->CreateFrameDescription( 
		ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
	ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) );
	ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) );
	ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) );

	colorBuffer.resize(colorWidth * colorHeight * colorBytesPerPixel);
}

void myKinect::initDepthFrame(){
	ComPtr<IDepthFrameSource> depthFrameSource;
	ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
	ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

	ComPtr<IFrameDescription> depthFrameDescription;
	ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
	ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) );
	ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) );

	ERROR_CHECK( depthFrameSource->get_DepthMinReliableDistance( &minDepthReliableDistance ) );
	ERROR_CHECK( depthFrameSource->get_DepthMaxReliableDistance( &maxDepthReliableDistance ) );
	minDepth = minDepthReliableDistance;
	maxDepth = maxDepthReliableDistance;

	depthBuffer.resize(depthWidth * depthHeight);

	bufferSize = depthWidth * depthHeight * sizeof( unsigned short );
	Mat tmp_rawBuffer(depthWidth, depthHeight, CV_16UC1);
	rawBuffer = tmp_rawBuffer.clone();
	//rawBuffer->create(depthWidth, depthHeight);
}

void myKinect::updateIRFrame(){
	// フレームを取得する
	ComPtr<IInfraredFrame> infraredFrame;
	auto ret = infraredFrameReader->AcquireLatestFrame( &infraredFrame );
	if ( ret == S_OK ){
		// BGRAの形式でデータを取得する
		ERROR_CHECK( infraredFrame->CopyFrameDataToArray( infraredBuffer.size(), &infraredBuffer[0] ) );

		cv::Mat re( infraredHeight, infraredWidth, CV_16UC1, &infraredBuffer[0] );
		infraredImage = re.clone();
     
		// フレームを解放する
		// infraredFrame->Release();
	}
}


void myKinect::updateColorFrame(){
	ComPtr<IColorFrame> colorFrame;
	auto ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
	if ( ret != S_OK ){
		return;
	}

	ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
		colorBuffer.size(), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );

		Mat re(colorHeight, colorWidth, CV_8UC4, &colorBuffer[0]);
		cvtColor(re, colorImage, CV_RGBA2RGB);

//		 std::cout << colorImage.size() << std::endl;

}

void myKinect::multiUpdateColorFrame_in(myKinect* kinect)
{
	ComPtr<IColorFrame> colorFrame;
	auto ret = kinect->colorFrameReader->AcquireLatestFrame( &colorFrame );
	if ( ret != S_OK ){
		return;
	}

	ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray(
		kinect->colorBuffer.size(), &kinect->colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );

		Mat re(kinect->colorHeight, kinect->colorWidth, CV_8UC4, &kinect->colorBuffer[0]);
		cvtColor(re, kinect->colorImage, CV_RGBA2RGB);
}

void myKinect::swap_endiannes(uint16_t *out,const uint16_t *in, int size) {
	for(int i=0; i<size; i++) {
        uint16_t value=in[i];
		out[i] = ((value & 0x00ff) << 8) | (value >> 8);
    }
}

void myKinect::saveDepth_PMG(){
		std::ostringstream s;
		string filename;

		s.fill('0');
		s << std::right << std::setw(4) << image_idx;
		filename = s.str() + "-d1.pgm";

		unique_ptr<uint16_t[]> buffer(new uint16_t[depthWidth*depthHeight]);

		std::ofstream writer(outdir + "/" + filename);
		writer << "P2 " << depthWidth << " " << depthHeight << " 8000\n";//Header
		
		//std::ofstream writer(filename, std::ios_base::binary);
		//writer << "P5 " << depthWidth << " " << depthHeight << " 8000\n";//Header

		swap_endiannes((uint16_t*)buffer.get(), (uint16_t*)&depthBuffer[0], depthWidth*depthHeight); //Writing the buffer directly has endianness issues

		writer.write((char *)buffer.get(), depthWidth*depthHeight*2);
		std::vector<int> params(2);
		params[0] = CV_IMWRITE_PXM_BINARY;
		params[1] = 1;//0:ascii 1:binary
 //		imwrite(outdir + "/" + filename + "_ascii", rawBuffer, params);
 		imwrite(outdir + "/" + filename + "_binary", rawBuffer, params);
		//writer.write((char *)&depthBuffer[0], depthWidth*depthHeight*2);
		writer.close();
}

void myKinect::saveColor()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-c1.jpg";

	cv::imwrite(outdir + "/" + filename, colorImage);

}

void myKinect::saveDepth_JPG()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-d1.jpg";

	cv::imwrite(outdir + "/" + filename, depthImage);

}

void myKinect::saveIR()
{
	std::ostringstream s;
	string filename;

	s.fill('0');
	s << std::right << std::setw(4) << image_idx;
	filename = s.str() + "-r1.png";

	cv::imwrite(outdir + "/" + filename, infraredImage);
}

	void myKinect::updateDepthFrame(){
		ComPtr<IDepthFrame> depthFrame;
		auto ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
		if ( ret != S_OK ){
			return;
		}

		ERROR_CHECK( depthFrame->CopyFrameDataToArray( depthBuffer.size(), &depthBuffer[0] ) );
		//ERROR_CHECK( depthFrame->AccessUnderlyingBuffer( &bufferSize, reinterpret_cast<UINT16**>( &rawBuffer->data ) ) );
//		ERROR_CHECK( depthFrame->AccessUnderlyingBuffer( &bufferSize, reinterpret_cast<UINT16**>( &rawBuffer.data ) ) );

		Mat re(depthHeight, depthWidth, CV_8UC1);
		Mat raw_data(depthHeight, depthWidth, CV_16UC1);
		for(int i=0; i < re.total(); ++i){
			re.data[i] = depthBuffer[i];
			raw_data.data[i] = depthBuffer[i];
		}
		rawBuffer = raw_data.clone();
		//cout << "8000mm ---- " << getRawDepthValue(8000) << endl;
		//cout << "300mm ---- " << getRawDepthValue(300) << endl;
//		int center = depthWidth * (depthHeight / 2) + depthWidth/2;
//		cout << "depth["  << center << "]: " << 
//			depthBuffer[center]  << "mm ---- " << getRawDepthValue(depthBuffer[center]) << endl;

		depthImage = re.clone();
	}

	void myKinect::coordinateColorDepth(){
		colorPoints.resize(depthBuffer.size());
		ERROR_CHECK(coordinateMapper->MapDepthFrameToColorSpace(depthBuffer.size(), &depthBuffer[0], colorPoints.size(), &colorPoints[0]));

	    Mat re(depthHeight, depthWidth, CV_8UC4);

		for(int i = 0; i < depthBuffer.size(); i++){
			int colorX = (int)colorPoints[i].X;
			int colorY = (int)colorPoints[i].Y;

			int colorIndex = (colorY * colorWidth + colorX) * colorBytesPerPixel;
			int depthIndex = i * colorBytesPerPixel;

			if(isValidColorFrameRange(colorX, colorY) && isValidDepthRange(i)){
				re.data[depthIndex + 0] = colorBuffer[colorIndex + 0];
				re.data[depthIndex + 1] = colorBuffer[colorIndex + 1];
				re.data[depthIndex + 2] = colorBuffer[colorIndex + 2];
			}else{
				re.data[depthIndex + 0] = 0;
				re.data[depthIndex + 1] = 0;
				re.data[depthIndex + 2] = 0;
			}
		}
		coordinatedImage = re.clone();
	}

void myKinect::coordinateDepthColor(){
	depthPoints.resize(colorWidth * colorHeight);
	ERROR_CHECK(coordinateMapper->MapColorFrameToDepthSpace(depthBuffer.size(), &depthBuffer[0], depthPoints.size(), &depthPoints[0]));

	Mat re(colorHeight, colorWidth, CV_8UC4);

	for(int i = 0; i < re.total(); i++){
		int depthX = (int)depthPoints[i].X;
		int depthY = (int)depthPoints[i].Y;

		int depthIndex = (depthY * depthWidth) + depthX;
		int colorIndex = i * colorBytesPerPixel;

		if(isValidDepthFrameRange(depthX, depthY) && isValidDepthRange(depthIndex)){
			re.data[colorIndex + 0] = colorBuffer[colorIndex + 0];
			re.data[colorIndex + 1] = colorBuffer[colorIndex + 1];
			re.data[colorIndex + 2] = colorBuffer[colorIndex + 2];
		}else{
			re.data[colorIndex + 0] = 255;
			re.data[colorIndex + 1] = 255;
			re.data[colorIndex + 2] = 255;
		}
	}
	coordinatedImage2 = re.clone();
}

void myKinect::draw(){
	if(colorImage.data != nullptr) {
		Size half(colorWidth/2, colorHeight/2);
		colorImage_half = colorImage.clone();
		resize(colorImage_half, colorImage_half,half);
		imshow(colorWinName, colorImage_half);
	}

	if(depthImage.data != nullptr) imshow(depthWinName, depthImage);
/*
    if(infraredImage.data != nullptr) imshow(infraredWinName, infraredImage);
	if(coordinatedImage.data != nullptr) imshow(coordinatedWinName, coordinatedImage);
	if(coordinatedImage2.data != nullptr){
		Size half(colorWidth/2, colorHeight/2);
		resize(coordinatedImage2, coordinatedImage2,half);
		imshow(coordinatedWinName2, coordinatedImage2);
	}
*/
}

bool myKinect::isValidColorFrameRange(float x, float y){
	return ((0 <= x) && (x < colorWidth)) && ((0 <= y) && (y < colorHeight));
}

bool myKinect::isValidDepthFrameRange(float x, float y){
	return ((0 <= x) && (x < depthWidth)) && ((0 <= y) && (y < depthHeight));
}

bool myKinect::isValidDepthRange( int index )
{
	return (minDepth <= depthBuffer[index]) && (depthBuffer[index] <= maxDepth);
}

double myKinect::getRawDepthValue(UINT16 depthmillimeter)
{
	double ret = 1000 - depthmillimeter * dc2;
	if(depthmillimeter != 0){
		ret /= depthmillimeter;
		ret /= dc1;
		return ret;
	}else return 0;
}

void myKinect::getDepthCameraIntrinsics(CameraIntrinsics cameraIntrinsic)
{
	while(coordinateMapper->GetDepthCameraIntrinsics(&cameraIntrinsic) == S_OK)
	{
		if(cameraIntrinsic.FocalLengthX != 0)
		{
			cout << "=====Kinect Depth Camera Intrinsins=====" << endl;
			cout << "FocalLengthX(fx): " << cameraIntrinsic.FocalLengthX << endl;
			cout << "FocalLengthY(fy): " << cameraIntrinsic.FocalLengthY << endl;
			cout << "PrincipalPointX(cx): " << cameraIntrinsic.PrincipalPointX << endl;
			cout << "PrincipalPointY(cy): " << cameraIntrinsic.PrincipalPointY << endl;
			cout << "RadialDistortionSecondOrder(k2): " << cameraIntrinsic.RadialDistortionSecondOrder << endl;
			cout << "RadialDistortionFourthOrder(k4): " << cameraIntrinsic.RadialDistortionFourthOrder << endl;
			cout << "RadialDistortionSixthOrder(k6): " << cameraIntrinsic.RadialDistortionSixthOrder << endl;
			break;
		}
	}
}

void myKinect::coordinateColorToCamera(Point camPro[PROJECTOR_HEIGHT][PROJECTOR_WIDTH], 
									   Point3f proWorld[PROJECTOR_HEIGHT][PROJECTOR_WIDTH], int* pointnum)
{
	cameraPoints.resize(colorWidth * colorHeight);
	ERROR_CHECK(coordinateMapper->MapColorFrameToCameraSpace(depthBuffer.size(), &depthBuffer[0], cameraPoints.size(), &cameraPoints[0]));

	for(int i = 0; i < PROJECTOR_HEIGHT; i++){
		for(int j = 0; j < PROJECTOR_WIDTH; j++){
			//キャプチャのときflipで反転したのを元に戻す
			int camX = CAMERA_WIDTH - camPro[i][j].x;
			int camY = camPro[i][j].y;
			//エラー値(か、範囲外)が入っていたら、エラー値(0,0,-1)を格納
			if(camX < 0 || camY < 0
				|| camX > CAMERA_WIDTH || camY > CAMERA_HEIGHT) {
				Point3f *w = new Point3f(0, 0, -1);
				proWorld[i][j] = *w;
			}else{
				int index = camY * colorWidth + camX;
				Point3f *w = new Point3f(cameraPoints[index].X, cameraPoints[index].Y, cameraPoints[index].Z);
				//深度がとれていない点にはエラー値-1
				if(w->z <= 0 || w->z > 8.0 ) {
					w->x = 0; w->y = 0; w->z = -1;
				}else{
					(*pointnum)++;//有効な点をカウント
					//デバッグ用
//					cout << "point[" << *pointnum << "]: (" << camX << ", " << camY << 
//						") --- (" << w->x << ", " << w->y << ", " << w->z << ")\n" << endl;
				}
				proWorld[i][j] = *w;
				//デバッグ用
				//if(proWorld[i][j].z != -1)
				//cout << "point[" << *pointnum << "]: (" << camX << ", " << camY << 
				//	") --- (" << proWorld[i][j].x << ", " << proWorld[i][j].y << ", " << proWorld[i][j].z << ")\n" << endl;
			}
		}
	}
}

