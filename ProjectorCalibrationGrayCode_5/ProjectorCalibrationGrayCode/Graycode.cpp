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
	// �\���̂̏�����
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
	// �O���C�R�[�h�B�e�摜
	_mkdir("./GrayCodeImage/CaptureImage");
	// �O���C�R�[�h���摜
	_mkdir("./GrayCodeImage/ProjectionGrayCode");
	// �O���C�R�[�h�B�e�摜�̓�l�������摜
	_mkdir("./GrayCodeImage/ThresholdImage");
}

/***************************
** �O���C�R�[�h�̍쐬�֘A **
****************************/

// �r�b�g���̌v�Z�ƃO���C�R�[�h�̍쐬
void GRAYCODE::initGraycode()
{
	int bin_code_h[PRJ_HEIGHT];  // 2�i�R�[�h�i�c�j
	int bin_code_w[PRJ_WIDTH];   // 2�i�R�[�h�i���j
	int graycode_h[PRJ_HEIGHT];  // �O���C�R�[�h�i�c�j
	int graycode_w[PRJ_WIDTH];   // �O���C�R�[�h�i���j
	//int *graycode_h =  new int[c->g.h_bit];  // �O���C�R�[�h�i�c�j
	//int *graycode_w =  new int[c->g.w_bit];  // �O���C�R�[�h�i���j

	/***** 2�i�R�[�h�쐬 *****/
	// �s�ɂ���
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		bin_code_h[y] = y + 1;
	// ��ɂ���
	for( int x = 0; x < PRJ_WIDTH; x++ )
		bin_code_w[x] = x + 1;

	/***** �O���C�R�[�h�쐬 *****/
	// �s�ɂ���
	for( int y = 0; y < PRJ_HEIGHT; y++ )
		graycode_h[y] = bin_code_h[y] ^ ( bin_code_h[y] >> 1 );
	// ��ɂ���
	for( int x = 0; x < PRJ_WIDTH; x++ )
		graycode_w[x] = bin_code_w[x] ^ ( bin_code_w[x] >> 1 );
	// �s������킹��i�s + ��j
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ )
			c->g.graycode[y][x] = ( graycode_h[y] << c->g.w_bit) | graycode_w[x];
	}
}

// �p�^�[���R�[�h�摜�쐬�i��x���΃v���W�F�N�^�̉𑜓x���ς��Ȃ������蒼���K�v�͂Ȃ��j
void GRAYCODE::makeGraycodeImage()
{
	std::cout << "���e�p�O���C�R�[�h�쐬��" << std::endl;
	//initGraycode();
	cv::Mat posi_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	cv::Mat nega_img ( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );
	int bit = c->g.all_bit-1;
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit];  // �����t���o��
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];  // �����t���o��

	// �|�W�p�^�[���R�[�h�摜�쐬
	for( unsigned int z = 0; z < c->g.all_bit; z++) {
		for( int y = 0; y < PRJ_HEIGHT; y++ ) {
			for( int x = 0; x < PRJ_WIDTH; x++ ) {
				if( ( (c->g.graycode[y][x] >> (bit-z)) & 1 ) == 0 ) {  // �ŏ�ʃr�b�g���珇�ɒ��o���C���̃r�b�g��0��������
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
		// �A�ԂŃt�@�C������ۑ��i������X�g���[���j
		Filename_posi[z] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_posi[z].str(), posi_img);
		Filename_posi[z] << std::endl;
	}

	// �l�K�p�^�[���R�[�h�摜�쐬
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
		// �A�ԂŃt�@�C������ۑ��i������X�g���[���j
		Filename_nega[z] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << z << ".bmp"; 
		cv::imwrite(Filename_nega[z].str(), nega_img);
		Filename_nega[z] << std::endl;
	}

	delete[] Filename_posi;
	delete[] Filename_nega;
}

// �p�^�[���R�[�h���e & �B�e
void GRAYCODE::code_projection()
{
	// �萔
	typedef enum flag{
		POSI = true,
		NEGA = false,
		VERTICAL = true,
		HORIZONTAL = false,
	} flag;

	Graycode *g = new Graycode();
	//TPGROpenCV	pgrOpenCV;

	//������&�J�����N��
	initGraycode();
	////pgrOpenCV.init( FlyCapture2::PIXEL_FORMAT_BGR );
	//pgrOpenCV.setShutterSpeed(pgrOpenCV.getShutter_h());
	//pgrOpenCV.start();

	cv::Mat *posi_img = new cv::Mat[c->g.all_bit];  // �|�W�p�^�[���p
	cv::Mat *nega_img = new cv::Mat[c->g.all_bit];  // �l�K�p�^�[���p

	// �����t���o�́i�O���C�R�[�h�ǂݍ��ݗp�j
	std::stringstream *Filename_posi = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega = new std::stringstream[c->g.all_bit];
	// �����t���o�́i�B�e�摜�������ݗp�j
	std::stringstream *Filename_posi_cam = new std::stringstream[c->g.all_bit]; 
	std::stringstream *Filename_nega_cam = new std::stringstream[c->g.all_bit];

	// �A�ԂŃt�@�C������ǂݍ��ށi������X�g���[���j
	std::cout << "���e�p�O���C�R�[�h�摜�ǂݍ��ݒ�" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		Filename_posi[i] << "./GrayCodeImage/ProjectionGrayCode/posi" << std::setw(2) << std::setfill('0') << i << ".bmp";
		Filename_nega[i] << "./GrayCodeImage/ProjectionGrayCode/nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		// �ǂݍ���
		posi_img[i] = cv::imread(Filename_posi[i].str(), 1);
		nega_img[i] = cv::imread(Filename_nega[i].str(), 1);
		Filename_posi[i] << std::endl;
		Filename_nega[i] << std::endl;
		// �ǂݍ��ޖ���������Ȃ�������O���C�R�[�h�摜����蒼��
		if(posi_img[i].empty() || nega_img[i].empty()){
			std::cout << "ERROR(1)�F���e�p�̃O���C�R�[�h�摜���s�����Ă��܂��B" << std::endl;
			std::cout << "ERROR(2)�F�O���C�R�[�h�摜���쐬���܂��B" << std::endl;
			makeGraycodeImage();
			code_projection();
			return;
		}
	}

	/***** �O���C�R�[�h���e & �B�e *****/
	/*  �S��ʕ\���p�E�B���h�E�̍쐬  */
	cv::namedWindow(GC, 0);
	Projection::MySetFullScrean(DISPLAY_NUMBER, GC);

	// �|�W�p�^�[�����e & �B�e
	//start capturing

	std::cout << "�|�W�p�^�[���B�e��" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// ���e
		cv::imshow(GC, posi_img[i]);
		// �x���҂�
		cv::waitKey(2.0*delay);
		// �B�e
		//pgrOpenCV.queryFrame();
		kinect->updateColorFrame();
		// �B�e�摜��Mat�^�Ɋi�[
		//cv::Mat cap = pgrOpenCV.getVideo();
		cv::Mat cap = kinect->colorImage;
		//kinect�����獶�E���]
		flip(cap, cap, 1);
		// �B�e�̗l�q���`�F�b�N
		//pgrOpenCV.showCapImg(cap);
		imshow("colorImage", cap);

		// �|�W�p�^�[���B�e���ʂ�ۑ�
		// ����
		if(i < c->g.h_bit)
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << POSI << ".bmp"; 
		// �c��
		else
			Filename_posi_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << POSI << ".bmp"; 
		// �ۑ�
		//cv::imwrite(Filename_posi_cam[i].str(), pgrOpenCV.getVideo());
		cv::imwrite(Filename_posi_cam[i].str(), cap);
		Filename_posi_cam[i] << std::endl;
	}

	// �l�K�p�^�[�����e & �B�e
	std::cout << "�l�K�p�^�[���B�e��" << std::endl;
	for( unsigned int i = 0; i < c->g.all_bit; i++ ) {
		// ���e
		cv::imshow(GC, nega_img[i]);
		// �x���҂�
		cv::waitKey(2*delay);
		// �B�e
		//pgrOpenCV.queryFrame();
		kinect->updateColorFrame();
		// �B�e�摜��Mat�^�Ɋi�[
		//cv::Mat cap = pgrOpenCV.getVideo();
		cv::Mat cap = kinect->colorImage;
		//kinect�����獶�E���]
		flip(cap, cap, 1);
		// �B�e�̗l�q���`�F�b�N
		//pgrOpenCV.showCapImg(cap);
		imshow("colorImage", cap);
		// �|�W�p�^�[���B�e���ʂ�ێ�
		// ����
		if(i < c->g.h_bit)
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << HORIZONTAL << "_" << std::setw(2) << std::setfill('0') << i+1 << "_" << NEGA << ".bmp"; 
		// �c��
		else
			Filename_nega_cam[i] << "./GrayCodeImage/CaptureImage/CameraImg" << VERTICAL << "_" << std::setw(2) << std::setfill('0') << i-c->g.h_bit+1 << "_" << NEGA << ".bmp"; 
		////Filename_nega_cam[i] << "./output/Camera_nega" << std::setw(2) << std::setfill('0') << i << ".bmp";
		//cv::imwrite(Filename_nega_cam[i].str(), pgrOpenCV.getVideo());
		cv::imwrite(Filename_nega_cam[i].str(), kinect->colorImage);
		Filename_nega_cam[i] << std::endl;
	}
	/***** ���e & �B�e�I�� *****/


	cv::Mat src = cv::imread("./Penguins.jpg",1);
	cv::imshow(GC, src);
	cv::waitKey(2*delay);
	//pgrOpenCV.queryFrame();
	kinect->updateColorFrame();
	//cv::imwrite("./cap.jpg", pgrOpenCV.getVideo());
	cv::imwrite("./cap.jpg", kinect->colorImage);

	// �J�����I������
	//pgrOpenCV.stop();

	/**** �I�� *****/

	// �������̊J��
	delete[] posi_img;
	delete[] nega_img;
	delete[] Filename_posi;
	delete[] Filename_nega;
	delete[] Filename_posi_cam;
	delete[] Filename_nega_cam;
}


/***************
** ��l���֘A **
****************/

// �J�����B�e�摜��ǂݍ��ފ֐�
void GRAYCODE::loadCam(cv::Mat &mat, int div_bin, bool vh, bool pn)
{
	char buf[256];
	sprintf_s(buf, "./GrayCodeImage/CaptureImage/CameraImg%d_%02d_%d.bmp", vh, div_bin, pn);
	mat = cv::imread(buf, 0);
}

// �}�X�N���쐬����C���^�t�F�[�X
void GRAYCODE::makeMask(cv::Mat &mask)
{
	cv::Mat posi_img;
	cv::Mat nega_img;

	// �}�X�N�摜����
	cv::Mat mask_vert, mask_hor;
	static int useImageNumber = 6;
	// y�����̃O���C�R�[�h�摜�ǂݍ���
	loadCam(posi_img, useImageNumber, 0, 1);
	loadCam(nega_img, useImageNumber, 0, 0);

	// ���̃}�X�N�摜Y����
	makeMaskFromCam(posi_img, nega_img, mask_vert);

	// x�����̃O���C�R�[�h�摜�ǂݍ���
	loadCam(posi_img, useImageNumber, 1, 1);
	loadCam(nega_img, useImageNumber, 1, 0);

	// ���̃}�X�N�摜X����
	makeMaskFromCam(posi_img, nega_img, mask_hor);

	// X��Y��OR�����
	// �}�X�N�O�͂ǂ�������Ȃ̂ō�
	// �}�X�N���́i���_�I�ɂ́j�K����������ł�����������Ȃ̂ŁA���ɂȂ�
	// ���ۂ͂��܉��m�C�Y���c���Ă��܂�
	cv::bitwise_or(mask_vert, mask_hor, mask);

	// �c�������܉��m�C�Y�������i���S�}�����S�}���œK�p�����t�ɂȂ�j
	dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);
	erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 5);

	cv::imwrite("./GrayCodeImage/mask.bmp", mask);
}

// �O���C�R�[�h�̉摜�𗘗p���ă}�X�N�𐶐�����֐�
// �|�W�ƃl�K�̍����������thresholdValue�ȏ�̋P�x�̃s�N�Z���𔒂ɂ���
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

// �B�e�摜��2�l��������C���^�t�F�[�X
void GRAYCODE::make_thresh()
{
	cv::Mat posi_img;
	cv::Mat nega_img;
	cv::Mat Geometric_thresh_img;  // 2�l�����ꂽ�摜
	cv::Mat mask;

	// �}�X�N�𐶐�
	makeMask(mask);

	int h_bit = (int)ceil( log(PRJ_HEIGHT+1) / log(2) );
	int w_bit = (int)ceil( log(PRJ_WIDTH+1) / log(2) );
	int all_bit = h_bit + w_bit;

	std::cout << "��l���J�n" << std::endl;
	// �A�ԂŃt�@�C������ǂݍ���
	for( int i = 0; i < h_bit; i++ ) {
		// �ǂݍ���
		char buf[256];
		// �|�W�p�^�[���ǂݍ���
		loadCam(posi_img, i+1, 0, 1);
		// �l�K�p�^�[���ǂݍ���
		loadCam(nega_img, i+1, 0, 0);

		// 2�l��
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// �}�X�N��K�p����2�l��
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i);
		cv::imwrite(buf, masked_img);

		std::cout << i << ", ";
	}
	for( int i = 0; i < w_bit; i++ ) {
		// �ǂݍ���
		char buf[256];
		// �|�W�p�^�[���ǂݍ���
		loadCam(posi_img, i+1, 1, 1);
		// �l�K�p�^�[���ǂݍ���
		loadCam(nega_img, i+1, 1, 0);

		// 2�l��
		cv::Mat masked_img;
		thresh( posi_img, nega_img, Geometric_thresh_img, 0 );
		// �}�X�N��K�p����2�l��
		Geometric_thresh_img.copyTo( masked_img, mask );
		sprintf_s(buf, "./GrayCodeImage/ThresholdImage/Geometric_thresh%02d.bmp", i+h_bit);
		cv::imwrite(buf, masked_img);

		std::cout << i+h_bit << ", ";
	}
	std::cout << std::endl;
	std::cout << "��l���I��" << std::endl;
}

// ���ۂ�2�l������ 
void GRAYCODE::thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value )
{
	thresh_img = cv::Mat(posi.rows, posi.cols, CV_8UC1);
	for( int y = 0; y < posi.rows; y++ ) {
		for(int x = 0; x < posi.cols; x++ ) {
			int posi_pixel = posi.at<uchar>( y, x );
			int nega_pixel = nega.at<uchar>( y, x );

			// thresh_value���傫�����ǂ����œ�l��
			if( posi_pixel - nega_pixel >= thresh_value )
				thresh_img.at<uchar>( y, x ) = 255;
			else
				thresh_img.at<uchar>( y, x ) = 0;
		}
	}
}

/***********************************
** �v���W�F�N�^�ƃJ�����̑Ή��t�� **
************************************/

// 2�l���R�[�h����
void GRAYCODE::code_restore()
{
	// 2�l���R�[�h����
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

	// �A�z�z��ŃO���C�R�[�h�̒l�̏ꏊ�ɍ��W���i�[
	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ) {
			int a = c->graycode[y][x];
			if( a != 0 )
				(*c->code_map)[a] = cv::Point(x, y);
		}
	}

	// 0�Ԗڂ͎g��Ȃ�
	(*c->code_map)[0] = cv::Point(-1, -1);

	// �v���W�F�N�^�ƃJ�����̑Ή��t��
	for( int y = 0; y < PRJ_HEIGHT; y++ ) {
		for( int x = 0; x < PRJ_WIDTH; x++ ) {
			// �O���C�R�[�h�擾
			int a = c->g.graycode[y][x];
			// map���ɑ��݂��Ȃ��R�[�h�i�J�����ŎB�e����肭�����Ȃ����������j�̏ꏊ�ɂ̓G���[�l-1���i�[
			if ( (*c->code_map).find(a) == (*c->code_map).end() ) {
				c->CamPro[y][x] = cv::Point(-1, -1);
			}
			// ���݂���ꍇ�́A�Ή�����O���C�R�[�h�̍��W���i�[
			else {
				c->CamPro[y][x] = (*c->code_map)[a];
			}
		}
	}
}

// �אڂ����f���玝���Ă���
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

// ��f���
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

// �v���W�F�N�^ - �J�����\���̏�����
void GRAYCODE::initCorrespondence()
{
	initGraycode();

	for( int y = 0; y < CMR_HEIGHT; y++ ) {
		for( int x = 0; x < CMR_WIDTH; x++ ){
			c->graycode[y][x] = 0;
		}
	}
}

// �Ή��t�����s���C���^�[�t�F�[�X
void GRAYCODE::makeCorrespondence()
{
	initCorrespondence();
	code_restore();
	//	�⊮����on off
	//interpolation();
}


// �v���W�F�N�^-3�������W�ԑΉ��t�����s��
void GRAYCODE::makeCorrespondenceToWorld()
{
	//�ŐV�̃t���[�������g����CoordinateMapper
	kinect->updateColorFrame();
	kinect->updateDepthFrame();
	kinect->coordinateColorToCamera(c->CamPro, ProWorld, &validPointNum);

	//�f�o�b�O�p
	cout << "�Ή��_�擾�I��...\n" << endl;
	cout << "���_���F" << validPointNum << "�_\n" << endl;
}

//�Ή��_��p���ăp�����[�^����(6Points->�œK��)
void GRAYCODE::calcParameters(){

	//�L�����u���[�V����(6 points)
	std::cout << "*---------------------------------------------*" << std::endl;
	std::cout << "*       calibration by 6 points algrithm      *" << std::endl;

//	//�݊��ϊ�
//	changePoints(points_pro, points_3d);

	//six_points_calibration(points_3d, points_pro, pro_param.perspectiveMat);
	six_points_calibration_2(points_3d, points_pro, pro_param.perspectiveMat);

	//Mat�̃T�C�Y�����Ă��邩
	std::cout << "perspectiveMat = " << pro_param.perspectiveMat.size() << std::endl;

	//���ʂ̕\��
	std::cout << "*                result = " << 
		inspection_error_value(pro_param.perspectiveMat, points_3d, Size(800, 600), points_pro, false)
		<<std::endl; //Kinect�̐F�摜�T�C�Y�F1920�~1080

	//���ʂ̕ۑ�
	pro_param.alpha = 1.0;
	pro_param.beta = 0.0;
	saveCameraParams("../projector_paramater_6points.xml", pro_param);
	std::cout << "***********************************************" << std::endl;

}

void GRAYCODE::OptimizeParameters()
{
	//�L�����u���[�V�����i�p�����[�^��12�̍œK���j
	std::cout << "*---------------------------------------------*" << std::endl;
	std::cout << "*         calibration by optimization         *" << std::endl;
	//���Ή��_�S���˂����ނƂ��������Ԃ��|����̂ŁARANSAC�I�Ȋ����ō팸����K�v����
//	getper::get_perspectiveMat(points_pro, points_3d, pro_param.perspectiveMat, pro_param.perspectiveMat);
	getper::get_perspectiveMat(g_imagePointInlierSet, g_worldPointInlierSet, pro_param.perspectiveMat, pro_param.perspectiveMat);
	//���ʂ̕\��
	std::cout << "*                result = " << 
//		inspection_error_value(pro_param.perspectiveMat, points_3d, Size(800, 600), points_pro, false)
		inspection_error_value(pro_param.perspectiveMat, g_worldPointInlierSet, Size(800, 600), g_imagePointInlierSet, false)
		<<std::endl;
	//���ʂ̕ۑ�
	pro_param.alpha = 1.0;
	pro_param.beta = 0.0;
	saveCameraParams("../projector_paramater_optimize.xml", pro_param);
	std::cout << "***********************************************" << std::endl;
}

/***********************************
*** 6Points�֘A************
************************************/

//�݊��ϊ�����C���^�[�t�F�[�X
void GRAYCODE::PushPoints()
{
	validPointNum = 0;//������
	//�݊��ϊ��Ɣz��N���A
	changePoints(points_pro, points_3d);
}

//�݊��ϊ�
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

				//�_���v�b�V�������̂ŃN���A
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
		//points_3d�ɒl�������������Ă��Ȃ�
		//std::cout << "points_3d = " << points_3d << std::endl;

		get_six_points(calib_2d_points, calib_3d_points, points_2d, points_3d);
		//std::cout << "calib_2d_points = " << calib_2d_points << std::endl;
		//calib_3d_points�ɒl�������������Ă��Ȃ�
		//std::cout << "calib_3d_points = " << calib_3d_points << std::endl;


		six_points_algolism(calib_3d_points, calib_2d_points, pm);
		//std::cout << "pm�̒l�F" << pm.at<double>(0,0) << std::endl;
		//std::cout << "pm�̃T�C�Y�F" << pm.size() << std::endl;
		ave = inspection_error_value(pm, points_3d, Size(512, 424), points_2d, false);
		//std::cout << "ave = " << ave << std::endl;
		if(ave < min_ave){
			dst = pm;
			//std::cout << "dst.size = " << dst.size() << std::endl;
		}
	}
}

//�����_����6�_�𒊏o
void GRAYCODE::get_six_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P){
	int i=0;
	srand(time(NULL));    /* �����̏����� */ //rand()<32767
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

//���摜�̓����_�ƁA�Čv�Z���������_�̌덷�����߂�
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
		//�ē��e
		//circle(cretex::projectorImage, groundTruth[i], cretex::RADIUS, cv::Scalar(255,0,0), -1, CV_AA); //���� ��
		//circle(cretex::projectorImage, p[i], cretex::RADIUS, cv::Scalar(0,0,255), -1, CV_AA); //�ē��e ��
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
		//�ē��e
		Point2f reprojectPoint = ProjectTo(pro_param, g_worldPointInlierSet[i]);
		//�`��
		circle(reProjectImage, g_imagePointInlierSet[i], radius, cv::Scalar(255,0,0), -1, CV_AA); //���� ��
		circle(reProjectImage, reprojectPoint, radius, cv::Scalar(0,0,255), -1, CV_AA); //�ē��e ��
	}

	for(int i = 0; i < random_calib_2d_points.size(); i++)
	{
		//�����_����100�_��������
		circle(reProjectImage, random_calib_2d_points[i], radius+2, cv::Scalar(0,255,0), -1, CV_AA); //��
	}
}

Point2f GRAYCODE::ProjectTo(Params pro_param, Point3f world_point)
{
	//�@
	cv::Mat wp = (cv::Mat_<double>(4,1) << world_point.x, world_point.y, world_point.z, 1.0);

	Mat mult_result = pro_param.perspectiveMat * wp;
	Point2f dst;
	dst.x = mult_result.at<double>(0,0) / mult_result.at<double>(2,0);
	dst.y = mult_result.at<double>(1,0) / mult_result.at<double>(2,0);

	////�A
	//Mat cameraMat = pro_param.perspectiveMat;
	//dst.x = (cameraMat.at<double>(0,0)*world_point.x + cameraMat.at<double>(0,1)*world_point.y + cameraMat.at<double>(0,2)*world_point.z + cameraMat.at<double>(0,3))
	//	/ (cameraMat.at<double>(2,0)*world_point.x + cameraMat.at<double>(2,1)*world_point.y + cameraMat.at<double>(2,2)*world_point.z + cameraMat.at<double>(2,3));
	//dst.y = (cameraMat.at<double>(1,0)*world_point.x + cameraMat.at<double>(1,1)*world_point.y + cameraMat.at<double>(1,2)*world_point.z + cameraMat.at<double>(1,3))
	//	/ (cameraMat.at<double>(2,0)*world_point.x + cameraMat.at<double>(2,1)*world_point.y + cameraMat.at<double>(2,2)*world_point.z + cameraMat.at<double>(2,3));

	 return dst;
}


//RoomAlive���Q�l��RANSAC���g���Ă݂�
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
		bool nonCoplanar = false; //���ꕽ�ʏ�łȂ����ǂ���
		int nTries = 0;

		//1000�_��񓯈ꕽ�ʂɂȂ�悤�Ɏ���Ă���
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

		//6pointsAlgorithm��100points�ł��o�[�W����
		six_points_algolism_2(calib_3d_points, calib_2d_points, pm, 100);
//		error = inspection_error_value(pm, points_3d, Size(512, 424), points_2d, false);
		error = inspection_error_value(pm, calib_3d_points, Size(512, 424), calib_2d_points, false);
		cout << "error =" << error << endl;

		//���Ή��𒊏o
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
			 if(thisError < 1.0f) //�ē��e�덷�����e�͈͓��Ȃ�ΐ��Ή��_�Ƃ���
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
//****�����������������_���ɔ񕽖ʂœ_���擾
		//vector<Point2d> calib_2d_points_inlier;
		//vector<Point3d> calib_3d_points_inlier;
		//Mat pm_inlier;
		//bool nonCoplanar_inlier = false; //���ꕽ�ʏ�łȂ����ǂ���
		//int nTries_inlier = 0;

		////100�_��񓯈ꕽ�ʂɂȂ�悤�Ɏ���Ă���
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
			//���Ή��_���ׂĂ�6points
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

			////100�_�o�[�W����
			////�����ł������_���ɔ񕽖ʂœ_������Ă��Ȃ��ƃ_��
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

//�����_����1000�_�𒊏o
void GRAYCODE::get_hundred_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P){
	int i=0;
	//srand(time(NULL));    /* �����̏����� */ 
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

//points�𕽖ʃt�B�b�e�B���O
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
		//Outer(���ρH)
		for (int i = 0; i < M.rows; i++)
			for (int j = 0; j < M.cols; j++)
				M.at<double>(i,j) = pc.at<double>(i,0) * pc.at<double>(j,0);
				//M.data[i * M.cols + j] = pc.data[i] * pc.data[j];
		A += M;
	}
	Mat V(3, 3, CV_64F, Scalar::all(0)); //A�̌ŗL�x�N�g��
	Mat d(3, 1, CV_64F, Scalar::all(0));//A�̌ŗL�l
	eigen(A, d, V); //cv�̊֐�

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
		//�ŏ��ŗL�l�̃x�N�g������X�̂��̗�ɃR�s�[
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
** �Ή��_�ۑ� **
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
** �[�x������ **
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
** ���̑��i�p�r�s���ȉߋ��̈╨�j **
************************************/

// �摜�ό`�E����
// �J�����B�e�̈悩��v���W�F�N�^���e�̈��؂�o��
void GRAYCODE::transport_camera_projector(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // ���T�C�Y�����摜
	resize( src, src_resize, cv::Size(CMR_WIDTH, CMR_HEIGHT) );

	dst = cv::Mat( PRJ_HEIGHT, PRJ_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // �􉽕␳���ꂽ�摜�i���e�摜�j

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

// ���͉摜���J�����B�e�̈�ɕό`
void GRAYCODE::transport_projector_camera(cv::Mat &src, cv::Mat &dst)
{
	cv::Mat src_resize;  // ���T�C�Y�����摜
	resize( src, src_resize, cv::Size(PRJ_WIDTH, PRJ_HEIGHT) );

	dst = cv::Mat( CMR_HEIGHT, CMR_WIDTH, CV_8UC3, cv::Scalar(0, 0, 0) );  // �􉽕␳���ꂽ�摜�i���e�摜�j

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