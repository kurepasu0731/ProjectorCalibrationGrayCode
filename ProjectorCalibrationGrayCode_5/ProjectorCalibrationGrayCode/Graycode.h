#ifndef GRAYCODE_H
#define GRAYCODE_H

#pragma once

// GrayCode��p�����􉽕␳

// �O���C�R�[�h�́C�c�C���ʂɍ쐬���C��ō���
// �p�^�[���摜�̓r�b�g���Z��p���č쐬�i������char�^�͎g��Ȃ��j
// �p�^�[���摜��1������������ŕۑ�
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>  // ������X�g���[��
#include <direct.h> // create dir
#include <math.h>
#include <random>
#include "myKinect.h"

typedef struct {
	Size imageSize;
	Mat perspectiveMat;		//�������e�s��
	Mat internalMat;		//�����s��
	Mat translationalMat;	//���i�s��
	Mat rotationMat;		//��]�s��
	Mat distCoffs;			//�c�݌W��
	double alpha;
	double beta;
} Params;

/** 
@brief GrayCode��p�����􉽕␳<br>
�Ή��t�����Ȃ�������f�ɂ͋ߖT�̉�f���R�s�[���ĕ��<br>
�ŏI�I�ȃv���W�F�N�^�ƃJ������Ή��t�����z���c->CamPro�Ɋi�[�����B
*/
class GRAYCODE{
public:
	// �v���W�F�N�^�𑜓x
	static const int PRJ_WIDTH = PROJECTOR_WIDTH;
	static const int PRJ_HEIGHT = PROJECTOR_HEIGHT;
	static const int CMR_WIDTH = CAMERA_WIDTH;
	static const int CMR_HEIGHT = CAMERA_HEIGHT;
	static const int PRJ_X = DISPLAY_WIDTH;
	static const int PRJ_Y = DISPLAY_HEIGHT;

	// �O���C�R�[�h�쐬�ɕK�v�ȍ\����
	typedef struct _Graycode {
		int graycode[PRJ_HEIGHT][PRJ_WIDTH];  // �O���C�R�[�h�i�v���W�F�N�^�𑜓x[����][��]�j
		unsigned int h_bit, w_bit;     // �����C���̕K�v�r�b�g��
		unsigned int all_bit;          // ���v�r�b�g���ih_bit + w_bit�j
	} Graycode;

	// �v���W�F�N�^ - �J�����Ή��ɕK�v�ȍ\����
	typedef struct _correspondence {
		int graycode[CMR_HEIGHT][CMR_WIDTH];  // 2�l���R�[�h�𕜌��������̂��J������f�Ɋi�[
		cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH];  // �v���W�F�N�^��f�ɑ΂���J�����Ή���f
		std::map<int, cv::Point> *code_map;
		Graycode g;
	} correspondence;

	correspondence *c; //�v���W�F�N�^-�J�����ԑΉ�

	cv::Point3f ProWorld[PRJ_HEIGHT][PRJ_WIDTH]; //�v���W�F�N�^-3�������W�Ή�
	int validPointNum; //�L���ȑΉ��_��
	Params pro_param; //�v���W�F�N�^�p�����[�^

	//�S�Ή��_
	vector<Point2d> points_pro;//�v���W�F�N�^�摜
	vector<Point3d> points_3d;//�O�������W

	//���Ή��_
	vector<Point2d> g_imagePointInlierSet;
	vector<Point3d> g_worldPointInlierSet;

	//�ŏ��Ƀ����_����100�_��������
	vector<Point2d> random_calib_2d_points;
	vector<Point3d> random_calib_3d_points;

	//�ē��e�p�摜
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
	// �p�^�[���R�[�h���e & �B�e
	void code_projection();
	// 2�l��
	void make_thresh();
	// ������
	void makeCorrespondence();
	// �v���W�F�N�^-3�������W�ԑΉ��t�����s���C���^�[�t�F�[�X
	void makeCorrespondenceToWorld();
	//�_���݊��ϊ�����C���^�[�t�F�[�X
	void GRAYCODE::PushPoints();
	//�p�����[�^����J�n
	void calcParameters();
	//�œK��
	void OptimizeParameters();

	//// �摜�ό`�E����
	//// �J�����B�e�̈悩��v���W�F�N�^���e�̈��؂�o��
	void transport_camera_projector(cv::Mat &src, cv::Mat &dst);
	//// ���͉摜���J�����B�e�̈�ɕό`
	void transport_projector_camera(cv::Mat &src, cv::Mat &dst);

	//�Ή��_���`���ŕۑ�
	void save_g_worldPointInlierSet();
	void load_g_worldPointInlierSet();

private:
	// �E�B���h�E�l�[��
	char* GC;
	char* MP;
	float SHUTTER;	// �V���b�^�[���x
	double delay;

	Graycode *g;

	myKinect *kinect;

	/// �O���C�R�[�h�̍쐬�֘A
	// �J�����̏�����
	void initCamera();
	// �O���C�R�[�h�쐬
	void initGraycode();
	// �p�^�[���R�[�h�摜�쐬
	void makeGraycodeImage();
	// �f�B���N�g���̍쐬
	void createDirs();
	/// ��l���֘A
	// �J�����B�e�摜��ǂݍ��ފ֐�
	void loadCam(cv::Mat &mat, int div_bin, bool flag, bool pattern);
	// �ŏI�I�Ɏg�p����}�X�N�𐶐�����֐�
	void makeMask(cv::Mat &mask);
	// �O���C�R�[�h�̉摜�𗘗p���ă}�X�N�𐶐�����֐�
	// �|�W�ƃl�K�̍����������MASK_THRESH�ȏ�̋P�x�̃s�N�Z���𔒂ɂ���
	void makeMaskFromCam(cv::Mat &posi, cv::Mat &nega, cv::Mat &result, int thresholdValue = 25);
	// 2�l�������֐� 
	void thresh( cv::Mat &posi, cv::Mat &nega, cv::Mat &thresh_img, int thresh_value );

	//�p�����[�^����֘A
	//6 points algorithm
	void six_points_calibration(vector<Point3d>& points_3d, vector<Point2d>& points_2d, Mat& dst);
	//RANSAC�o�[�W����
	void six_points_calibration_2(vector<Point3d>& points_3d, vector<Point2d>& points_2d, Mat& dst);
	//�����_����100�_�𒊏o
	void get_hundred_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P);
	//�݊��ϊ�
	void changePoints(vector<Point2d>& points_pro, vector<Point3d>& points_3d);
	//�����_����6�_�𒊏o
	void get_six_points(vector<Point2d>& calib_p, vector<Point3d>& calib_P, vector<Point2d>& src_p, vector<Point3d>& src_P);
	//���摜�̓����_�ƁA�Čv�Z���������_�̌덷�����߂�
	double inspection_error_value(Mat& cameraMat, vector<Point3d>& P, Size& imageSize, vector<Point2d>& groundTruth, bool isDraw);
	//���ʃt�B�b�e�B���O
	double PlaneFit(vector<Point3d>& points, Mat& X, double* D);

	/// ���̑�
	// �v���W�F�N�^ - �J�����\���̏�����
	void initCorrespondence();
	// 2�l���R�[�h����
	void code_restore();
	//// ��f��Ԏ�@
	// �אڂ����f���玝���Ă���
	cv::Point getInterpolatedPoint2(int y, int x, cv::Point CamPro[PRJ_HEIGHT][PRJ_WIDTH]);
	// ��f���
	void interpolation();
};


#endif