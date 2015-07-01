#ifndef SQUARE_H
#define SQUARE_H

//#include "main.h"
#include <cmath>
#include <random>
#include <fstream>
#include <filesystem>

#include <opencv2\opencv.hpp>

/**
@brief �􉽕␳�p�̉摜���쐬����N���X
*/
class SQUARE{
public:
	SQUARE(std::string windowName = "���ڃ��T�C�Y")
	{
		this->windowName = windowName;
		pixels = 0;
		debug = false;
	}

	~SQUARE()
	{
	}

	std::string windowName;
	cv::Mat mask;	//!< �g�p����}�X�N�摜�isetMask�œǂݍ��ށj
	int Xa, Ya;	//!< �n�_�̍��W
	int Xb, Yb;	//!< �I�_�̍��W

	/// �����Ɋۂ߂�
	int round(double d)
	{
		return (int)(d+0.5);
	}

	// �}�X�N�摜��ǂݍ���
	void setMask(std::string filename, double showSizeRate = 0.5);
	// �E�B���h�E����ݒ�
	void setWindowName(std::string windowName);
	// �����I�ɍ��W�����肷��
	int autoSelectSquare(int minPixels, cv::Size size, cv::Point shift = cv::Point(0, 0), double precision = 0.005, bool savePoint = true);
	void randomSelectSquare(cv::Size size, double precision = 0.005, int num = 100);
	// ���ʉ摜��\��
	void drawResult(bool savePoint = true);
	// �J�[�\���L�[�ŋ�`���̈ʒu�𒲐߂���
	void adjustSquare(cv::Size size, std::string searchDir = ".", std::string saveDir = ".");
	// �t�@�C��������W��ǂݍ���
	bool loadData();
	// �摜��ۑ�����
	void saveImage(cv::Mat &src, std::string saveName);
	// �}�E�X�̃R�[���o�b�N�֐�
	void onMouse( int event, int x, int y, int flag, void* param);
	// ���T�C�Y�����摜���ʒu���킹�{�}�X�N�K�p�i���ʊm�F�p�j
	void imageShiftAdaptMask(cv::Mat &src, cv::Mat &dst, cv::Point center = cv::Point(0, 0));
	// ���T�C�Y�����摜���ʒu���킹
	void imageShift(cv::Mat &src, cv::Mat &dst, cv::Point center = cv::Point(0, 0));
	// �w�肵����`�̃T�C�Y�Ƀ��T�C�Y
	cv::Point resize(cv::Mat &src, cv::Mat &dst, cv::Point start, cv::Point end);

private:
	double showSizeRate;	//!< �}�X�N�摜���f�B�X�v���C�ɕ\������Ƃ��̃T�C�Y�̊���
	int orgWidth, orgHeight;	//!< �ǂݍ��ݎ��̃}�X�N�摜�̃T�C�Y�i���T�C�Y���Ă��܂��̂Œl��ۑ����Ă����j
	int pixels;	//!< ��`���̃s�N�Z����
	bool debug;	//!< �f�o�b�O�p�摜�̕\��

	// ���ʃf�B���N�g�����̑S�t�@�C���̖��O���擾
	std::vector<std::string> getAllFilename(std::string searchDir);
};

#endif