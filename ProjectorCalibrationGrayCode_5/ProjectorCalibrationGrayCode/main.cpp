/**
@file main.cpp
@mainpage
�P�x�␳���s���v���O����<br>
@section �ȒP�ȏ����̗���
- ���O�ɃK���}�l��RGB���Ƃɑ��肵�Ă����i��x���肷��΁A�v���W�F�N�^���J������ς��Ȃ�������j
- �O���C�R�[�h���g���ăv���W�F�N�^�ƃJ�����̊e��f��Ή��t��
- �}�X�N�i�v���W�F�N�^���e�̈�j�ɍ��킹�ē��e����摜��ϊ��i����ɂ͔�Ή��j
- �Ή��t�����֌W���P�x�␳�p�̃N���X�ɓn��
- �ŏ��P�x�摜�ƍő�P�x�摜���B�e�i�_�C�i�~�b�N�����W�v�Z�p�j
- �����֐��𐄒�
- �����֐����g���ēǂݍ��񂾉摜�̖ڕW�摜���쐬

�ȍ~�́A�␳���@�ɂ���ĈقȂ�B<br>
@section �����֐���ω������Ȃ��ꍇ
- �J�����B�e�摜���ڕW�摜�ɋ߂��Ȃ�悤�ɓ��e�f����ϊ�
���t���[���ڕW�摜��ς��邱�Ƃœ���Ή��\<br>
���̏ꍇ�A�␳���ɃJ�����͕s�v<br>
���O�Ɍv�������K���}�l�����m�ȂقǕ␳���Y��ɍs����B

@section �����֐���ω�������ꍇ
- �J�����B�e�摜���甽�˗��i�����֐��̐���ɕK�v�ȃp�����[�^�j���v��
- LMS�A���S���Y�����g���ď��X�ɐ^�̔��˗��i�^�̉����֐��j�֋߂Â��Ă���
������x���[�v���i�ނƔ��˗��͎�������B<br>
�����ɕω������Ȃ�����ł���Δ��˗��̕ω��͏��Ȃ����߁A���̂܂ܓ���Ή����\<br>
�������傫���ꍇ��V�[���̐؂�ւ����Ȃǂ͎c�����c���Ă��܂��B<br>
�����֐���^�̒l�ɋ߂Â��Ă������߁A�K���}�l�͂�����x��G�c�ł����R�ƏC������ď�肭�����B

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
#define GRAYCODE_NUM (5) //�O���C�R�[�h���J��Ԃ����e�����

int main()
{
	myKinect *mykinect = new myKinect();
	GRAYCODE *gc = new GRAYCODE(mykinect);

	cv::Mat src = cv::imread("./cap.jpg",1);
	cv::Mat src2 = cv::imread("./geo.bmp",1); //�􉽕␳��̌��������J�����摜(square.cpp/.h�ō��)
	cv::Mat dst;

	printf("0�F�O���C�R�[�h���e&��l��\n");
	printf("1�F�Ή��t��\n");
	printf("2�F�p�����[�^����(6 points algolism)\n");
	printf("3�F�p�����[�^����(optimization)\n");
	printf("4: GrayCode�􉽕␳\n");
	printf("5: �_�ē��e\n");
	printf("6: GrayCode �A�����e�J�n\n");
	printf("w�F�ҋ@���ɔ��摜�𓊉e���邩���Ȃ���\n");
	printf("\n");

	mykinect->init();

	static bool prjWhite = true;

	// �L�[���͎�t�p�̖������[�v
	while(true){
		printf("====================\n");
		printf("��������͂��Ă�������....\n");
		int command;

		// �����摜��S��ʂœ��e�i�B�e�����m�F���₷�����邽�߁j
		//���C�����[�v
		while(true){
			// true�Ŕ��𓊉e�Afalse�Œʏ�̃f�B�X�v���C��\��
			if(prjWhite){
				cv::Mat white = cv::Mat(PROJECTOR_WIDTH, PROJECTOR_HEIGHT, CV_8UC3, cv::Scalar(255, 255, 255));
				cv::namedWindow("white_black", 0);
				Projection::MySetFullScrean(DISPLAY_NUMBER, "white_black");
				cv::imshow("white_black", white);
			}

			// �����̃L�[�����͂��ꂽ�烋�[�v�𔲂���
			command = cv::waitKey(33);
			if ( command > 0 ) break;

			mykinect->updateColorFrame();
			mykinect->updateDepthFrame();

			//�摜�\��
			mykinect->draw();
		}

		cv::destroyWindow("white_black");

		// ��������
		switch (command){
		case '0':
			gc->code_projection();
			gc->make_thresh();
			break;

		case '1':
			gc->makeCorrespondence(); //�v���W�F�N�^-�J�����Ή��t��
			gc->makeCorrespondenceToWorld();//�v���W�F�N�^-3�������W�Ή��t��
			gc->PushPoints(); //�_���v�b�V��&�z��N���A
			break;

		case '2':
			gc->calcParameters();//�p�����[�^����J�n
			break;

		case '3':
			gc->OptimizeParameters();//�œK��
			break;

		case '4':
			//�􉽑Ή��t���Ŋm�F
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
			//����Ɏg�����_���ē��e
			cv::namedWindow("reProjection", 0);
			gc->reProjectPoints(RADIUS);
			//�t���X�N���[���\��
			Projection::MySetFullScrean(DISPLAY_NUMBER, "reProjection");
			cv::imshow("reProjection", gc->reProjectImage);
			waitKey(0);
			break;

		case '6':
			cout << GRAYCODE_NUM << "�񓊉e���܂��B\n�����e��Aany key�Ŏ��ɐi�߂Ă�������." << endl;
			int current = 0;
			int key = 0;
			while(current < GRAYCODE_NUM)
			{
				current++;
				//GrayCode���e�A��l��
				gc->code_projection();
				gc->make_thresh();
				//�Ή��t��
				cout << "�Ή��t���J�n." << endl;

				gc->makeCorrespondence(); //�v���W�F�N�^-�J�����Ή��t��
				gc->makeCorrespondenceToWorld();//�v���W�F�N�^-3�������W�Ή��t��
				gc->PushPoints(); //�_���v�b�V��&�z��N���A

				//�����܂ł̎擾�_��
				cout << current <<  "��ڂ܂ł̎擾�_���F" << gc->points_3d.size() << "�_" << endl;

				cout << "�Ή��t���I��.\n���̏������ł����牽���L�[�������Ă�������..." << endl;
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
