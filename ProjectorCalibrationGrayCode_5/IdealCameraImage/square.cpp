#include "square.h"


/**
@brief �E�B���h�E����ݒ肷��
*/
void SQUARE::setWindowName(std::string windowName){
	this->windowName = windowName;
}

/**
@brief �}�X�N�摜��ݒ肷��֐�
@param filename �}�X�N�摜�i��l�摜�j�̃p�X
@param _showSizeRate �f�B�X�v���C�ɕ\������Ƃ��̑傫��
*/
void SQUARE::setMask(std::string filename, double showSizeRate)
{
	mask = cv::imread(filename, 0);
	orgWidth = mask.cols;
	orgHeight = mask.rows;
	this->showSizeRate = showSizeRate;
	cv::resize(mask, mask, cv::Size(), showSizeRate, showSizeRate, cv::INTER_AREA);

	// Xa<Xb, Ya<Yb�ɂȂ��ĂȂ��ƃG���[���o��̂œK���ɏ����l����Ă���
	Xa = 0;
	Xb = 10;
	Ya = 0;
	Yb = 10;
}

/**
@brief �J�[�\���L�[�ŋ�`�̈ʒu�𒲐߂��郁�C���̊֐�
@param size �ϊ�����摜�̃T�C�Y�i�A�X�y�N�g����ێ����邽�߂ɕK�v�j
@param searchDir �ϊ�����摜�������Ă���f�B���N�g����
*/
void SQUARE::adjustSquare(cv::Size size, std::string searchDir, std::string saveDir)
{
	std::cout << "�J�[�\���L�[�ŋ�`�̈ʒu�𒲐߂���Enter�������ĉ������B" << std::endl;
	std::cout << "�O��̍��W���g���ꍇ�̓J�[�\���L�[����������Enter�������ĉ������B" << std::endl;
	std::cout << "r�F�����_�������T��" << std::endl;
	std::cout << "q�F�I�����܂��B" << std::endl;
	std::cout << "S�F�w�肵���f�B���N�g�����̉摜���ꊇ�ϊ����ĕۑ����܂��B" << std::endl;

	cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
	cv::imshow(windowName, mask);
	autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(0, 0), 0.01, false);
	//randomSelectSquare(cv::Size(size.width, size.height), 0.0005, 500);

	// ��`�̒��S�ʒu
	int centX = round( (Xa + (Xb - Xa)*0.5) * showSizeRate );
	int centY = round( (Ya + (Yb - Ya)*0.5) * showSizeRate );
	int x = round( centX - mask.cols*0.5 );
	int y = round( centY - mask.rows*0.5 );

	// �J�[�\���L�[�������ɓ����s�N�Z����
	int step = 5;
	// �I���t���O
	bool exit = false;

	while(true){
		switch(cv::waitKey(0)){
		case 'q':
		case 'Q':
			return;

		case 2490368:	// �J�[�\���L�[�u���v
			y -= step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "���T�C�Y��𑜓x�F" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 2621440:	// �J�[�\���L�[�u���v
			y += step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "���T�C�Y��𑜓x�F" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 2555904:	// �J�[�\���L�[�u���v
			x += step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "���T�C�Y��𑜓x�F" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 2424832:	// �J�[�\���L�[�u���v
			x -= step;
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(x, y));
			std::cout << "���T�C�Y��𑜓x�F" << Xb-Xa << " x " << Yb-Ya << std::endl;
			break;

		case 'r':	// �ő��`�����ǉ��ōs���i���݂�菬�����Ȃ邱�Ƃ͂Ȃ��j
			randomSelectSquare(cv::Size(size.width, size.height), 0.001, 500);

			// ��`�̒��S�ʒu
			centX = round( (Xa + (Xb - Xa)*0.5) * showSizeRate );
			centY = round( (Ya + (Yb - Ya)*0.5) * showSizeRate );
			x = round( centX - mask.cols*0.5 );
			y = round( centY - mask.rows*0.5 );
			break;

		case 'c':	// ���S�ŋ�`����
			autoSelectSquare(0, cv::Size(size.width, size.height), cv::Point(0, 0));
			std::cout << "���T�C�Y��𑜓x�F" << Xb-Xa << " x " << Yb-Ya << std::endl;

			// ��`�̒��S�ʒu
			x = 0;
			y = 0;
			break;

		case 'd':	// �f�o�b�O�摜�\��
			debug = !debug;
			randomSelectSquare(cv::Size(size.width, size.height), 0.001, 500);
			// ��`�̒��S�ʒu
			centX = round( (Xa + (Xb - Xa)*0.5) * showSizeRate );
			centY = round( (Ya + (Yb - Ya)*0.5) * showSizeRate );
			x = round( centX - mask.cols*0.5 );
			y = round( centY - mask.rows*0.5 );
			break;

		case 'S':
			{
				// �ۑ��������W���擾
				loadData();
				// IMAGE_DIRECTORY���̉摜�t�@�C���������ׂĎ擾
				std::vector<std::string> file_list = getAllFilename(searchDir);
				int i = 0;

				// IMAGE_DIRECTORY���̉摜�����ׂĕϊ�
				std::cout << "�ϊ���" << std::endl;
				for (auto &path : file_list) {
					i++;
					// �摜�ǂݍ���
					char fileName[128];
					sprintf_s(fileName, "%s/%s", searchDir.c_str(), path.c_str());
					cv::Mat src = cv::imread(fileName, 1);
					// �ϊ����ĕۑ�
					char saveName[128];
					sprintf_s(saveName, "%s/sample%02d.png", saveDir.c_str(), i);
					saveImage(src, saveName);
					std::cout.fill('0');
					std::cout.width(2);
					std::cout << i << ":\t" << fileName << std::endl;
				}

				// �}�X�N�p�̉摜�쐬
				cv::Mat src = cv::Mat(size.height, size.width, CV_8UC3, cv::Scalar(255, 255, 0));
				saveImage(src, "./CalculationData/RectMask.bmp");

				std::cout << "complete!" << std::endl;
				exit = true;
			}
			break;

		default:
			cv::Mat src = cv::Mat(size.height, size.width, CV_8UC3, cv::Scalar(255, 255, 0));
			saveImage(src, "./CalculationData/RectMask.bmp");
			std::cout << "��`�̈�̎w�芮��" << std::endl;
			exit = true;
			break;
		}

		// while���[�v�𔲂���
		if(exit)
			break;
	}
	cv::destroyWindow(windowName);
}

/**
@brief ���S�ʒu�������_���Ɍ��߂ċ�`��������x���s���A�ő�T�C�Y�ɂȂ��`��T���B
@param size �ϊ�����摜�̃T�C�Y�i�A�X�y�N�g����ێ����邽�߂ɕK�v�j
@param shift ���S�ʒu�����炷��f���B
@param precision ���x�B�l���������قǍׂ������T�C�Y�����B
@param savePoint true�ō��W��ۑ�����B
@param num �ő��`�̐�����s����
*/
void SQUARE::randomSelectSquare(cv::Size size, double precision, int num){
	// ��������
	std::random_device rd;
	std::mt19937 mt(rd());
	double randomSizeRateX = 0.3;
	double randomSizeRateY = 0.3;
	std::uniform_int_distribution<int> randX(-round(mask.cols*randomSizeRateX), round(mask.cols*randomSizeRateX));
	std::uniform_int_distribution<int> randY(-round(mask.rows*randomSizeRateY), round(mask.rows*randomSizeRateY));

	cv::Mat random;
	// �����_���͈̔͂̎��o��
	if(debug){
		random = mask.clone();
		cv::rectangle(random, cv::Rect(mask.cols/2-round(mask.cols*randomSizeRateX), mask.rows/2-round(mask.rows*randomSizeRateY),
			round(mask.cols*randomSizeRateX*2), round(mask.rows*randomSizeRateY*2)), cv::Scalar(230, 230, 230), -1);
	}

	// �ŏ��̋�`���̔��̉�f���𐔂���
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

	// ��`����̃��[�v
	for (int j = 0; j < num; j++)
	{
		// �����_���ɍ��W������
		int x = randX(mt);
		int y = randY(mt);

		// �}�X�N�O�ɂȂ������蒼���iheight*rate=�\���p�Ƀ��T�C�Y�����摜�T�C�Y -> *0.5=���̉摜�̐^��, +y=�^�񒆂���ړ����������j
		uchar Y = mask.data[round(mask.rows*0.5+y)*mask.step + round(mask.cols*0.5+x)*mask.elemSize()];
		if(Y == 0){
			j--;
			continue;
		}

		// ���W�Ƀ}�[�N
		if(debug)
			cv::circle(random, cv::Point(round(mask.cols*0.5+x), round(mask.rows*0.5+y)), 2, cv::Scalar(128, 128, 128), -1, CV_AA);

		// �ő�ƂȂ�̈�̎n�_�ƏI�_���ċA�I�ɒT��
		int tmp = autoSelectSquare(pixels, size, cv::Point(x, y), precision, false);
		if(pixels < tmp)
			pixels = tmp;
	}
	drawResult(true);
	std::cout << "���T�C�Y��𑜓x�F" << Xb-Xa << " x " << Yb-Ya << std::endl;

	// �f�o�b�O�p�̉摜
	if(debug){
		char debugWindow[] = "�f�o�b�O�F�����_���ɍ��W���Ƃ����摜";
		cv::namedWindow(debugWindow, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
		cv::imshow(debugWindow, random);
		cv::imwrite("./�����_��.png", random);
	}
}

/**
@brief �}�X�N�̗̈���ōő�ƂȂ��`�̍���ƉE���̍��W�����߂�֐�<br>
�f�t�H���g�ł͉摜�̒��S���}�X�N�̒��S�Ƃ��ċ��߂�
@param minPixels ���܂�����`���̃s�N�Z���������̒l�ȉ��̏ꍇ�͖�������
@param size �ϊ�����摜�̃T�C�Y�i�A�X�y�N�g����ێ����邽�߂ɕK�v�j
@param shift ���S�ʒu�����炷��f���B
@param precision ���x�B�l���������قǍׂ������T�C�Y�����B
@param savePoint true�ō��W��ۑ�����B
@return ��`���̃s�N�Z����
*/
int SQUARE::autoSelectSquare(int minPixels, cv::Size size, cv::Point shift, double precision, bool savePoint){
	// �G���[����
	if( (abs(shift.x) >= round(mask.cols*0.5)) || (abs(shift.y) >= round(mask.rows*0.5)) )
		return minPixels;

	// �l���ɍ����܂܂�Ȃ��ő�T�C�Y�̋�`��T��
	cv::Mat rect;
	if(debug){
		rect = mask.clone();
		cv::cvtColor(rect, rect, cv::COLOR_GRAY2BGR);
	}

	// �G���[���o�Ȃ��悤�ɁutXa<tXb, tYa<tYb�v�ƂȂ�K���Ȓl������
	int tXa = 0;
	int tXb = 0;
	int tYa = 10;
	int tYb = 10;
	int i = 0;
	while(true){
		i++;
		// ��������`���`
		int sWidth = round(size.width * precision * i);
		int sHeight = round(size.height * precision * i);
		// �E�B���h�E�T�C�Y0���ƃG���[���o��̂ŏ���������ꍇ�͔�΂�
		// �傫���l�ɂ���قǃ��[�v�񐔂�����̂Ŏ኱�����Ȃ�
		if( (sWidth < 10) | (sHeight < 10) ){
			continue;
		}

		cv::Mat roi(mask, cv::Rect(mask.cols/2-sWidth/2+shift.x, mask.rows/2-sHeight/2+shift.y, sWidth, sHeight));

		// �S���̋�`��`�悵���摜���쐬
		if(debug){
			cv::rectangle(rect, cv::Rect(mask.cols/2-sWidth/2+shift.x, mask.rows/2-sHeight/2+shift.y, sWidth, sHeight), cv::Scalar(0, 0, 255), 1);
			cv::circle(rect, cv::Point(mask.cols/2+shift.x, mask.rows/2+shift.y), 2, cv::Scalar(0, 255, 0), -1, CV_AA);
		}

		// ��`�̎l����1�����ł������܂܂�Ă�����T���I��
		int leftUp = roi.at<uchar>(0, 0);
		int rightUp = roi.at<uchar>(0, roi.cols-1);
		int rightDown = roi.at<uchar>(roi.rows-1, roi.cols-1);
		int leftDown = roi.at<uchar>(roi.rows-1, 0);
		if( (leftUp == 0) || (rightUp == 0) || (rightDown == 0) || (leftDown == 0) )
			break;

		// �n�_�ƏI�_�̍��W����
		//Xa = round( (mask.cols/2 - sWidth/2 - shift.x) / showSizeRate );
		//Ya = round( (mask.rows/2 - sHeight/2 - shift.y) / showSizeRate );
		//Xb = round( (mask.cols/2 + sWidth/2 - shift.x) / showSizeRate );
		//Yb = round( (mask.rows/2 + sHeight/2 - shift.y) / showSizeRate );

		// �ꎞ�I�Ȏn�_�ƏI�_�̍��W����
		tXa = round( (mask.cols/2 - sWidth/2 + shift.x) );
		tYa = round( (mask.rows/2 - sHeight/2 + shift.y) );
		tXb = round( (mask.cols/2 + sWidth/2 + shift.x) );
		tYb = round( (mask.rows/2 + sHeight/2 + shift.y) );
	}

	// ��`���̔��̉�f���𐔂���
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

	// �O���葽�������ꍇ�͍��W���X�V
	if(minPixels < truePixels){
		Xa = round( tXa / showSizeRate );
		Ya = round( tYa / showSizeRate );
		Xb = round( tXb / showSizeRate );
		Yb = round( tYb / showSizeRate );
		drawResult(savePoint);
		// �f�o�b�O�p�̉摜
		if(debug){
			char debugWindow[] = "�f�o�b�O�F���X�ɑ傫���Ȃ��`�摜";
			cv::namedWindow(debugWindow, CV_WINDOW_AUTOSIZE | CV_WINDOW_KEEPRATIO);
			cv::imshow(debugWindow, rect);
			cv::imwrite("./��.png", rect);
		}
	}

	return truePixels;
}

/**
@brief ���ʂ̉摜��\�����Ēl��ۑ�����B
@param savePoint true�ō��W��ۑ�����B
*/
void SQUARE::drawResult(bool savePoint){
	// �f�B�X�v���C�\���p
	cv::Mat kakunin = mask.clone();
	cv::cvtColor(kakunin, kakunin, cv::COLOR_GRAY2BGR);
	cv::rectangle(kakunin, cv::Point2d(Xa*showSizeRate, Ya*showSizeRate), cv::Point2d(Xb*showSizeRate, Yb*showSizeRate), cv::Scalar(255, 255, 0), -1);
	// ���S
	cv::circle(kakunin, cv::Point2d(mask.cols/2, mask.rows/2), 4, cv::Scalar(160, 160, 200), -1);
	// ��`�̒��S
	cv::circle(kakunin, cv::Point2d(Xa*showSizeRate+(Xb-Xa)/2*showSizeRate, Ya*showSizeRate+(Yb-Ya)/2*showSizeRate), 4, cv::Scalar(0, 0, 200), -1);
	cv::imshow(windowName, kakunin);

	// ���ʂ�ۑ�
	if(savePoint){
		// ���T�C�Y����ƃJ�[�\���ŘA���I�ɓ������Ƃ��d���Ȃ�
		//cv::resize(kakunin, kakunin, cv::Size(), 1.0/showSizeRate, 1.0/showSizeRate);
		cv::imwrite("./SaveImage/RectMask�i�m�F�p�j.png", kakunin);
		std::ofstream ofs("./CalculationData/Rect_value.txt");
		ofs << Xa << "\t" << Ya << std::endl;
		ofs << Xb << "\t" << Yb << std::endl;
		ofs.close();
	}
}

/**
@brief �t�@�C������f�[�^��ǂݍ��ފ֐�
*/
bool SQUARE::loadData()
{
	std::ifstream ifs("./CalculationData/Rect_value.txt");

	if(ifs.fail()){
		std::cout << "LOAD ERROR�F�t�@�C�����ǂݍ��߂܂���ł���" << std::endl;
		return false;
	}

	int col = 1;	// �G���[���Ŏg�p
	char buf[256];
	float x[2], y[2];

	// 1�s���ǂݍ���
	while(ifs.getline(buf, 256)){
		// �R�����g�Ƌ�s�͓ǂݔ�΂�
		if( (strncmp(buf, "//", 2) == 0) || (strncmp(buf, "#", 1) == 0) || (strcmp(buf, "\0") == 0) ){
			col++;
			continue;
		}
		// �l��2������Ă���s�ȊO���ǂݍ��܂ꂽ��G���[
		if(sscanf_s(buf, "%f %f", &x[col-1], &y[col-1]) != 2){
			std::cout << "DATA LOAD ERROR(1)�F" << col << " �s�ڂ̏������Ԉ���Ă��܂��B" << std::endl;
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
@brief ���T�C�Y�����摜��ۑ�����֐�
@param src �ۑ�����摜
@param saveName �ۑ���̃t�@�C����
*/
void SQUARE::saveImage(cv::Mat &src, std::string saveName)
{
	cv::Mat dst, save;
	// ��`�T�C�Y�Ƀ��T�C�Y
	cv::Point center = resize(src, dst, cv::Point(Xa, Ya), cv::Point(Xb, Yb));
	// ��`���S�ɕ��i�ړ�
	imageShift(dst, save, center);
	// �}�X�N�T�C�Y�Ƀ��T�C�Y
	cv::resize(save, save, cv::Size(orgWidth, orgHeight));
	cv::imwrite(saveName, save);
}

/**
@brief �摜����i�ړ�������
@param src �ړ�����摜
@param dst �ړ���̉摜
@param center ���S���炸�炷����
*/
void SQUARE::imageShift(cv::Mat &src, cv::Mat &dst, cv::Point center)
{
	// ����ƍ���񂪍������͉摜���쐬
	cv::Mat tmp(cv::Size(src.cols+1, src.rows+1), CV_8UC3, cv::Scalar(0));
	cv::getRectSubPix(src, cv::Size(src.cols, src.rows), cv::Point(src.cols/2+1, src.rows/2+1), tmp);
	src = tmp.clone();
	cv::line(src, cv::Point(0, 0), cv::Point(src.cols-1, 0), cv::Scalar(0, 0, 0), 1);
	cv::line(src, cv::Point(0, 0), cv::Point(0, src.rows-1), cv::Scalar(0, 0, 0), 1);

	// ���w�i�ƍ���
	cv::Mat back = cv::Mat(orgHeight, orgWidth, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat roi(back, cv::Rect(0, 0, src.cols, src.rows));
	src.copyTo(roi);

	// ���S�ʒu�̕␳
	int offsetX = back.cols/2 - center.x;
	int offsetY = back.rows/2 - center.y;

	// ���i�ړ�
	cv::getRectSubPix(back, cv::Size(back.cols, back.rows), cv::Point(src.cols/2 + offsetX, src.rows/2 + offsetY), dst);
}

/**
@brief �摜����i�ړ������ă}�X�N��K�p����i���ʊm�F�p�j
@param src �ړ�����摜
@param dst �ړ���̉摜
@param center ���S���炸�炷����
*/
void SQUARE::imageShiftAdaptMask(cv::Mat &src, cv::Mat &dst, cv::Point center)
{
	// ����ƍ���񂪔������͉摜���쐬
	cv::Mat tmp(cv::Size(src.cols+1, src.rows+1), CV_8UC3, cv::Scalar(255));
	cv::getRectSubPix(src, cv::Size(src.cols, src.rows), cv::Point(src.cols/2+1, src.rows/2+1), tmp);
	src = tmp.clone();
	cv::line(src, cv::Point(0, 0), cv::Point(src.cols-1, 0), cv::Scalar(255, 255, 255), 1);
	cv::line(src, cv::Point(0, 0), cv::Point(0, src.rows-1), cv::Scalar(255, 255, 255), 1);

	// ���w�i�ƍ���
	cv::Mat back = cv::Mat(orgHeight, orgWidth, CV_8UC3, cv::Scalar(255, 255, 255));
	cv::Mat roi(back, cv::Rect(0, 0, src.cols, src.rows));
	src.copyTo(roi);

	// ���S�ʒu�̕␳
	int offsetX = back.cols/2 - center.x;
	int offsetY = back.rows/2 - center.y;

	// ���i�ړ�
	cv::getRectSubPix(back, cv::Size(back.cols, back.rows), cv::Point(src.cols/2 + offsetX, src.rows/2 + offsetY), dst);

	// �}�X�N��K�p
	cv::Mat _mask;
	cv::resize(mask, _mask, cv::Size(), 1.0/showSizeRate, 1.0/showSizeRate, cv::INTER_AREA);
	tmp = _mask.clone();
	dst.copyTo(tmp, _mask);
	dst = tmp.clone();
}

/**
@brief �Ίp���2�_���g���ă��T�C�Y���s��
@param src ���T�C�Y����摜
@param dst ���T�C�Y��̉摜
@param start ����̍��W
@param end �E���̍��W
@return ��`�̒��S���W
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

	// �w�肵����`���Ɏ��܂�悤�ɁA�k�����̑傫�������g��
	double r = r2;
	if(r1 < r2)
		r = r1;
	// �G���[����
	if(r == 0)
		r = 0.01;

	cv::resize(src, dst, cv::Size(), r, r, cv::INTER_AREA);
	return center;
}

/**
@brief ���ʃf�B���N�g�����̑S�t�@�C���̖��O���擾�i�ꊇ�ϊ��p�j
*/
std::vector<std::string> SQUARE::getAllFilename(std::string searchDir)
{
	std::vector<std::string> file_list;

	// �J�����g�f�B���N�g���ȉ��̃t�@�C�������擾����
	// �ċA�I�Ƀt�@�C�������擾����ꍇ�́Astd::tr2::sys::recursive_directory_iterator���g��
	for (std::tr2::sys::directory_iterator it(searchDir), end; it != end; ++it) {
		// �摜�t�@�C�������擾
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

	// �擾�����t�@�C���������ׂĕ\������
	//for (auto &path : file_list) {
	//	std::cout << path << std::endl;
	//}
	return file_list;
}

/**
@brief setMouseCallback�Őݒ肵���R�[���o�b�N�֐����ŌĂԊ֐�<br>
�}�E�X�Œ��ɋ�`���w�肷��
@warning �����ŏo����悤�ɂ����̂ŕs�v�ɂȂ������ǈꉞ�c���Ă���
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
		// �N���b�N�������W�ɓ_��`��
		isLButton = true;
		Xa = x*2;
		Ya = y*2;
		cv::circle(referenceMat, cv::Point(Xa/2, Ya/2), ptSize, cv::Scalar(0, 0, 200), -1);
		cv::imshow(windowName, referenceMat);
		break;

	case cv::EVENT_MOUSEMOVE:
		// �h���b�O���ɋ�`�Ȃǂ�`��
		if(isLButton){
			// �n�_�A�I�_�A���S�̍��W
			cv::circle(referenceMat, cv::Point(Xa/2, Ya/2), ptSize, cv::Scalar(0, 0, 200), -1);
			cv::circle(referenceMat, cv::Point(x, y), ptSize, cv::Scalar(0, 0, 200), -1);
			cv::circle(referenceMat, cv::Point(x-(x-Xa/2)/2, y-(y-Ya/2)/2), ptSize, cv::Scalar(0, 0, 200), -1);
			// ��`
			cv::rectangle(referenceMat, cv::Point(Xa/2, Ya/2), cv::Point(x, y), cv::Scalar(0, 0, 200));
			cv::imshow(windowName, referenceMat);
		}
		break;

	case cv::EVENT_LBUTTONUP:
		// ���T�C�Y�Ǝw�肵�����W�̕ۑ����s��
		isLButton = false;
		Xb = x*2;
		Yb = y*2;
		cv::Mat src = cv::Mat(orgHeight, orgWidth, CV_8UC3, cv::Scalar(255, 255, 0));
		cv::Mat dst, kakunin, save;
		// Xa, Ya, Xb, Yb, center��ۑ����Ă����Έꊇ�Ń��T�C�Y���ʒu���킹�ł���
		cv::Point center = resize(src, dst, cv::Point(Xa, Ya), cv::Point(Xb, Yb));
		imageShiftAdaptMask(dst, kakunin, center);
		cv::imwrite("resize.bmp", kakunin);
		cv::resize(kakunin, kakunin, cv::Size(), showSizeRate, showSizeRate, cv::INTER_AREA);

		// �n�_�A�I�_�A���S�̍��W
		cv::circle(kakunin, cv::Point(Xa/2, Ya/2), ptSize, cv::Scalar(0, 0, 200), -1);
		cv::circle(kakunin, cv::Point(Xb/2, Yb/2), ptSize, cv::Scalar(0, 0, 200), -1);
		cv::circle(kakunin, cv::Point(Xb/2-(Xb/2-Xa/2)/2, Yb/2-(Yb/2-Ya/2)/2), ptSize, cv::Scalar(0, 0, 200), -1);
		// ��`
		cv::rectangle(kakunin, cv::Point(Xa/2, Ya/2), cv::Point(Xb/2, Yb/2), cv::Scalar(0, 0, 200));
		cv::imshow(windowName, kakunin);
		imageShift(dst, kakunin, center);
		cv::imwrite("resize2.bmp", kakunin);

		// ���ʂ�ۑ�
		std::ofstream ofs("./CalculationData/Rect_value.txt");
		ofs << Xa << "\t" << Ya << std::endl;
		ofs << Xb << "\t" << Yb << std::endl;
		ofs.close();
		break;
	}
}
