#pragma once

#include <stdio.h>
#include <tchar.h>
#include <string>
#include <math.h>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;

namespace getper{ 
	const int C11 = 0;
	const int C12 = 1;
	const int C13 = 2;
	const int C14 = 3;
	const int C21 = 4;
	const int C22 = 5;
	const int C23 = 6;
	const int C24 = 7;
	const int C31 = 8;
	const int C32 = 9;
	const int C33 = 10;
	const int C34 = 11;

	const int xi = 0;
	const int yi = 1;
	const int Xi = 2;
	const int Yi = 3;
	const int Zi = 4;


	class Function{
	public:
		Function(){}
		double virtual func(Mat& c, Mat& m){return 0;}
		double virtual differentiation(Mat& c, Mat& m, int type){return 0;}
	};

	class Function_g11:Function{
	public:
		Function_g11(){}
		double virtual func(Mat& c, Mat& m){
			double re = c.at<double>(C11,0)*m.at<double>(Xi,0)+c.at<double>(C12,0)*m.at<double>(Yi,0)+c.at<double>(C13,0)*m.at<double>(Zi,0)+c.at<double>(C14,0);
			//std::cout << "g11 = " << re << std::endl;
			return re;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			if(type < C11 || type > C34){
				std::cout << "error : gauss newton" << endl;
				exit(0);
			}
			if(type == C11){
				//std::cout << "diff g11/C11 = " << m.at<double>(Xi,0) << std::endl;
				return m.at<double>(Xi,0);
			}else if(type == C12){
				//std::cout << "diff g11/C12 = " <<m.at<double>(Yi,0) << std::endl;
				return m.at<double>(Yi,0);
			}else if(type == C13){
				//std::cout << "diff g11/C13 = " <<m.at<double>(Zi,0) << std::endl;
				return m.at<double>(Zi,0);
			}else if(type == C14){
				//std::cout << "diff g11/C14 = 1.0" << std::endl;
				return 1.0;
			}else{
				//std::cout << "diff g11/" << type << " = 0.0" << std::endl;
				return 0;
			}
		}
	};

	class Function_g12:Function{
	public:
		Function_g12(){}
		double virtual func(Mat& c, Mat& m){
			double re = c.at<double>(C31,0)*m.at<double>(Xi,0)+c.at<double>(C32,0)*m.at<double>(Yi,0)+c.at<double>(C33,0)*m.at<double>(Zi,0)+c.at<double>(C34,0);
			//std::cout << "g12 = " << re << std::endl;
			/*std::cout << "g2" << " = " << c.at<double>(C31,0)*m.at<double>(Xi,0) << " + " << 
				c.at<double>(C32,0)*m.at<double>(Yi,0) << " + "
				<< c.at<double>(C33,0)*m.at<double>(Zi,0) << "+" 
				<< c.at<double>(C34,0) << " = " 
				<< re << std::endl;*/
			//std::cout << "C31 = " << c.at<double>(C31,0) << " X = " << m.at<double>(Xi,0) << std::endl;
			return re;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			if(type < C11 || type > C34){
				std::cout << "error : gauss newton" << endl;
				exit(0);
			}
			if(type == C31){
				//std::cout << "diff g12/C31 = " << m.at<double>(Xi,0) << std::endl;
				return m.at<double>(Xi,0);
			}else if(type == C32){
				//std::cout << "diff g12/C32 = " << m.at<double>(Yi,0) << std::endl;
				return m.at<double>(Yi,0);
			}else if(type == C33){
				//std::cout << "diff g12/C33 = " << m.at<double>(Zi,0) << std::endl;
				return m.at<double>(Zi,0);
			}else if(type == C34){
				//std::cout << "diff g12/C34 = 1.0" << std::endl;
				return 1.0;
			}else{
				//std::cout << "diff g12/" << type << " = 0.0" << std::endl;
				return 0;
			}
		}
	};

	class Function_g21:Function{
	public:
		Function_g21(){}
		double virtual func(Mat& c, Mat& m){
			double re = c.at<double>(C21,0)*m.at<double>(Xi,0)+c.at<double>(C22,0)*m.at<double>(Yi,0)+c.at<double>(C23,0)*m.at<double>(Zi,0)+c.at<double>(C24,0);
			//std::cout << "g21 = " << re << std::endl;
			return re;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			if(type < C11 || type > C34){
				std::cout << "error : gauss newton" << endl;
				exit(0);
			}
			if(type == C21){
				//std::cout << "diff g21/C21 = " << m.at<double>(Xi,0) << std::endl;
				return m.at<double>(Xi,0);
			}else if(type == C22){
				//std::cout << "diff g21/C22 = " << m.at<double>(Yi,0) << std::endl;
				return m.at<double>(Yi,0);
			}else if(type == C23){
				//std::cout << "diff g21/C23 = " << m.at<double>(Zi,0) << std::endl;
				return m.at<double>(Zi,0);
			}else if(type == C24){
				//std::cout << "diff g21/C24 = 1.0" << std::endl;
				return 1.0;
			}else{
				//std::cout << "diff g21/" << type << " = 0.0" << std::endl;
				return 0;
			}
		}
	};

	class Function_G1:Function{
	private:
		Function_g11 g1;
		Function_g12 g2;
	public:
		Function_G1(){}
		double virtual func(Mat& c, Mat& m){
			double re = g1.func(c,m)/g2.func(c,m);
			//std::cout << "G1 = " << re << std::endl;
			return re;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			double term1 = g1.differentiation(c,m,type)*g2.func(c,m);
			double term2 = g1.func(c,m)*g2.differentiation(c,m,type);
			double term3 = pow(g2.func(c,m),2.0);
			//std::cout << "diff G1/" << type << " = (" << term1 << " - " << term2 << " )/ " << term3 << " = " << (term1-term2)/term3 << std::endl;
			return (term1-term2)/term3;
		}
	};

	class Function_G2:Function{
	private:
		Function_g21 g1;
		Function_g12 g2;
	public:
		Function_G2(){}
		double virtual func(Mat& c, Mat& m){
			double re = g1.func(c,m)/g2.func(c,m);
			//std::cout << "G2 = " << re << std::endl;
			return re;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			double term1 = g1.differentiation(c,m,type)*g2.func(c,m);
			double term2 = g1.func(c,m)*g2.differentiation(c,m,type);
			double term3 = pow(g2.func(c,m),2.0);
			//std::cout << "diff G1/" << type << " = (" << term1 << " - " << term2 << " )/ " << term3 << " = " << (term1-term2)/term3 << std::endl;
			return (term1-term2)/term3;
		}
	};

	class Function_F1:Function{
	private:
		Function_G1 g;
	public:
		Function_F1(){}
		double virtual func(Mat& c, Mat& m){
			double term1 = pow(m.at<double>(xi,0),2.0);
			double term2 = 2*m.at<double>(xi,0)*g.func(c,m);
			double term3 = pow(g.func(c,m),2.0);
			//std::cout << "F1 = " << term1 << " - " << term2 << " + " << term3 << " = " << term1-term2+term3 << std::endl;
			return term1-term2+term3;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			double term1 = -1*2*m.at<double>(xi,0)*g.differentiation(c,m,type);
			double term2 = g.differentiation(c,m,type)*g.func(c,m);
			double term3 = g.differentiation(c,m,type)*g.func(c,m);
			//std::cout << "diff F1/" << type << " = " << term1 << " + " << term2 << " + " << term3 << " = " << term1+term2+term3 << std::endl;
			return term1+term2+term3;
		}
	};

	class Function_F2:Function{
	private:
		Function_G2 g;
	public:
		Function_F2(){}
		double virtual func(Mat& c, Mat& m){
			double term1 = pow(m.at<double>(yi,0),2.0);
			double term2 = 2*m.at<double>(yi,0)*g.func(c,m);
			double term3 = pow(g.func(c,m),2.0);
			//std::cout << "F2 = " << term1 << " - " << term2 << " + " << term3 << " = " << term1-term2+term3 << std::endl;
			return term1-term2+term3;
		}
		double virtual differentiation(Mat& c, Mat& m, int type){
			double term1 = -1*2*m.at<double>(yi,0)*g.differentiation(c,m,type);
			double term2 = g.differentiation(c,m,type)*g.func(c,m);
			double term3 = g.differentiation(c,m,type)*g.func(c,m);
			//std::cout << "diff F2/" << type << " = " << term1 << " + " << term2 << " + " << term3 << " = " << term1+term2+term3 << std::endl;
			return term1+term2+term3;
		}
	};

	Vector<Mat> create_measurementParam(vector<Point2d>& p, vector<Point3d>& P){
		Vector<Mat> re;
		if((int)p.size() != (int)P.size()){
			std::cout << "error:gauss newton 2!" << std::endl;
			exit(0);
		}
		for(int i=0; i<(int)p.size(); i++){
			Mat m(5, 1, CV_64FC1, Scalar::all(0));
			m.at<double>(xi,0) = p[i].x;
			m.at<double>(yi,0) = p[i].y;
			m.at<double>(Xi,0) = P[i].x;
			m.at<double>(Yi,0) = P[i].y;
			m.at<double>(Zi,0) = P[i].z;
			re.push_back(m);
		}
		return re;
	}

	Mat create_unknownParam(Mat& cameraMat){
		Mat re(12, 1, CV_64FC1, Scalar::all(0));
		re.at<double>(C11,0) = cameraMat.at<double>(0,0);
		re.at<double>(C12,0) = cameraMat.at<double>(0,1);
		re.at<double>(C13,0) = cameraMat.at<double>(0,2);
		re.at<double>(C14,0) = cameraMat.at<double>(0,3);
		re.at<double>(C21,0) = cameraMat.at<double>(1,0);
		re.at<double>(C22,0) = cameraMat.at<double>(1,1);
		re.at<double>(C23,0) = cameraMat.at<double>(1,2);
		re.at<double>(C24,0) = cameraMat.at<double>(1,3);
		re.at<double>(C31,0) = cameraMat.at<double>(2,0);
		re.at<double>(C32,0) = cameraMat.at<double>(2,1);
		re.at<double>(C33,0) = cameraMat.at<double>(2,2);
		re.at<double>(C34,0) = cameraMat.at<double>(2,3);
		return re;
	}

	Mat create_cameraMat(Mat& unknownParam){
		Mat cameraMat(3, 4, CV_64FC1, Scalar::all(0));
		cameraMat.at<double>(0,0) = unknownParam.at<double>(C11,0);
		cameraMat.at<double>(0,1) = unknownParam.at<double>(C12,0);
		cameraMat.at<double>(0,2) = unknownParam.at<double>(C13,0);
		cameraMat.at<double>(0,3) = unknownParam.at<double>(C14,0);
		cameraMat.at<double>(1,0) = unknownParam.at<double>(C21,0);
		cameraMat.at<double>(1,1) = unknownParam.at<double>(C22,0);
		cameraMat.at<double>(1,2) = unknownParam.at<double>(C23,0);
		cameraMat.at<double>(1,3) = unknownParam.at<double>(C24,0);
		cameraMat.at<double>(2,0) = unknownParam.at<double>(C31,0);
		cameraMat.at<double>(2,1) = unknownParam.at<double>(C32,0);
		cameraMat.at<double>(2,2) = unknownParam.at<double>(C33,0);
		cameraMat.at<double>(2,3) = unknownParam.at<double>(C34,0);
		return cameraMat;
	}

	void get_perspectiveMat(vector<Point2d>& p, vector<Point3d>& P, Mat& cameraMat, Mat& dst){
//		const double EP = 0.0001;
		const double EP = 0.00001;
		const int MAX_COUNT = 10000;
		const int N=(int)p.size();
		const int M = 2;
		Vector<Mat> m = create_measurementParam(p,P);
		Mat next_c = create_unknownParam(cameraMat);
		Mat c(12, 1, CV_64FC1, Scalar::all(0));
		Function_F1 f1;
		Function_F2 f2;

		int count=0;

		cout << "m size:" << m.size() << endl;
		while(count < MAX_COUNT && abs(norm(c)-norm(next_c)) > EP){
			count++;
			cout << "abs: " << abs(norm(c)-norm(next_c)) << endl;

			next_c.copyTo(c);
		
			//left * delta_x = right
			Mat left(12, 12, CV_64FC1, Scalar::all(0));
			Mat right(12, 1, CV_64FC1, Scalar::all(0));
			Mat delta;
			for(int i=0; i<(int)m.size(); i++){
				//ヤコビ行列
				Mat Jacobian_matrix(2, 12, CV_64FC1, Scalar::all(0));
				for(int s=0; s<12; s++){
					Jacobian_matrix.at<double>(0,s) = f1.differentiation(c,m[i],s);
					Jacobian_matrix.at<double>(1,s) = f2.differentiation(c,m[i],s);
				}
				//左辺の計算
				Mat t_Jacobian_matrix;
				transpose(Jacobian_matrix, t_Jacobian_matrix);
				left = left + t_Jacobian_matrix * Jacobian_matrix;
				//右辺の計算
				Mat F(2, 1, CV_64FC1, Scalar::all(0));
				F.at<double>(0,0) = f1.func(c,m[i]);
				F.at<double>(1,0) = f2.func(c,m[i]);
				right = right + t_Jacobian_matrix * F;
			}
			right = -1*right;
			//連立一次方程式を解く
			solve(left, right, delta);
			next_c = c + delta;
		}
		dst = create_cameraMat(next_c);
	}
};

