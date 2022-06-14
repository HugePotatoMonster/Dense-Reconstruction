#define _USE_MATH_DEFINES

#include "stdafx.h"
#include "DepthFilter.h"
#include <algorithm>
#include <cassert>
#include <vector>
#include <queue>
#include <cmath>

DepthFilter::DepthFilter():is_initialized_(false)
{

}

DepthFilter::~DepthFilter()
{
	// Release();
	is_initialized_ = false;
}

void DepthFilter::Release()
{

}

bool DepthFilter::Initialize(const uint32 &width, const uint32& height, double& mu_first, double& sigma_first, const FilterType& filter_type)
{
	// ������ ��ֵ

	// ͼƬ�Ŀ��͸�
	img_width_ = width;
	img_height_ = height;

	// ��ʼ��ȡ�����
	init_depth = mu_first;
	init_cov2 = sigma_first;

	// ��ʼ�����ͼ��ؾ���
	Mat depth_initial(img_width_, img_height_, CV_64F, init_depth);
	depth_ = depth_initial; // ����initial����Ҳ��仯
	Mat depth_cov_initial(img_width_, img_height_, CV_64F, init_cov2);
	depth_cov_ = depth_cov_initial; // ����initial����Ҳ��仯
	

	// ����������Ϣ
	// mu_latest_ = mu_first;
	// sigma_latest_ = sigma_first;

	/*
	if (mu_latest_ < 0 || sigma_latest_ < 0)
	{
		is_initialized_ = false;
		return is_initialized_;
	}
	*/

	// ����˲�������
	if (filter_type == NULL) {
		is_initialized_ = false;
		return false;
	}
	filter_type_ = filter_type;

	// ��ʼ�����
	is_initialized_ = true;

	return is_initialized_;
}

//bool DepthFilter::Initialize(const uint32& width, const uint32& height, double& mu_first, double& sigma_first, const FilterType& filter_type)
//{
//	return false;
//}

template<typename T>
double arr_norm(T* arr, int arr_len)
{
	double sum = 0.0;
	for (int i = 0; i < arr_len; i++)
	{
		sum += arr[i] * arr[i];
	}
	return sqrt(sum);
}

template<typename T>
double arr_dot(T* arr1, T* arr2, int arr_len)
{
	double sum = 0.0;
	for (int i = 0; i < arr_len; i++)
	{
		sum += arr1[i] * arr2[i];
	}
	return sum;
}

void DepthFilter::px2cam(const sint32* px, sint32* res)
{
	// Intrinsic parameters
	double fx = 481.20;
	double fy = 480.00;
	double cx = 319.50;
	double cy = 239.50;
	res[0] = (px[0] - cx) / fx;
	res[1] = (px[1] - cy) / fy;
	res[2] = 1;
}

bool DepthFilter::UpdateDepth(const double *depthMap, const Mat left, const Mat right)
{
	// �����ͼ����ͼ�����
	Mat leftR(3, 3, CV_64F, 0.0);
	Mat leftT(3, 1, CV_64F, 0.0);
	Mat rightR(3, 3, CV_64F, 0.0);
	Mat rightT(3, 1, CV_64F, 0.0);

	// �ȴ�����תR
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			leftR.at<double>(i, j) = left.at<double>(i, j);
			rightR.at<double>(i, j) = right.at<double>(i, j);
		}
	}

	// �ٴ���ƽ��t
	for (int i=0;i<3;i++)
	{
		leftT.at<double>(i, 0) = left.at<double>(i, 3);
		rightT.at<double>(i, 0) = right.at<double>(i, 3);
	}


	// parameters
	double fx = 481.20;
	// ��Ҫ����ƥ���λ�õ�����(a,w,h) a��ʾ�ڼ���ͼ,w��h�ֱ��ʾ���͸�
	// mat.ptr<type>(row)[col]
	if (filter_type_ == Gaussion)
	{
		// ��������ͼ���ϵĵ�
		for (int i = 0; i < img_width_; i++)
		{
			for (int j = 0; j < img_height_; j++)
			{
				// ��ȡ��ǰ����λ�ú�Ԥ�����
				sint32* px_ref = new int[2];
				px_ref[0] = i; px_ref[1] = j;
				double depth_estimation = depthMap[i * img_height_ + j];

				// ���㲻ȷ���ԣ���һ������Ϊ��
				
				// ԭ���ص������ƽ��
				//Vector3d f_ref = 0.0;
				sint32* f_ref = new int[3];
				for (int temp = 0; temp < 3; temp++)
				{
					f_ref[temp] = 0;
				}
				px2cam(px_ref, f_ref);
				
				// ���ƽ�������ȹ���
				//Vector3d p = f_ref * depth_estimation;
				double* p = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					p[temp] = f_ref[temp] * depth_estimation;
				}

				// ����ƫ��
				// Vector3d a = p - t; 
				// t�����λ�˵�ƽ�Ʊ仯
				// ���ﻹҪ��
				// ��hzw�Ĺ�ʽ
				Mat relT;
				relT = leftR * (rightT - leftT);

				double* t = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					t[temp] = relT.at<double>(temp, 0);
				}

				double* a = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					a[temp] = p[temp]-t[temp];
				}

				// double t_norm = t.norm();
				// double a_norm = a.norm();
				double t_norm = arr_norm(t, 3);
				double a_norm = arr_norm(a, 3);
				double alpha = acos((arr_dot((double *)f_ref,t,3)) / t_norm);
				double beta = acos(-(arr_dot(a, t, 3)) / (a_norm * t_norm));
				double beta_prime = beta + atan(1 / fx); // fx���ڲ�
				double gamma = M_PI - alpha - beta_prime;
				double p_prime = t_norm * sin(beta_prime) / sin(gamma);
				double d_cov = p_prime - depth_estimation;
				double d_cov2 = d_cov * d_cov;

				// ��˹�ں�
				/*double mu = mu_latest_;
				double sigma2 = sigma_latest_;*/
				double mu = depth_.ptr<double>(i)[j];
				double sigma2 = depth_cov_.ptr<double>(i)[j];

				double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
				double sigma_fuse = (sigma2 * d_cov2) / (sigma2 + d_cov2);

				depth_.ptr<double>(i)[j] = mu_fuse;
				depth_cov_.ptr<double>(i)[j] = sigma_fuse;

				// �ͷ��ڴ�
				delete[]px_ref;
				delete[]f_ref;
				delete[]p;
				delete[]t;
				delete[]a;
			}
		}
	}
	else if (filter_type_ == Even_Gaussion)
	{
		// ��������ͼ���ϵĵ�
		for (int i = 0; i < img_width_; i++)
		{
			for (int j = 0; j < img_height_; j++)
			{
				// ��ȡ��ǰ����λ�ú�Ԥ�����
				sint32* px_ref = new int[2];
				px_ref[0] = i; px_ref[1] = j;
				double depth_estimation = depthMap[i * img_height_ + j];

				// ���㲻ȷ���ԣ���һ������Ϊ��

				// ԭ���ص������ƽ��
				//Vector3d f_ref = 0.0;
				sint32* f_ref = new int[3];
				for (int temp = 0; temp < 3; temp++)
				{
					f_ref[temp] = 0;
				}
				px2cam(px_ref, f_ref);

				// ���ƽ�������ȹ���
				//Vector3d p = f_ref * depth_estimation;
				double* p = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					p[temp] = f_ref[temp] * depth_estimation;
				}

				// ����ƫ��
				// Vector3d a = p - t; 
				// t�����λ�˵�ƽ�Ʊ仯
				double* t = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					t[temp] = 0.0;
				}

				double* a = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					a[temp] = p[temp] - t[temp];
				}

				// double t_norm = t.norm();
				// double a_norm = a.norm();
				double t_norm = arr_norm(t, 3);
				double a_norm = arr_norm(a, 3);
				double alpha = acos((arr_dot((double*)f_ref, t, 3)) / t_norm);
				double beta = acos(-(arr_dot(a, t, 3)) / (a_norm * t_norm));
				double beta_prime = beta + atan(1 / fx); // fx���ڲ�
				double gamma = M_PI - alpha - beta_prime;
				double p_prime = t_norm * sin(beta_prime) / sin(gamma);
				double d_cov = p_prime - depth_estimation;
				double d_cov2 = d_cov * d_cov;

				// ����Ӧ�ò���Ҫ��

				// ��˹�ں�
				/*double mu = mu_latest_;
				double sigma2 = sigma_latest_;*/
				double mu = depth_.ptr<double>(i)[j];
				double sigma2 = depth_cov_.ptr<double>(i)[j];

				double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
				double sigma_fuse = (sigma2 * d_cov2) / (sigma2 + d_cov2);

				depth_.ptr<double>(i)[j] = mu_fuse;
				depth_cov_.ptr<double>(i)[j] = sigma_fuse;

				// �ͷ��ڴ�
				delete[]px_ref;
				delete[]f_ref;
				delete[]p;
				delete[]t;
				delete[]a;
			}
		}
	}
	return true;
}

/*
* ������鸳ֵ
*/
Mat DepthFilter::SetDepth(double* d_i)
{
	depth_ = Mat(img_width_, img_height_, CV_64F, d_i); //Ӧ���ǿ��Ը�ֵ��
}

/*
* �����������
*/
double* DepthFilter::GetDepth()
{
	double* d_a = new double[img_width_ * img_height_];
	for (int i = 0; i < img_width_ * img_height_; i++)
	{
		d_a[i] = depth_.at<double>(i / img_width_, i-(i/img_width_)*img_width_);
	}
	return d_a;
}