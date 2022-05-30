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
	is_initialized_ = false;
}

bool DepthFilter::Initialize(const uint32 &width, const uint32& height, double& mu_first, double& sigma_first, const FilterType& filter_type)
{
	// ··· 赋值

	// 图片的宽和高
	img_width_ = width;
	img_height_ = height;

	// 初始深度、方差
	init_depth = mu_first;
	init_cov2 = sigma_first;

	// 初始化深度图相关矩阵
	Mat depth_initial(img_width_, img_height_, CV_64F, init_depth);
	depth_ = depth_initial; // 这里initial后面也会变化
	Mat depth_cov_initial(img_width_, img_height_, CV_64F, init_cov2);
	depth_cov_ = depth_cov_initial; // 这里initial后面也会变化
	

	// 最初的深度信息
	mu_latest_ = mu_first;
	sigma_latest_ = sigma_first;

	if (mu_latest_ < 0 || sigma_latest_ < 0)
	{
		is_initialized_ = false;
		return is_initialized_;
	}

	// 深度滤波器类型
	if (filter_type == NULL) {
		is_initialized_ = false;
		return false;
	}
	filter_type_ = filter_type;

	// 初始化完成
	is_initialized_ = true;

	return is_initialized_;
}

bool DepthFilter::Initialize(const uint32& width, const uint32& height, double& mu_first, double& sigma_first, const FilterType& filter_type)
{
	return false;
}

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
	res[0] = (px[0] - cx) / fx;
	res[1] = (px[1] - cy) / fy;
	res[2] = 1;
}

bool DepthFilter::UpdateDepth(const double *depthMap, double& mu_fuse, double& sigma_fuse)
{
	// 需要输入匹配点位置的坐标(a,w,h) a表示第几张图,w和h分别表示宽和高
	// mat.ptr<type>(row)[col]
	if (filter_type_ == Gaussion)
	{
		// 遍历所有图像上的点
		for (int i = 0; i < img_width_; i++)
		{
			for (int j = 0; j < img_height_; j++)
			{
				// 获取当前像素位置和预估深度
				sint32* px_ref = new int[2];
				px_ref[0] = i; px_ref[1] = j;
				double depth_estimation = depthMap[i * img_height_ + j];

				// 计算不确定性（以一个像素为误差）
				
				// 原像素点在相机平面
				//Vector3d f_ref = 0.0;
				sint32* f_ref = new int[3];
				for (int temp = 0; temp < 3; temp++)
				{
					f_ref[temp] = 0;
				}
				px2cam(px_ref, f_ref);
				
				// 相机平面加上深度估计
				//Vector3d p = f_ref * depth_estimation;
				double* p = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					p[temp] = f_ref[temp] * depth_estimation;
				}

				// 计算偏差
				// Vector3d a = p - t; 
				// t是相机位姿的平移变化
				double* t = new double[3];
				for (int temp = 0; temp < 3; temp++)
				{
					t[temp] = 0.0;
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
				double beta_prime = beta + atan(1 / fx); // fx是内参
				double gamma = M_PI - alpha - beta_prime;
				double p_prime = t_norm * sin(beta_prime) / sin(gamma);
				double d_cov = p_prime - depth_estimation;
				double d_cov2 = d_cov * d_cov;

				// 高斯融合
				/*double mu = mu_latest_;
				double sigma2 = sigma_latest_;*/
				double mu = depth_.ptr<double>(i)[j];
				double sigma2 = depth_cov_.ptr<double>(i)[j];

				double mu_fuse = (d_cov2 * mu + sigma2 * depth_estimation) / (sigma2 + d_cov2);
				double sigma_fuse = (sigma2 * d_cov2) / (sigma2 + d_cov2);

				depth_.ptr<double>(i)[j] = mu_fuse;
				depth_cov_.ptr<double>(i)[j] = sigma_fuse;
			}
		}
	}
	return true;
}

/*
* 返回深度数组
*/
Mat DepthFilter::GetDepth()
{
	return depth_;
}