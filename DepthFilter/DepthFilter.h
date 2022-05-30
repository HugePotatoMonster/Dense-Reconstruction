#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "sgm_types.h"
#include <vector>

using namespace cv;

/**
 * \brief Depth Filter��
*/
class DepthFilter
{
public:
	DepthFilter();
	~DepthFilter();

	/* ����˲������� */
	enum FilterType {
		Gaussion = 0,
	};

	/* ����˲������� */
	// ��ʱֻ���˲������ͣ���������б�Ҫ����Ϊstruct
	FilterType filter_type_; // ������˲���������
	Mat depth_; // ���ͼ
	Mat depth_cov_; // ���ͼ����

public:
	/**
	 * \brief ��ĳ�ʼ�������һЩ�ڴ��Ԥ���䡢������Ԥ���õ�
	 * \param filter_type	���룬DepthFilter����
	 */
	bool Initialize(const uint32& width, const uint32& height, double &mu_first, double &sigma_first, const FilterType &filter_type);
	/**
	 * \brief ������ȷֲ�
	 * \param mu	���룬����SGM,�õ���ĳһλ�����ص����mu
	 * \param sigma	���룬����SGM,�õ���ĳһλ�����ص����sigma
	 * \param mu_fuse	�������������˲��������º�����mu_fuse
	 * \param sigma_fuse	�������������˲��������º�����sigma_fuse
	 */
	bool UpdateDepth(const double* depthMap, double &mu_fuse, double &sigma_fuse);

	/**
	* ����Mat depth_����Ϊ���յ���Ⱦ���
	*/
	Mat GetDepth();

private:
	/** \brief �Ƿ��ʼ����־	*/
	bool is_initialized_;
	/* ͼ��Ŀ� */
	sint32 img_width_;
	/* ͼ��ĳ� */
	sint32 img_height_;
	/* ��ȳ�ʼֵ */
	double init_depth = 3.0;
	/* �����ʼֵ */
	double init_cov2 = 3.0;
	/* �������һ�θ��µ�mu */
	double mu_latest_;
	/* �������һ�θ��µ�sigma */
	double sigma_latest_;

	/* ���������ģ */
	template<typename T>
	double arr_norm(T* arr, int arr_len);
	/* ��������ĵ�� */
	template<typename T>
	double arr_dot(T *arr1, T *arr2,int arr_len);
	/* ����ƽ��ӳ�䵽���ƽ�� */
	void px2cam(const sint32 *px, sint32 *res); // ���ܲ��ܱ�֤res�����е�����������sint32����
};