#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace cv;

#pragma once

#include <cstdint>
#include <limits>

constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

typedef int8_t			sint8;		
typedef uint8_t			uint8;		
typedef int16_t			sint16;		
typedef uint16_t		uint16;		
typedef int32_t			sint32;		
typedef uint32_t		uint32;		
typedef int64_t			sint64;		
typedef uint64_t		uint64;		
typedef float			float32;	
typedef double			float64;	

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
		Even_Gaussion = 1,
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
	bool Initialize(const uint32& width, const uint32& height, double& mu_first, double& sigma_first, const FilterType& filter_type);
	/**
	 * \brief ������ȷֲ�
	 * \param mu	���룬����SGM,�õ���ĳһλ�����ص����mu
	 * \param sigma	���룬����SGM,�õ���ĳһλ�����ص����sigma
	 * \param mu_fuse	�������������˲��������º�����mu_fuse
	 * \param sigma_fuse	�������������˲��������º�����sigma_fuse
	 */
	bool UpdateDepth(const double* depthMap, const Mat left, const Mat right);

	/**
	* ����Mat depth_��ʹ��SGM��֪��Ⱦ��󸳳�ֵ
	*/
	Mat SetDepth(double* d_i);

	/**
	* ����Mat depth_����Ϊ���յ���Ⱦ���
	*/
	double* GetDepth();

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
	// double mu_latest_;
	/* �������һ�θ��µ�sigma */
	// double sigma_latest_;

	/* ���������ģ */
	template<typename T>
	double arr_norm(T* arr, int arr_len);
	/* ��������ĵ�� */
	template<typename T>
	double arr_dot(T* arr1, T* arr2, int arr_len);
	/* ����ƽ��ӳ�䵽���ƽ�� */
	void px2cam(const sint32* px, sint32* res); // ���ܲ��ܱ�֤res�����е�����������sint32����

	/* �ڴ��ͷ� */
	void Release();
	double arr_norm(double* arr, int arr_len);
	double arr_dot(double* arr1, double* arr2, int arr_len);
};
