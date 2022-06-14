#include "stdafx.h"
#include "SemiGlobalMatching.h"
#include "DepthFilter.h"
#include <chrono>
using namespace std::chrono;
using namespace std;

#include <iostream>
#include <fstream>
#include <string>

// opencv library
#include <opencv2/opencv.hpp>
#ifdef _DEBUG
#pragma comment(lib,"opencv_world310d.lib")
#else
#pragma comment(lib,"opencv_world310.lib")
#endif

/**
 * \brief
 * \param argv 3
 * \param argc argc[1]:��Ӱ��·�� argc[2]: ��Ӱ��·�� argc[3]: ��С�Ӳ�[��ѡ��Ĭ��0] argc[4]: ����Ӳ�[��ѡ��Ĭ��64]
 * \param eg. ..\Data\cone\im2.png ..\Data\cone\im6.png 0 64
 * \param eg. ..\Data\Reindeer\view1.png ..\Data\Reindeer\view5.png 0 128
 * \return
 */

/*
* ��ÿһ�ε�����depth��Ϣ�������ߴ�ԭ����depthһά�����л��
*/
void transfer_depth(double *d_i, double *d, int iter, int flag, uint64_t imageWH)
{
    if (flag == 1) // ��ԭ����depth�л�ȡ����depth_i
    {
        for (int i = 0; i < imageWH; i++)
        {
            d_i[i] = d[iter * imageWH + i];
        }
    }
    else // ��depth_i����depth
    {
        for (int i = 0; i < imageWH; i++)
        {
            d[iter * imageWH + i] = d_i[i];
        }
    }
}

double* use_filter(double* depthMap, uint64_t imageWidth, uint64_t imageHeight, const Mat left, const Mat right)
{
    // dch: ��ʼ������˲�������
    DepthFilter::FilterType filter_type = DepthFilter::Gaussion;
    DepthFilter depth_filter;
    double init_depth = 3.0;
    double init_cov2 = 3.0;
    depth_filter.Initialize(imageWidth, imageHeight, init_depth, init_cov2, filter_type);

    printf("Updating Depth with Depth Dilter...\n");
    // ��ʼ��ÿһ��ͼ��������˲�
    int imageNum = (sizeof(depthMap) / sizeof(depthMap[0])) / (imageWidth * imageHeight); // ����ͼƬ����
    double* depth_i = new double[imageWidth*imageHeight]; // ���ÿһ��ͼƬԭ���������Ϣ
    double* depth_i_next = new double[imageWidth * imageHeight]; // �����һ��ͼƬԭ���������Ϣ
    double* depth_a = new double[imageWidth * imageHeight]; // ���ÿһ��ͼƬ�˲���������Ϣ
    // ѡȡ��һ�Ŷ�ǰһ�Ž����˲� ���һ�Ų���
    for (int i = 0; i < imageNum-1; i++)
    {
        cout << "Now updating..." << i << "........" << endl;
        // �ȸ������������
        transfer_depth(depth_i, depthMap, i, 1, imageWidth * imageHeight);
        depth_filter.SetDepth(depth_i);
        transfer_depth(depth_i_next, depthMap, i + 1, 1, imageWidth * imageHeight);
        depth_filter.UpdateDepth(depth_i_next,left,right); // ����ֻ����һ��
        depth_a = depth_filter.GetDepth();
        transfer_depth(depth_a, depthMap, i, 2, imageWidth * imageHeight); // �˲�����µ�ԭ����depthMap��
    }

    // ����˲�
    printf("Depth Dilter Finished...\n");
    delete[] depth_i;
    delete[] depth_i_next;
    delete[] depth_a;
    return depthMap;
}