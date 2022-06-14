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
 * \param argc argc[1]:左影像路径 argc[2]: 右影像路径 argc[3]: 最小视差[可选，默认0] argc[4]: 最大视差[可选，默认64]
 * \param eg. ..\Data\cone\im2.png ..\Data\cone\im6.png 0 64
 * \param eg. ..\Data\Reindeer\view1.png ..\Data\Reindeer\view5.png 0 128
 * \return
 */

/*
* 将每一次迭代的depth信息赋给或者从原来的depth一维数组中获得
*/
void transfer_depth(double *d_i, double *d, int iter, int flag, uint64_t imageWH)
{
    if (flag == 1) // 从原来的depth中获取赋给depth_i
    {
        for (int i = 0; i < imageWH; i++)
        {
            d_i[i] = d[iter * imageWH + i];
        }
    }
    else // 将depth_i还给depth
    {
        for (int i = 0; i < imageWH; i++)
        {
            d[iter * imageWH + i] = d_i[i];
        }
    }
}

double* use_filter(double* depthMap, uint64_t imageWidth, uint64_t imageHeight, const Mat left, const Mat right)
{
    // dch: 初始化深度滤波器参数
    DepthFilter::FilterType filter_type = DepthFilter::Gaussion;
    DepthFilter depth_filter;
    double init_depth = 3.0;
    double init_cov2 = 3.0;
    depth_filter.Initialize(imageWidth, imageHeight, init_depth, init_cov2, filter_type);

    printf("Updating Depth with Depth Dilter...\n");
    // 开始对每一张图进行深度滤波
    int imageNum = (sizeof(depthMap) / sizeof(depthMap[0])) / (imageWidth * imageHeight); // 所有图片总数
    double* depth_i = new double[imageWidth*imageHeight]; // 存放每一张图片原本的深度信息
    double* depth_i_next = new double[imageWidth * imageHeight]; // 存放下一张图片原本的深度信息
    double* depth_a = new double[imageWidth * imageHeight]; // 存放每一张图片滤波后的深度信息
    // 选取后一张对前一张进行滤波 最后一张不动
    for (int i = 0; i < imageNum-1; i++)
    {
        cout << "Now updating..." << i << "........" << endl;
        // 先赋给迭代的深度
        transfer_depth(depth_i, depthMap, i, 1, imageWidth * imageHeight);
        depth_filter.SetDepth(depth_i);
        transfer_depth(depth_i_next, depthMap, i + 1, 1, imageWidth * imageHeight);
        depth_filter.UpdateDepth(depth_i_next,left,right); // 现在只更新一次
        depth_a = depth_filter.GetDepth();
        transfer_depth(depth_a, depthMap, i, 2, imageWidth * imageHeight); // 滤波后更新到原来的depthMap中
    }

    // 完成滤波
    printf("Depth Dilter Finished...\n");
    delete[] depth_i;
    delete[] depth_i_next;
    delete[] depth_a;
    return depthMap;
}