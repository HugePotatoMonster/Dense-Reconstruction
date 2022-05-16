#pragma once

#include <cstddef>
#include <cstring>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <sstream>
#include <cmath>
#include <cstdio>
#include <cassert>
#include <opencv2/opencv.hpp>


//Basic
#define i8 char
#define i16 short
#define i32 int
#define i64 long long
#define u8 unsigned char
#define u16 unsigned short
#define u32 unsigned int
#define u64 unsigned long long
#define f32 float
#define f64 double

#define usize size_t

#define U8_MAX (0xff)
#define U32_MAX (0xffffffff)

#define I8_MAX (0x7f)
#define I32_MAX (0x7fffffff)

#define F64_INF (1e40)

//Memory
#define allocate_mem(tp,size) (new tp[size])
#define free_mem(obj) (delete[] obj)
#define set_zero(x,l) (memset(x,0,l))

//CUDA
#define PR_CUDA_ENABLE 

#ifdef PR_CUDA_ENABLE

#include <cuda_runtime.h>
#define cu_enabled (true)
#define cu_global __global__
#define cu_device __device__
#define cu_host __host__
#define cu_call(function,blocks,threads) function << <blocks, threads>> >
#define cu_sync() cudaDeviceSynchronize()
#define cu_allocate_mem(var, tp,size) ((var=nullptr;),cudaMalloc((void**)(&(var)),sizeof(tp)*(usize)(size)))
#define cu_free_mem(var) (cudaFree(var))

#else

#define cu_enabled (false)
#define cu_global
#define cu_device
#define cu_host
#define cu_call(function,blocks,threads) function
#define cu_sync() 
#define cu_allocate_mem(var, tp,size) ((var) = allocate_mem(tp,size))
#define cu_free_mem(var) (free_mem(var))

#endif


//SGM
#define SGM_INVALID_DISPARITY_F (-1e4)
#define SGM_INVALID_DISPARITY_F_THRESH (-1e2)
#define SGM_INVALID_DISPARITY_I (0)

//SGM Parallel
#define SGMP_PARALLEL_THREADS 128

//DepthEstimation
#define DEPEST_INVALID_PIXEL (-1e30)

//Math Constants
#define eps (1e-6)
#define pi (3.1415926)

//Fundamental
#define Max(a,b) ((a)>(b)?(a):(b))
#define Min(a,b) ((a)<(b)?(a):(b))
#define Abs(x) ((x)<0?(-(x)):(x))
#define Fabs(x) ((x)<eps?(-(x)):(x))

//Coordinates
#define get_pixel(im,x,y,w,h) ((im)[(y)*(w)+(x)])
#define get_pixel_hf(im,x,y,w,h)  get_pixel(im,((w)-1)-(x),y,w,h)
#define get_pixel3(im,x,y,z,w,h,d) ((im)[(y)*(w)*(d)+(x)*(d)+(z)])
#define coord2idx(x,y,w,h) ((y)*(w)+(x))
#define idx2ycoord(idx,w,h) ((i32)(idx)/(w))
#define idx2xcoord(idx,w,h) ((i32)(idx)%(w))
#define coord2idx3(x,y,z,w,h,d) ((y)*(w)*(d)+(x)*(d)+(z))

//Data Structure
#define is_queue_empty(fr,ta,len) ((fr)==(ta))
#define is_queue_full(fr,ta,len) (((ta)+1)%(len)==((fr))%(len))
#define queue_push(q,el,fr,ta,len) (q)[((ta)++)%(len)]=(el);(ta)%=(len);
#define queue_front(q,fr,ta,len) ((q)[((fr))%(len)])
#define queue_pop(q,fr,ta,len) ((q)[((fr)++)%(len)]);(fr)%=(len);



//Debug
#define DEBUG_MODE true
#define dbg_trace(x) if(DEBUG_MODE){x}
#define dbg_output std::cout
#define dbg_toutput(x) dbg_trace(std::cout<<(x)<<std::endl;)

//Aux Tags
#define OUT_ARG //output arguments
#define IN_ARG  //input arguments
#define CU_ARG

//Linear Algebra
#define det2(a11,a12,a21,a22) ((a11)*(a22)-(a21)*(a12))
#define len_vec3(x,y,z) (sqrt((x)*(x)+(y)*(y)+(z)*(z)))
#define norm_vec3(x,len) ((x)=(x)/(len))

//OpenCV
#define OCV_IDEAL_DISTORTION ((cv::Mat)(cv::Mat_<f64>(4 , 1) << 0,0, 0, 0));
#define OCV_IDENTITY_3 ((cv::Mat)(cv::Mat_<f64>(3 , 3) << 1,0,0,  0,1,0, 0,0,1));
#define get_cvmat(mat,i,j) ((mat).at<f64>((i),(j)))
#define get_cvmatu8(mat,i,j) ((mat).at<u8>((i),(j)))
#define get_cvmatp(mat,i,j) ((mat)->at<f64>((i),(j)))
#define dbg_printcvmap(mat,i,j) dbg_trace(for(i32 _i=0;_i<i;_i++){for(i32 _j=0;_j<j;_j++)dbg_output<<get_cvmat(mat,_i,_j)<<",";dbg_output<<std::endl;}dbg_output<<std::endl;)

//Assertions
#define pr_assert(x) assert(x);if(!(x))abort();
#define pr_assert_notnull(x) pr_assert((x)!=nullptr)
#define pr_deprecate() std::cout<<"Function deprecated"<<std::endl;assert(true);


namespace Common {
	namespace Camera {
		struct MonocularCameraIntrinsic {
			f64 fx, fy;  //Focal Length
			f64 cx, cy;  //Pixel Translation
			f64 dx, dy;  //Inverse of Pixel Shape
		};
		struct MonocularCameraExtrinsic {
			f64 t[3]; //Translation
			f64 r[3][3]; //Rotation
		};
	}

	namespace Mesh {
		struct Vertex {
			f64 x, y, z;
		};
		struct IndexedTriangularFace {
			i32 a, b, c;
		};
		struct SimpleMesh {
			std::vector<Vertex> v;
			std::vector<IndexedTriangularFace> f;
		};
		struct ColoredSimpleMesh {
			std::vector<Vertex> v;
			std::vector<Vertex> c;
			std::vector<IndexedTriangularFace> f;
		};
	}

	namespace Math {
		struct Mat3 {
			f64 a[9];
		};
		struct Vec3 {
			f64 a[3];
			void normalize() {
				f64 len = len_vec3(a[0], a[1], a[2]);
				a[0] /= len;
				a[1] /= len;
				a[2] /= len;
			}
			void dist(OUT_ARG f64* out) {
				*out = len_vec3(a[0], a[1], a[2]);
			}
		};
	}
	
}
