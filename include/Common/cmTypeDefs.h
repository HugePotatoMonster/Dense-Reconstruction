#pragma once

#include <cstring>
#include <vector>

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

#define U8_MAX (0xff)
#define U32_MAX (0xffffffff)

#define I8_MAX (0x7f)
#define I32_MAX (0x7fffffff)

#define F64_INF (1e40)

#define SGM_INVALID_DISPARITY_F (-1e-3)
#define SGM_INVALID_DISPARITY_I (0)

#define eps (1e-6)
#define pi (3.1415926)

#define Max(a,b) ((a)>(b)?(a):(b))
#define Min(a,b) ((a)<(b)?(a):(b))
#define Abs(x) ((x)<0?(-(x)):(x))

#define get_pixel(im,x,y,w,h) ((im)[(y)*(w)+(x)])
#define get_pixel_hf(im,x,y,w,h)  get_pixel(im,((w)-1)-(x),y,w,h)
#define get_pixel3(im,x,y,z,w,h,d) ((im)[(y)*(w)*(d)+(x)*(d)+(z)])
#define coord2idx(x,y,w,h) ((y)*(w)+(x))
#define idx2ycoord(idx,w,h) ((i32)(idx)/(w))
#define idx2xcoord(idx,w,h) ((i32)(idx)%(w))
#define coord2idx3(x,y,z,w,h,d) ((y)*(w)*(d)+(x)*(d)+(z))

#define is_queue_empty(fr,ta,len) ((fr)==(ta))
#define is_queue_full(fr,ta,len) (((ta)+1)%(len)==((fr))%(len))
#define queue_push(q,el,fr,ta,len) (q)[((ta)++)%(len)]=(el);(ta)%=(len);
#define queue_front(q,fr,ta,len) ((q)[((fr))%(len)])
#define queue_pop(q,fr,ta,len) ((q)[((fr)++)%(len)]);(fr)%=(len);

#define allocate_mem(tp,size) (new tp[size])
#define free_mem(obj) (delete[] obj)
#define set_zero(x,l) (memset(x,0,l))

namespace Common {
	struct MonocularCameraIntrinsic {
		f64 fx, fy;  //Focal Length
		f64 cx, cy;  //Pixel Translation
		f64 dx, dy;  //Inverse of Pixel Shape
	};
	struct MonocularCameraExtrinsic {
		f64 tx, ty, tz;
		f64 rx, ry, rz;
	};
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
	}
	
}
