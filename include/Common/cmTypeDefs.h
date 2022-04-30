#pragma once

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

#define eps (1e-9)
#define pi (3.1415926)

#define Max(a,b) ((a)>(b)?(a):(b))
#define Min(a,b) ((a)<(b)?(a):(b))

#define get_pixel(im,x,y,w,h) ((im)[(y)*(w)+(x)])
#define get_pixel3(im,x,y,z,w,h,d) ((im)[(y)*(w)*(d)+(x)*(d)+(z)])
#define allocate_mem(tp,size) (new tp[size])
#define free_mem(obj) (delete[] obj)