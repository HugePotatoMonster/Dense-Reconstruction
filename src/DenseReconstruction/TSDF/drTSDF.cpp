#include "../../../include/DenseReconstruction/TSDF/drTSDF.h"

namespace DenseReconstruction {
	void TruncatedSDF::drIdealFirstTsdfEstimate(f64* depthMap, u32 imageWidth, u32 imageHeight, Common::MonocularCameraIntrinsic* cameraInt, VoxelStore* outStore, f64 truncationValue) {
		//This assumes "World Coordinate System" equals to "Camera Coordinate System". That is, the Camera Extrinsic is the identity matrix
		//This assumes the camera has no distortion
		for (i32 x = outStore->ox ; x < outStore->width + outStore->ox; x++) {
			for (i32 y = outStore->oy; y < outStore->height + outStore->oy; y++) {
				for (i32 z = outStore->oz; z < outStore->depth+outStore->oz; z++) {
					//The point represented on the Camera Plane is (x,y,z)
					//And the point represented on the Retina Camera Plane is (x/z,y/z,1)
					//Then transform (x/z,y/z,1) into (u,v,1)
					// (cU/dx)   ( fx  0  cx ) (x/z) 
					// (cV/dy) = ( 0   fy cy ) (y/z)
					// (c    )   ( 0   0  1  ) (1  )
					i32 u = (((f64)x / z) * cameraInt->fx + cameraInt->cx) * cameraInt->dx + 0.5;
					i32 v = (((f64)y / z) * cameraInt->fy + cameraInt->cy) * cameraInt->dy + 0.5;
					if (u<0 || u>imageWidth || v<0 || v>imageHeight) {
						continue;
					}
					f64 depth = get_pixel(depthMap, u, v, imageWidth, imageHeight);
					if (depth < 0.0f) {
						continue;
					}
					//Calcualte TSDF
					f64 sdf = depth - (f64)z;
					f64 tsdf = (sdf < -truncationValue) ? (-truncationValue) : ((sdf > truncationValue) ? (truncationValue) : sdf);
					f64 retinaTsdf = tsdf / truncationValue;
					
					//Set TSDF
					Voxel* temp;
					outStore->drGetVoxel(x, y, z, &temp);
					temp->tsdf = retinaTsdf;
				}
			}
		}
	}
}