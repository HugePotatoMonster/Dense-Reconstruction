#include "../../../include/Common/Utility/cmVisExt.h"
#include <fstream>
#include <iostream>

namespace Common {
	namespace Util {
		void VisualizationExt::cmuExportVoxelToObj(std::string fileName, DenseReconstruction::VoxelStore* store) {
			//Export the Voxel model to object(*.obj) format
			std::ofstream fs;
			fs.open(fileName);

			u64* pointIndex = allocate_mem(u64, (store->height+1) * (store->depth + 1) * (store->width + 1));
			set_zero(pointIndex, sizeof(u64) * (store->height + 1) * (store->depth + 1) * (store->width + 1));
			u64 currentPointer = 0;
			const i32 dir[8][3] = { {0,0,1},{0,1,0},{1,0,0},{1,0,1},{1,1,0},{0,1,1},{1,1,1},{0,0,0} };
			DenseReconstruction::Voxel* vx;
			std::cout << "Saving Vertices" << std::endl;
			//Saving Vertices
			for (i32 i = 0; i < store->width; i++) {
				std::cout << "Saving Vertices - " <<i << std::endl;
				for (i32 j = 0; j < store->height; j++) {
					for (i32 k = 0; k < store->depth; k++) {
						i32 tx = store->ox + i;
						i32 ty = store->oy + j;
						i32 tz = store->oz + k;
						store->drGetVoxel(tx, ty, tz, &vx);
						if (vx->tsdf > 0.5) {
							for (i32 p = 0; p < 8; p++) {
								if (get_pixel3(pointIndex, i + dir[p][0], j + dir[p][1], k + dir[p][2], store->width + 1, store->height + 1, store->depth + 1) == 0) {
									get_pixel3(pointIndex, i + dir[p][0], j + dir[p][1], k + dir[p][2], store->width + 1, store->height + 1, store->depth + 1) = ++currentPointer;
									fs << "v " << (tx + dir[p][0]) << " " << -(ty + dir[p][1]) << " " << (tz + dir[p][2]) << std::endl;
								}
							}
						}
					}
				}
			}
			std::cout << "Saving Faces" << std::endl;
			//Saving Faces
			//Saving Vertices
			for (i32 i = 0; i < store->width; i++) {
				std::cout << "Saving Faces - " <<i<< std::endl;
				for (i32 j = 0; j < store->height; j++) {
					for (i32 k = 0; k < store->depth; k++) {
						i32 tx = store->ox + i;
						i32 ty = store->oy + j;
						i32 tz = store->oz + k;
						store->drGetVoxel(tx, ty, tz, &vx);
						if (vx->tsdf > 0.5) {
							u64 topLB = get_pixel3(pointIndex, i + 0, j + 0, k + 0, store->width + 1, store->height + 1, store->depth + 1);
							u64 topLT = get_pixel3(pointIndex, i + 0, j + 0, k + 1, store->width + 1, store->height + 1, store->depth + 1);
							u64 topRB = get_pixel3(pointIndex, i + 1, j + 0, k + 0, store->width + 1, store->height + 1, store->depth + 1);
							u64 topRT = get_pixel3(pointIndex, i + 1, j + 0, k + 1, store->width + 1, store->height + 1, store->depth + 1);
							u64 bottomLB = get_pixel3(pointIndex, i + 0, j + 1, k + 0, store->width + 1, store->height + 1, store->depth + 1);
							u64 bottomLT = get_pixel3(pointIndex, i + 0, j + 1, k + 1, store->width + 1, store->height + 1, store->depth + 1);
							u64 bottomRB = get_pixel3(pointIndex, i + 1, j + 1, k + 0, store->width + 1, store->height + 1, store->depth + 1);
							u64 bottomRT = get_pixel3(pointIndex, i + 1, j + 1, k + 1, store->width + 1, store->height + 1, store->depth + 1);
							//Top & Bottom
							fs << "f " << topLB << " " << topRB << " " << topRT << " " << topLT << std::endl;
							fs << "f " << bottomLT << " " << bottomRT << " " << bottomRB << " " << bottomLB << std::endl;
							//Left & Right
							fs << "f " << bottomLT << " " << bottomLB << " " << topLB << " " << topLT << std::endl;
							fs << "f " << topRT << " " << topRB << " " << bottomRB << " " << bottomRT << std::endl;
							//Front & Rear
							fs << "f " << bottomLB << " " << bottomRB << " " << topRB << " " << topLB << std::endl;
							fs << "f " << topLT << " " << topRT << " " << bottomRT << " " << bottomLT << std::endl;
						}
					}
				}
			}
		}
		void VisualizationExt::cmuTsdfBinarization(DenseReconstruction::VoxelStore* inStore, DenseReconstruction::VoxelStore* outStore) {
			//Transform TSDF voxels to binary voxels
			for (i32 x = inStore->ox; x < inStore->ox + inStore->width; x++) {
				for (i32 y = inStore->oy; y < inStore->oy + inStore->height; y++) {
					for (i32 z = inStore->oz; z < inStore->oz + inStore->depth; z++) {
						DenseReconstruction::Voxel* inVoxel;
						DenseReconstruction::Voxel* outVoxel;
						inStore->drGetVoxel(x, y, z, &inVoxel);
						outStore->drGetVoxel(x, y, z, &outVoxel);
						if (inVoxel->tsdf > eps) {
							outVoxel->tsdf = 0;
						}
						else {
							outVoxel->tsdf = 1;
						}
					}
				}
			}
		}
	}
}