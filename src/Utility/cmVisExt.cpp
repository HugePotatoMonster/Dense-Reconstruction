#include "../../include/Utility/cmVisExt.h"
#include "../../include/Utility/cmMarchingCubesDef.h"
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
			const int dir[8][3] = { {0,0,1},{0,1,0},{1,0,0},{1,0,1},{1,1,0},{0,1,1},{1,1,1},{0,0,0} };
			DenseReconstruction::Voxel* vx;
			std::cout << "Saving Vertices" << std::endl;
			//Saving Vertices
			for (int i = 0; i < store->width; i++) {
				std::cout << "Saving Vertices - " <<i << std::endl;
				for (int j = 0; j < store->height; j++) {
					for (int k = 0; k < store->depth; k++) {
						int tx = store->ox + i;
						int ty = store->oy + j;
						int tz = store->oz + k;
						store->drGetVoxel(tx, ty, tz, &vx);
						if (vx->tsdf > 0.5) {
							for (int p = 0; p < 8; p++) {
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
			for (int i = 0; i < store->width; i++) {
				std::cout << "Saving Faces - " <<i<< std::endl;
				for (int j = 0; j < store->height; j++) {
					for (int k = 0; k < store->depth; k++) {
						int tx = store->ox + i;
						int ty = store->oy + j;
						int tz = store->oz + k;
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
			for (int x = inStore->ox; x < inStore->ox + inStore->width; x++) {
				for (int y = inStore->oy; y < inStore->oy + inStore->height; y++) {
					for (int z = inStore->oz; z < inStore->oz + inStore->depth; z++) {
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

		void VisualizationExt::cmuVoxelMarchingCubes(DenseReconstruction::VoxelStore* inStore, Common::Mesh::SimpleMesh* outMesh) {
			//Traverse the voxel store and generate a mesh with vertices and faces
			using namespace Common::Util::Constant;

			std::cout << "inStore Size:" << inStore->width << "*" << inStore->height << "*"  << inStore->depth << std::endl;

			for (int i = 0; i < inStore->width - 1; i++) {
				for (int j = 0; j < inStore->height - 1; j++) {
					for (int k = 0; k < inStore->depth - 1; k++) {
						// int tx = i + inStore->ox;
						// int ty = j + inStore->oy;
						// int tz = k + inStore->oz;
						int tx = i;
						int ty = j;
						int tz = k;
						//Find Vertices Index
						int vIdx = 0;
						// std::cout << "voxel->tsdf[" << tx << "][ " << ty << "][" <<tz << "]:" << std::endl;
						for (int p = 0; p < 8; p++) {
							DenseReconstruction::Voxel* temp;
							if (i + cmuMarchingCubesVertices[p][0] < inStore->width && j + cmuMarchingCubesVertices[p][1] < inStore->height && k + cmuMarchingCubesVertices[p][2] < inStore->depth) {
								inStore->drGetVoxel(tx + cmuMarchingCubesVertices[p][0], ty + cmuMarchingCubesVertices[p][1], tz + cmuMarchingCubesVertices[p][2], &temp);
								// std::cout << "voxel->tsdf[" << tx + cmuMarchingCubesVertices[p][0] << "][ " << ty + cmuMarchingCubesVertices[p][1] << "][" << tz + cmuMarchingCubesVertices[p][2] << "]: ";
								// std::cout << temp->tsdf << std::endl;
								// if (temp->tsdf > 0) {
                        			// std::cout << "voxel->tsdf[" << tx + cmuMarchingCubesVertices[p][0] << "][ " << cmuMarchingCubesVertices[p][1] << "][" << cmuMarchingCubesVertices[p][2] << "]: " << temp->tsdf << std::endl;
								// }
								if (temp->tsdf > 0.5) {
									vIdx |= (1 << p);
									// std::cout << "[" << i << "][ " << j << "][" << k << "]: " << std::endl;
									// std::cout << "vIdx: " << vIdx << std::endl;
								}
							}
						}
						
						//Generate Mesh Vertices
						int edges = cmuMarchingCubesEdgeFlags[vIdx];
						int edgeIdxInMesh[12] = { 0 };
						int edgeStart, edgeEnd;
						for (int p = 0; p < 12; p++) {
							if ((edges & (1 << p))) {
								Common::Mesh::Vertex q;
								edgeStart = cmuMarchingCubesConnection[p][0];
								edgeEnd = cmuMarchingCubesConnection[p][1];
								q.x = -((double)tx + cmuMarchingCubesVertices[edgeStart][0] + (cmuMarchingCubesVertices[edgeEnd][0] - cmuMarchingCubesVertices[edgeStart][0]) * 0.5);
								q.y = -((double)ty + cmuMarchingCubesVertices[edgeStart][1] + (cmuMarchingCubesVertices[edgeEnd][1] - cmuMarchingCubesVertices[edgeStart][1]) * 0.5);
								q.z = (double)tz + cmuMarchingCubesVertices[edgeStart][2] + (cmuMarchingCubesVertices[edgeEnd][2] - cmuMarchingCubesVertices[edgeStart][2]) * 0.5;
								outMesh->v.push_back(q);
								edgeIdxInMesh[p] = outMesh->v.size();
							}
						}
						//Generate Mesh Faces
						for (int p = 0; p < 15; p+=3) {
							if (cmuMarchingCubesTriangleTable[vIdx][p] == -1) {
								continue;
							}
							Common::Mesh::IndexedTriangularFace face;
							face.a = edgeIdxInMesh[cmuMarchingCubesTriangleTable[vIdx][p]];
							face.b = edgeIdxInMesh[cmuMarchingCubesTriangleTable[vIdx][p + 1]];
							face.c = edgeIdxInMesh[cmuMarchingCubesTriangleTable[vIdx][p + 2]];
							outMesh->f.push_back(face);
						}
					}
				}
			}
		}
		void VisualizationExt::cmuExportMeshToObj(std::string fileName, Common::Mesh::SimpleMesh* mesh) {
			std::ofstream fs;
			fs.open(fileName);
			int meshFaceLen = mesh->f.size();
			int meshVertexLen = mesh->v.size();
			std::cout << "Saving Vertices" << meshVertexLen << std::endl;
			for (int i = 0; i < meshVertexLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshVertexLen << std::endl;
				}
				fs << "v " << mesh->v[i].x << " " << mesh->v[i].y << " " << mesh->v[i].z << std::endl;
			}
			std::cout << "Saving Faces" << meshFaceLen << std::endl;
			for (int i = 0; i < meshFaceLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshFaceLen << std::endl;
				}
				fs << "f " << mesh->f[i].a << " " << mesh->f[i].b << " " << mesh->f[i].c << std::endl;
			}
			fs.close();
		}
	}
}