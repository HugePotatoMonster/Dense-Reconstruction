#include "../../../include/Common/Utility/cmVisExt.h"
#include "../../../include/Common/Utility/cmMarchingCubesDef.h"
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

		void VisualizationExt::cmuVoxelMarchingCubes(DenseReconstruction::VoxelStore* inStore, Common::Mesh::SimpleMesh* outMesh) {
			//Traverse the voxel store and generate a mesh with vertices and faces
			pr_deprecate();
			using namespace Common::Util::Constant;

			for (i32 i = 0; i < inStore->width - 1; i++) {
				for (i32 j = 0; j < inStore->height - 1; j++) {
					for (i32 k = 0; k < inStore->depth - 1; k++) {
						i32 tx = i + inStore->ox;
						i32 ty = j + inStore->oy;
						i32 tz = k + inStore->oz;
						//Find Vertices Index
						i32 vIdx = 0;
						for (i32 p = 0; p < 8; p++) {
							DenseReconstruction::Voxel* temp;
							if (i + cmuMarchingCubesVertices[p][0] < inStore->width && j + cmuMarchingCubesVertices[p][1] < inStore->height && k + cmuMarchingCubesVertices[p][2] < inStore->depth) {
								inStore->drGetVoxel(tx + cmuMarchingCubesVertices[p][0], ty + cmuMarchingCubesVertices[p][1], tz + cmuMarchingCubesVertices[p][2], &temp);
								if (temp->tsdf > 0.5) {
									vIdx |= (1 << p);
								}
							}
						}
						
						//Generate Mesh Vertices
						i32 edges = cmuMarchingCubesEdgeFlags[vIdx];
						i32 edgeIdxInMesh[12] = { 0 };
						i32 edgeStart, edgeEnd;
						for (i32 p = 0; p < 12; p++) {
							if ((edges & (1 << p))) {
								Common::Mesh::Vertex q;
								edgeStart = cmuMarchingCubesConnection[p][0];
								edgeEnd = cmuMarchingCubesConnection[p][1];
								q.x = -((f64)tx + cmuMarchingCubesVertices[edgeStart][0] + (cmuMarchingCubesVertices[edgeEnd][0] - cmuMarchingCubesVertices[edgeStart][0]) * 0.5);
								q.y = -((f64)ty + cmuMarchingCubesVertices[edgeStart][1] + (cmuMarchingCubesVertices[edgeEnd][1] - cmuMarchingCubesVertices[edgeStart][1]) * 0.5);
								q.z = (f64)tz + cmuMarchingCubesVertices[edgeStart][2] + (cmuMarchingCubesVertices[edgeEnd][2] - cmuMarchingCubesVertices[edgeStart][2]) * 0.5;
								outMesh->v.push_back(q);
								edgeIdxInMesh[p] = static_cast<i32>(outMesh->v.size());
							}
						}
						//Generate Mesh Faces
						for (i32 p = 0; p < 15; p+=3) {
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
			i32 meshFaceLen = static_cast<i32>(mesh->f.size());
			i32 meshVertexLen = static_cast<i32>(mesh->v.size());
			std::cout << "Saving Vertices" << meshVertexLen << std::endl;
			for (i32 i = 0; i < meshVertexLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshVertexLen << std::endl;
				}
				fs << "v " << mesh->v[i].x << " " << mesh->v[i].y << " " << mesh->v[i].z << std::endl;
			}
			std::cout << "Saving Faces" << meshFaceLen << std::endl;
			for (i32 i = 0; i < meshFaceLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshFaceLen << std::endl;
				}
				fs << "f " << mesh->f[i].a << " " << mesh->f[i].b << " " << mesh->f[i].c << std::endl;
			}
			fs.close();
		}
		void VisualizationExt::cmuExportMeshToObj2(std::string fileName, Common::Mesh::Mesh* mesh) {
			std::ofstream fs;
			fs.open(fileName);
			i32 meshFaceLen = static_cast<i32>(mesh->f.size());
			i32 meshVertexLen = static_cast<i32>(mesh->v.size());
			std::cout << "Saving Vertices" << meshVertexLen << std::endl;
			for (i32 i = 0; i < meshVertexLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshVertexLen << std::endl;
				}
				fs << "v " << mesh->v[i].x << " " << mesh->v[i].y << " " << mesh->v[i].z << std::endl;
			}
			std::cout << "Saving Faces" << meshFaceLen << std::endl;
			for (i32 i = 0; i < meshFaceLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshFaceLen << std::endl;
				}
				fs << "f ";
				for(i32 j = 0; j < mesh->f[i].size(); j++) {
					fs << mesh->f[i][j] + 1 << " ";
				}
				fs << std::endl;
			}
			fs.close();
		}
		void VisualizationExt::cmuExportColoredMeshToObj(std::string fileName, Common::Mesh::ColoredSimpleMesh* mesh) {
			std::ofstream fs;
			fs.open(fileName);
			i32 meshFaceLen = static_cast<i32>(mesh->f.size());
			i32 meshVertexLen = static_cast<i32>(mesh->v.size());
			std::cout << "Saving Vertices" << meshVertexLen << std::endl;
			for (i32 i = 0; i < meshVertexLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshVertexLen << std::endl;
				}
				fs << "v " << mesh->v[i].x << " " << mesh->v[i].y << " " << mesh->v[i].z << " " << mesh->c[i].x << " " << mesh->c[i].y << " " << mesh->c[i].z << std::endl;
			}
			std::cout << "Saving Faces" << meshFaceLen << std::endl;
			for (i32 i = 0; i < meshFaceLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshFaceLen << std::endl;
				}
				fs << "f " << mesh->f[i].a << " " << mesh->f[i].b << " " << mesh->f[i].c << std::endl;
			}
			fs.close();
		}
		void VisualizationExt::cmuExportColoredMeshToPly(std::string fileName, Common::Mesh::ColoredSimpleMesh* mesh) {
			std::ofstream fs;
			using namespace std;
			fs.open(fileName);
			fs << "ply" << endl;
			fs << "format ascii 1.0" << endl;
			fs << "comment made by human" << endl;
			fs << "element vertex " << mesh->v.size() << endl;
			fs << "property float x" << endl;
			fs << "property float y" << endl;
			fs << "property float z" << endl;
			fs << "property uchar red" << endl;
			fs << "property uchar green" << endl;
			fs << "property uchar blue" << endl;
			fs << "element face " << mesh->f.size() << endl;
			fs << "property list uchar int vertex_index" << endl;
			fs << "end_header" << endl;
			i32 meshFaceLen = static_cast<i32>(mesh->f.size());
			i32 meshVertexLen = static_cast<i32>(mesh->v.size());
			std::cout << "Saving Vertices" << meshVertexLen << std::endl;
			for (i32 i = 0; i < meshVertexLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshVertexLen << std::endl;
				}
				fs << "" << mesh->v[i].x << " " << mesh->v[i].y << " " << mesh->v[i].z << " " << (i32)(255*mesh->c[i].x) << " " << (i32)(255 * mesh->c[i].y) << " " << (i32)(255 * mesh->c[i].z) << std::endl;
			}
			std::cout << "Saving Faces" << meshFaceLen << std::endl;
			for (i32 i = 0; i < meshFaceLen; i++) {
				if (i % 50000 == 0) {
					std::cout << i << "/" << meshFaceLen << std::endl;
				}
				fs << "3 " << mesh->f[i].a - 1 << " " << mesh->f[i].b - 1 << " " << mesh->f[i].c - 1 << std::endl;
			}
			fs.close();
		}
	}
}