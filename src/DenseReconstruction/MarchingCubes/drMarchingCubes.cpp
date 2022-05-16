#include "../../../include/Common/cmTypeDefs.h"
#include "../../../include/DenseReconstruction/MarchingCubes/drMarchingCubesDef.h"
#include "../../../include/DenseReconstruction/MarchingCubes/drMarchingCubes.h"
namespace DenseReconstruction {
	namespace MarchingCubes {
		void MarchingCubesUtil::mcConvertToMesh(MC_VOXEL_CLASS* volume, Common::Mesh::SimpleMesh* outMesh) {
			using namespace Constant;
			i32 dsX, dsY, dsZ;
			volume->getDims(&dsX, &dsY, &dsZ);
			for (i32 i = 0; i < dsX; i++) {
				for (i32 j = 0; j < dsY; j++) {
					for (i32 k = 0; k < dsZ; k++) {
						i32 vIdx = 0;

						for (i32 p = 0; p < 8; p++) {
							if (i + cmuMarchingCubesVertices[p][0] < dsX && j + cmuMarchingCubesVertices[p][1] < dsY && k + cmuMarchingCubesVertices[p][2] < dsZ) {
								f64 voxel = 0;
								volume->getVoxel(i + cmuMarchingCubesVertices[p][0], j + cmuMarchingCubesVertices[p][1], k + cmuMarchingCubesVertices[p][2], &voxel);
								if (voxel<=0) {
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
								q.x = ((f64)i + cmuMarchingCubesVertices[edgeStart][0] + (cmuMarchingCubesVertices[edgeEnd][0] - cmuMarchingCubesVertices[edgeStart][0]) * 0.5);
								q.y = ((f64)j + cmuMarchingCubesVertices[edgeStart][1] + (cmuMarchingCubesVertices[edgeEnd][1] - cmuMarchingCubesVertices[edgeStart][1]) * 0.5);
								q.z = (f64)k + cmuMarchingCubesVertices[edgeStart][2] + (cmuMarchingCubesVertices[edgeEnd][2] - cmuMarchingCubesVertices[edgeStart][2]) * 0.5;
								outMesh->v.push_back(q);
								edgeIdxInMesh[p] = static_cast<i32>(outMesh->v.size());
							}
						}
						//Generate Mesh Faces
						for (i32 p = 0; p < 15; p += 3) {
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
		void MarchingCubesUtil::mcConvertToColoredMesh(MC_VOXEL_CLASS* volume, Common::Mesh::ColoredSimpleMesh* outMesh) {
			using namespace Constant;
			i32 dsX, dsY, dsZ;
			volume->getDims(&dsX, &dsY, &dsZ);
			for (i32 i = 0; i < dsX; i++) {
				for (i32 j = 0; j < dsY; j++) {
					for (i32 k = 0; k < dsZ; k++) {
						i32 vIdx = 0;
						f64 color;
						volume->getColor(i, j, k, &color);
						for (i32 p = 0; p < 8; p++) {
							if (i + cmuMarchingCubesVertices[p][0] < dsX && j + cmuMarchingCubesVertices[p][1] < dsY && k + cmuMarchingCubesVertices[p][2] < dsZ) {
								f64 voxel = 0;
								volume->getVoxel(i + cmuMarchingCubesVertices[p][0], j + cmuMarchingCubesVertices[p][1], k + cmuMarchingCubesVertices[p][2], &voxel);
								if (voxel <= 0) {
									vIdx |= (1 << p);
								}
							}
						}
						//Generate Mesh Vertices & Colors
						i32 edges = cmuMarchingCubesEdgeFlags[vIdx];
						i32 edgeIdxInMesh[12] = { 0 };
						i32 edgeStart, edgeEnd;
						for (i32 p = 0; p < 12; p++) {
							if ((edges & (1 << p))) {
								Common::Mesh::Vertex q;
								Common::Mesh::Vertex cl;

								int B = floor(color / (256 * 256));
								int G = floor((color - B * 256 * 256) / 256);
								int R = floor(color - B * 256 * 256 - G * 256);
								cl.x = 1.0 * R / 255.0;
								cl.y = 1.0 * G / 255.0;
								cl.z = 1.0 * B / 255.0;
								edgeStart = cmuMarchingCubesConnection[p][0];
								edgeEnd = cmuMarchingCubesConnection[p][1];
								q.x = ((f64)i + cmuMarchingCubesVertices[edgeStart][0] + (cmuMarchingCubesVertices[edgeEnd][0] - cmuMarchingCubesVertices[edgeStart][0]) * 0.5);
								q.y = ((f64)j + cmuMarchingCubesVertices[edgeStart][1] + (cmuMarchingCubesVertices[edgeEnd][1] - cmuMarchingCubesVertices[edgeStart][1]) * 0.5);
								q.z = (f64)k + cmuMarchingCubesVertices[edgeStart][2] + (cmuMarchingCubesVertices[edgeEnd][2] - cmuMarchingCubesVertices[edgeStart][2]) * 0.5;
								outMesh->v.push_back(q);
								outMesh->c.push_back(cl);
								edgeIdxInMesh[p] = static_cast<i32>(outMesh->v.size());
							}
						}
						//Generate Mesh Faces
						for (i32 p = 0; p < 15; p += 3) {
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
	}
}