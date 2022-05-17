#include "../../../include/Common/cmTypeDefs.h"
#include "../../../include/DenseReconstruction/MarchingCubes/drMarchingCubesDef.h"
#include "../../../include/DenseReconstruction/MarchingCubes/drMarchingCubes.h"
namespace DenseReconstruction {
	namespace MarchingCubes {
		void MarchingCubesUtil::mcConvertToMesh(MC_VOXEL_CLASS* volume, Common::Mesh::SimpleMesh* outMesh) {
			using namespace Constant;
			i32 dsX, dsY, dsZ;
			volume->getDims(&dsX, &dsY, &dsZ);
			std::map<Common::Mesh::Vertex, i32> vlist;
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
						i32 edgeStart, edgeEnd = 0;
						for (i32 p = 0; p < 12; p++) {
							if ((edges & (1 << p))) {
								Common::Mesh::Vertex q;
								edgeStart = cmuMarchingCubesConnection[p][0];
								edgeEnd = cmuMarchingCubesConnection[p][1];
								q.x = ((f64)i + cmuMarchingCubesVertices[edgeStart][0] + (cmuMarchingCubesVertices[edgeEnd][0] - cmuMarchingCubesVertices[edgeStart][0]) * 0.5);
								q.y = ((f64)j + cmuMarchingCubesVertices[edgeStart][1] + (cmuMarchingCubesVertices[edgeEnd][1] - cmuMarchingCubesVertices[edgeStart][1]) * 0.5);
								q.z = (f64)k + cmuMarchingCubesVertices[edgeStart][2] + (cmuMarchingCubesVertices[edgeEnd][2] - cmuMarchingCubesVertices[edgeStart][2]) * 0.5;
								if (vlist[q] == 0) {
									outMesh->v.push_back(q);
									edgeIdxInMesh[p] = static_cast<i32>(outMesh->v.size());
									vlist[q] = static_cast<i32>(outMesh->v.size());
								}
								else {
									edgeIdxInMesh[p] = vlist[q];
								}
								
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
		void MarchingCubesUtil::mcCatmullClarkSurfaceSubdivision(Common::Mesh::SimpleMesh* inMesh, OUT_ARG Common::Mesh::Mesh* outMesh, i32 iterations) {
			//A temporary pair to avoid duplication
			struct EdgePair {
				i32 x, y;
				bool operator<(const EdgePair& p)const {
					i32 ax = Min(x, y);
					i32 ay = Max(x, y);
					i32 bx = Min(p.x, p.y);
					i32 by = Max(p.y, p.x);
					return ax < bx || (ax == bx && ay < by);
				}
			};
			// Generate Edge From the Simple Mesh
			using namespace Common::Mesh;
			Mesh* oldMesh = new Mesh();
			std::vector<std::vector<int>> ovFaceIdx(inMesh->v.size());
			std::vector<std::vector<int>> ovEdgeIdx(inMesh->v.size());
			for (i32 i = 0; i < inMesh->v.size(); i++) {
				oldMesh->v.push_back(inMesh->v[i]);
			}
			for (i32 i = 0; i < inMesh->f.size(); i++) {
				oldMesh->f.push_back(std::vector<i32>());
				oldMesh->f[i].push_back(inMesh->f[i].a - 1);
				oldMesh->f[i].push_back(inMesh->f[i].b - 1);
				oldMesh->f[i].push_back(inMesh->f[i].c - 1);
				ovFaceIdx[inMesh->f[i].a - 1].push_back(i);
				ovFaceIdx[inMesh->f[i].b - 1].push_back(i);
				ovFaceIdx[inMesh->f[i].c - 1].push_back(i);
				oldMesh->e.push_back({ inMesh->f[i].a - 1 ,inMesh->f[i].b - 1 });
				oldMesh->e.push_back({ inMesh->f[i].b - 1 ,inMesh->f[i].c - 1 });
				oldMesh->e.push_back({ inMesh->f[i].c - 1 ,inMesh->f[i].a - 1 });
				ovEdgeIdx[inMesh->f[i].a - 1].push_back(i * 3);
				ovEdgeIdx[inMesh->f[i].b - 1].push_back(i * 3);
				ovEdgeIdx[inMesh->f[i].b - 1].push_back(i * 3 + 1);
				ovEdgeIdx[inMesh->f[i].c - 1].push_back(i * 3 + 1);
				ovEdgeIdx[inMesh->f[i].c - 1].push_back(i * 3 + 2);
				ovEdgeIdx[inMesh->f[i].a - 1].push_back(i * 3 + 2);
			}
			//Starts iteration
			i32 T = iterations;
			u8 isFirstIteration = true;
			Mesh* oMesh, *nMesh;
			std::vector<std::vector<int>>* oMeshFaceIdx;
			std::vector<std::vector<int>>* oMeshEdgeIdx;

			//Start Conditions
			oMesh = oldMesh;
			if (iterations != 1) {
				nMesh = new Mesh();
			}
			else {
				nMesh = outMesh;
			}
			
			oMeshFaceIdx = &ovFaceIdx;
			oMeshEdgeIdx = &ovEdgeIdx;
			while (T--) {
				//TODO: Indexing
				oMesh->e.clear();
				(*oMeshFaceIdx).clear();
				(*oMeshEdgeIdx).clear();
				for (i32 i = 0; i < oMesh->v.size(); i++) {
					(*oMeshFaceIdx).push_back(std::vector<i32>());
					(*oMeshEdgeIdx).push_back(std::vector<i32>());
				}
				map<EdgePair, i32> pairCheck;
				for (i32 i = 0; i < oMesh->f.size(); i++) {
					for (i32 j = 0; j < oMesh->f[i].size(); j++) {
						//Index the face
						(*oMeshFaceIdx)[oMesh->f[i][j]].push_back(i);
						//Index the edge
						pr_assert(oMesh->f[i][j] != (oMesh->f[i][((usize)j + 1) % oMesh->f[i].size()]));
						if (!pairCheck[{ oMesh->f[i][j], oMesh->f[i][((usize)j + 1) % oMesh->f[i].size()] }]) {
							pairCheck[{ oMesh->f[i][j], oMesh->f[i][((usize)j + 1) % oMesh->f[i].size()] }] = 1;
							oMesh->e.push_back({ oMesh->f[i][j],oMesh->f[i][((usize)j + 1) % oMesh->f[i].size()] });
							(*oMeshEdgeIdx)[oMesh->f[i][j]].push_back(oMesh->e.size() - 1);
							(*oMeshEdgeIdx)[oMesh->f[i][((usize)j + 1) % oMesh->f[i].size()]].push_back(oMesh->e.size() - 1);
						}
					}
				}
				//End of Indexing
				i32* faceCenterIndexF = new i32[oMesh->f.size()];
				set_zero(faceCenterIndexF, sizeof(i32) * (oMesh->f.size()));
				//Traverse tetrahedrons/high-order polygons defined by vertices
				for (i32 i = 0; i < oMesh->v.size(); i++) {
					f64 cuX = oMesh->v[i].x;
					f64 cuY = oMesh->v[i].y;
					f64 cuZ = oMesh->v[i].z;
					std::vector<Vertex> faceCenter;
					std::vector<i32> faceCenterR;
					std::vector<Vertex> edgeCenter;
					std::vector<std::vector<i32>> faceAdjEdge;
					//If num of faces do not equal to num of edges
					if ((*oMeshFaceIdx)[i].size() != (*oMeshEdgeIdx)[i].size()) {
						continue;
					}
					//Calculate face centers
					for (i32 j = 0; j < (*oMeshFaceIdx)[i].size(); j++) {
						faceAdjEdge.push_back(std::vector<i32>());
						i32 faceIndex = (*oMeshFaceIdx)[i][j];
						f64 avX = 0, avY = 0, avZ = 0;
						for (i32 k = 0; k < oMesh->f[faceIndex].size(); k++) {
							avX += oMesh->v[oMesh->f[faceIndex][k]].x;
							avY += oMesh->v[oMesh->f[faceIndex][k]].y;
							avZ += oMesh->v[oMesh->f[faceIndex][k]].z;
						}
						avX /= oMesh->f[faceIndex].size();
						avY /= oMesh->f[faceIndex].size();
						avZ /= oMesh->f[faceIndex].size();
						faceCenter.push_back({ avX,avY,avZ });
						faceCenterR.push_back(faceIndex);
					}
					
					//Calculate edge centers & added edge point
					for (i32 j = 0; j < (*oMeshEdgeIdx)[i].size(); j++) {
						i32 edgeIndex = (*oMeshEdgeIdx)[i][j];
						f64 avX1 = 0, avY1 = 0, avZ1 = 0;
						f64 anX, anY, anZ;
						i32 anIdx;
						for (i32 k = 0; k < 2; k++) {
							Vertex* tmp = &oMesh->v[oMesh->e[edgeIndex][k]];
							avX1 += oMesh->v[oMesh->e[edgeIndex][k]].x;
							avY1 += oMesh->v[oMesh->e[edgeIndex][k]].y;
							avZ1 += oMesh->v[oMesh->e[edgeIndex][k]].z;
							if (Abs(tmp->x - cuX) > eps || Abs(tmp->y - cuY) > eps || Abs(tmp->z - cuZ) > eps) {
								anX = tmp->x;
								anY = tmp->y;
								anZ = tmp->z;
								anIdx = oMesh->e[edgeIndex][k];
							}
						}
						avX1 /= 2.0;
						avZ1 /= 2.0;
						avY1 /= 2.0;
						i32 faceIndex[2], faceCur = 0;
						//Find the neighbour faces
						for (i32 k = 0; k < (*oMeshFaceIdx)[i].size(); k++) {
							i32 fIdx = (*oMeshFaceIdx)[i][k];
							for (i32 q = 0; q < oMesh->f[fIdx].size(); q++) {
								if (oMesh->f[fIdx][q] == anIdx) {
									pr_assert(faceCur < 2);
									faceIndex[faceCur++] = k;
									faceAdjEdge[k].push_back(j);
								}
							}
						}
						//Calculate the center of the face center pair
						f64 avX2 = 0.5 * (faceCenter[faceIndex[0]].x + faceCenter[faceIndex[1]].x);
						f64 avY2 = 0.5 * (faceCenter[faceIndex[0]].y + faceCenter[faceIndex[1]].y);
						f64 avZ2 = 0.5 * (faceCenter[faceIndex[0]].z + faceCenter[faceIndex[1]].z);
						//Obtain the refined edge center
						edgeCenter.push_back({ (avX1 + avX2) * 0.5,(avY1 + avY2) * 0.5,(avZ1 + avZ2) * 0.5 });
					}

					//Readjust original vertex
					Vertex refinedVertex = { 0,0,0 };
					Vertex adjFaceCenter = { 0,0,0 };
					Vertex adjEdgeCenter = { 0,0,0 };
					for (i32 j = 0; j < faceCenter.size(); j++) {
						adjFaceCenter.x += faceCenter[j].x;
						adjFaceCenter.y += faceCenter[j].y;
						adjFaceCenter.z += faceCenter[j].z;
					}
					for (i32 j = 0; j < edgeCenter.size(); j++) {
						adjEdgeCenter.x += edgeCenter[j].x;
						adjEdgeCenter.y += edgeCenter[j].y;
						adjEdgeCenter.z += edgeCenter[j].z;
					}
					pr_assert(faceCenter.size() >= 3);
					i32 nP = faceCenter.size();
					refinedVertex.x = ((nP - 3) * cuX + 2 * adjEdgeCenter.x / edgeCenter.size() + adjFaceCenter.x / faceCenter.size()) / nP;
					refinedVertex.y = ((nP - 3) * cuY + 2 * adjEdgeCenter.y / edgeCenter.size() + adjFaceCenter.y / faceCenter.size()) / nP;
					refinedVertex.z = ((nP - 3) * cuZ + 2 * adjEdgeCenter.z / edgeCenter.size() + adjFaceCenter.z / faceCenter.size()) / nP;

					//Reconstruct the mesh
					i32 nS = nMesh->v.size();
					i32 nF = faceCenter.size();
					i32 nE = edgeCenter.size();
					//Only faces will be generated to avoid edge duplication
					//Insert vertex center
					nMesh->v.push_back({ refinedVertex.x,refinedVertex.y,refinedVertex.z }); //ns
					for (i32 j = 0; j < edgeCenter.size(); j++) {
						nMesh->v.push_back({ edgeCenter[j].x,edgeCenter[j].y,edgeCenter[j].z }); //ns+j+1
					}
					for (i32 j = 0; j < faceCenter.size(); j++) {
						if (faceCenterIndexF[faceCenterR[j]] == 0) {
							nMesh->v.push_back({ faceCenter[j].x,faceCenter[j].y,faceCenter[j].z });
							faceCenterIndexF[faceCenterR[j]] = nMesh->v.size() - 1;
						}
						//Face Generation
						nMesh->f.push_back(std::vector<i32>());
						i32 ts = nMesh->f.size() - 1;
						nMesh->f[ts].push_back(faceCenterIndexF[faceCenterR[j]]);
						nMesh->f[ts].push_back(nS + 1 + faceAdjEdge[j][0]);
						nMesh->f[ts].push_back(nS);
						nMesh->f[ts].push_back(nS + 1 + faceAdjEdge[j][1]);
					}

				}
				if (isFirstIteration) {
					isFirstIteration = false;
					oMesh = nMesh;
					nMesh = new Mesh();
				}
				else {
					delete oMesh;
					oMesh = nMesh;
					if (T != 1) {
						nMesh = new Mesh();
					}
					else {
						nMesh = outMesh;
					}
					
				}
				delete[] faceCenterIndexF;
			}
			delete oldMesh;
		}
	}
}

/*

cout << "EDGE" << (*oMeshEdgeIdx)[i].size() << endl;
for (i32 j = 0; j < (*oMeshEdgeIdx)[i].size(); j++) {
	cout << (*oMeshEdgeIdx)[i][j] << ":";
	for (i32 k = 0; k < oMesh->e[(*oMeshEdgeIdx)[i][j]].size(); k++) {
		cout << oMesh->e[(*oMeshEdgeIdx)[i][j]][k] << ",";
	}
	cout << endl;
}
cout << endl;
cout << "FACE" << (*oMeshFaceIdx)[i].size() << endl;
for (i32 j = 0; j < (*oMeshFaceIdx)[i].size(); j++) {
	cout << (*oMeshFaceIdx)[i][j] << ":";
	for (i32 k = 0; k < oMesh->f[(*oMeshFaceIdx)[i][j]].size(); k++) {
		cout << oMesh->f[(*oMeshFaceIdx)[i][j]][k] << ",";
	}
	cout << endl;
}
cout << endl;
pr_assert((*oMeshEdgeIdx)[i].size() == (*oMeshFaceIdx)[i].size());



*/