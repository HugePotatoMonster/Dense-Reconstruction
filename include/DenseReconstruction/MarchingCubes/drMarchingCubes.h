#pragma once
#include "../../Common/cmTypeDefs.h"
#include "../../TSDF/TSDFVolume.h"

#define MC_VOXEL_CLASS TSDF::TSDFVolume

namespace DenseReconstruction {
	namespace MarchingCubes {
		class MarchingCubesUtil {
		public:
			//Convert a volume consisting voxels to a mesh without coloring and vertex interpolation.
			//Marching cubes is the adopted algorithm
			//This deprecates marching cubes implemented in `Common::Utility` namespace
			//Constants are obtained from a Unity Marching Cube implementation
			//https://github.com/Scrawk/Marching-Cubes/blob/master/Assets/MarchingCubes/Marching/MarchingCubes.cs
			// @author: 1950641
			// @param volume: (Input) TSDF Volume
			// @param outMesh: (Output) Pointer to the output mesh
			static void mcConvertToMesh(MC_VOXEL_CLASS* volume, OUT_ARG Common::Mesh::SimpleMesh* outMesh);

			//Convert a volume consisting voxels to a colored mesh without vertex interpolation.
			//Marching cubes is the adopted algorithm
			//This deprecates marching cubes implemented in `Common::Utility` namespace
			//Constants are obtained from a Unity Marching Cube implementation
			//https://github.com/Scrawk/Marching-Cubes/blob/master/Assets/MarchingCubes/Marching/MarchingCubes.cs
			// @author: 1950641
			// @param volume: (Input) TSDF Volume
			// @param outMesh: (Output) Pointer to the vertex-colorable output mesh
			static void mcConvertToColoredMesh(MC_VOXEL_CLASS* volume, OUT_ARG Common::Mesh::ColoredSimpleMesh* outMesh);

			//Apply Catmull-Clark surface subdivison operation (namely,a kind of mesh smoothing) to a given mesh
			// @author: 1950641
			// @param: inMesh: (Input) a mesh with only vertices and triangular faces
			// @param: outMesh: (Output) a refined mesh
			// @param: iterations: (Input) iterations
			static void mcCatmullClarkSurfaceSubdivision(Common::Mesh::ColoredSimpleMesh* inMesh, OUT_ARG Common::Mesh::Mesh* outMesh, i32 iterations = 1);
		};
	}
}