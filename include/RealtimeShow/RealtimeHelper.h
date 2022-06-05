#pragma once

#include <easy3d/viewer/viewer.h>
#include <easy3d/fileio/ply_reader_writer.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/surface_mesh_builder.h>
#include <easy3d/fileio/translator.h>

using namespace easy3d;
using namespace std;

namespace RealtimeShow{
    class RealtimeHelper{
        public:
            static void getElements(vector<io::Element> &need, 
                              vector<vec3> points, 
                              vector<vec3> colors, 
                            //   vector<vec3> normals, 
                              int vertex_num, 
                              vector<vector<int>> faces, 
                              int face_num);

            static bool getMesh(std::vector<io::Element> &elements, SurfaceMesh *mesh);
    };
}
