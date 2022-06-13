#include <easy3d/viewer/viewer.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/core/surface_mesh_builder.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/util/timer.h>
#include <easy3d/core/random.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <easy3d/fileio/ply_reader_writer.h>
#include <easy3d/fileio/translator.h>
#include <string>

using namespace easy3d;
using namespace std;

namespace details {

    template <typename PropertyT>
    inline bool extract_named_property(std::vector<PropertyT>& properties, PropertyT& wanted, const std::string& name) {
        typename std::vector< PropertyT >::iterator it = properties.begin();
        for (; it != properties.end(); ++it) {
            const PropertyT& prop = *it;
            if (prop.name == name) {
                wanted = prop;
                properties.erase(it);
                return true;
            }
        }
        return false;
    }


    template <typename T, typename PropertyT>
    inline void add_vertex_properties(SurfaceMesh* mesh, const std::vector<PropertyT>& properties)
    {
        for (const auto& p : properties) {
            std::string name = p.name;
            if (p.size() != mesh->n_vertices()) {
                LOG(ERROR) << "vertex property size (" << p.size() << ") does not match number of vertices (" << mesh->n_vertices() << ")";
                continue;
            }
            if (name.find("v:") == std::string::npos)
                name = "v:" + name;
            auto prop = mesh->vertex_property<T>(name);
            prop.vector() = p;
        }
    }


    template <typename T, typename PropertyT>
    inline void add_face_properties(SurfaceMesh* mesh, const std::vector<PropertyT>& properties)
    {
        for (const auto& p : properties) {
            std::string name = p.name;
            if (p.size() != mesh->n_faces()) {
                LOG(ERROR) << "face property size (" << p.size() << ") does not match number of faces (" << mesh->n_faces() << ")";
                continue;
            }
            if (name.find("f:") == std::string::npos)
                name = "f:" + name;
            auto prop = mesh->face_property<T>(name);
            prop.vector() = p;
        }
    }


    template <typename T, typename PropertyT>
    inline void add_edge_properties(SurfaceMesh* mesh, const std::vector<PropertyT>& properties)
    {
        for (const auto& p : properties) {
            std::string name = p.name;
            if (p.size() != mesh->n_edges()) {
                LOG(ERROR) << "edge property size (" << p.size() << ") does not match number of edges (" << mesh->n_edges() << ")";
                continue;
            }
            if (name.find("e:") == std::string::npos)
                name = "e:" + name;
            auto prop = mesh->edge_property<T>(name);
            prop.vector() = p;
        }
    }

}

int flag = 0;

void load_change(SurfaceMesh *mesh, Viewer *viewer){

    viewer->delete_model(mesh);

    if (flag==0){
        mesh = SurfaceMeshIO::load("D:\\Code\\tsdf-fusion-python-master\\mesh_new_test.ply");
        flag = 1;
        std::cout << "1" << std::endl;

    }
    else{
        mesh = SurfaceMeshIO::load("D:\\Code\\tsdf-fusion-python-master\\mesh_p_fixed.ply");
        flag = 0;
        std::cout << "0" << std::endl;
    }

    viewer->add_model(mesh);

    viewer->update();
}

int change_test(){

    SurfaceMesh* mesh = new SurfaceMesh;

    Viewer viewer("Test");

    mesh = SurfaceMeshIO::load("D:\\Code\\tsdf-fusion-python-master\\mesh_p_fixed.ply");

    viewer.add_model(mesh);

    Timer<SurfaceMesh *, Viewer *> timer;

    timer.set_interval(2000, load_change, mesh, &viewer);

    return viewer.run();
}

bool getElements(vector<io::Element> &need, vector<vec3> points, vector<vec3> colors, int vertex_num, vector<vector<int>> faces, int face_num){
    io::Element vertex_element("vertex", vertex_num);

    io::Vec3Property point_property("point", points);
    // io::Vec3Property normal_property("normal", normals);
    io::Vec3Property color_property("color", colors);

    vertex_element.vec3_properties.push_back(point_property);
    // vertex_element.vec3_properties.push_back(normal_property);
    vertex_element.vec3_properties.push_back(color_property);

    io::Element face_element("face", face_num);

    io::IntListProperty face_int_list_property("vertex_index",faces);
    face_element.int_list_properties.push_back(face_int_list_property);

    need.push_back(face_element);
    need.push_back(vertex_element);

    return true;
}

SurfaceMesh* elements2Mesh(std::vector<io::Element> &elements, string file_name){

    std::setlocale(LC_NUMERIC, "C");

    SurfaceMesh *mesh = new SurfaceMesh;
    mesh->set_name(file_name);

    io::Vec3Property       coordinates;
    io::IntListProperty    face_vertex_indices;
    io::FloatListProperty  face_halfedge_texcoords;
    io::IntListProperty    edge_vertex_indices;

    const io::Element* element_vertex = nullptr;
    for (std::size_t i = 0; i < elements.size(); ++i) {
        io::Element& e = elements[i];
        if (e.name == "vertex") {
            element_vertex = &e;
            if (details::extract_named_property(e.vec3_properties, coordinates, "point"))
                continue;
            else {
                LOG(ERROR) << "vertex coordinates (x, y, z properties) do not exist";
                return false;
            }
        }
        else if (e.name == "face") {
            if (details::extract_named_property(e.int_list_properties, face_vertex_indices, "vertex_indices") ||
                details::extract_named_property(e.int_list_properties, face_vertex_indices, "vertex_index")) {
                details::extract_named_property(e.float_list_properties, face_halfedge_texcoords, "texcoord");
                continue;
            }
            else {
                LOG(ERROR) << "edge properties might not be parsed correctly because both 'vertex_indices' and 'vertex_index' not defined on faces";
                return false;
            }
        }
        else if (e.name == "edge") {
            if (details::extract_named_property(e.int_list_properties, edge_vertex_indices, "vertex_indices") ||
                details::extract_named_property(e.int_list_properties, edge_vertex_indices, "vertex_index"))
                continue;
            else {
                LOG(ERROR)
                        << "edge properties might not be parsed correctly because both 'vertex_indices' and 'vertex_index' not defined on edges";
                // return false; // no return because the model can still be visualized
            }
        }
    }

    mesh->clear();

    SurfaceMeshBuilder builder(mesh);
    builder.begin_surface();

    // add vertices
    for (auto p : coordinates)
        builder.add_vertex(p);

    if (element_vertex) {// add vertex properties
        // NOTE: to properly handle non-manifold meshes, vertex properties must be added before adding the faces
        details::add_vertex_properties<vec3>(mesh, element_vertex->vec3_properties);
        details::add_vertex_properties<vec2>(mesh, element_vertex->vec2_properties);
        details::add_vertex_properties<float>(mesh, element_vertex->float_properties);
        details::add_vertex_properties<int>(mesh, element_vertex->int_properties);
        details::add_vertex_properties<std::vector<int> >(mesh, element_vertex->int_list_properties);
        details::add_vertex_properties<std::vector<float> >(mesh, element_vertex->float_list_properties);
    } else {
        LOG(ERROR) << "element 'vertex' not found";
    }

    // add faces

    // create texture coordinate property if texture coordinates present
    SurfaceMesh::HalfedgeProperty<vec2> prop_texcoords;
    if (face_halfedge_texcoords.size() == face_vertex_indices.size())
        prop_texcoords = prop_texcoords = mesh->add_halfedge_property<vec2>("h:texcoord");

    // find the face's halfedge that points to v.
    auto find_face_halfedge = [](SurfaceMesh *mesh, SurfaceMesh::Face face,
                                    SurfaceMesh::Vertex v) -> SurfaceMesh::Halfedge {
        for (auto h : mesh->halfedges(face)) {
            if (mesh->target(h) == v)
                return h;
        }
        LOG_N_TIMES(3, ERROR) << "could not find a halfedge pointing to " << v << " in face " << face
                                << ". " << COUNTER;
        return SurfaceMesh::Halfedge();
    };

    for (std::size_t i=0; i<face_vertex_indices.size(); ++i) {
        const auto& indices = face_vertex_indices[i];
        std::vector<SurfaceMesh::Vertex> vts;
        for (auto id : indices)
            vts.emplace_back(SurfaceMesh::Vertex(id));
        auto face = builder.add_face(vts);

        // now let's add the texcoords (defined on halfedges)
        if (face.is_valid() && prop_texcoords) {
            const auto& face_texcoords = face_halfedge_texcoords[i];
            if (face_texcoords.size() == vts.size() * 2) { // 2 coordinates per vertex
                auto begin = find_face_halfedge(mesh, face, builder.face_vertices()[0]);
                auto cur = begin;
                unsigned int texcord_idx = 0;
                do {
                    prop_texcoords[cur] = vec2(face_texcoords[texcord_idx], face_texcoords[texcord_idx + 1]);
                    texcord_idx += 2;
                    cur = mesh->next(cur);
                } while (cur != begin);
            }
        }
    }

    // now let's add the remained properties
    for (std::size_t i = 0; i < elements.size(); ++i) {
        io::Element& e = elements[i];
        if (e.name == "vertex") {
            continue;   // the vertex property has already been added
        }
        else if (e.name == "face") {
            details::add_face_properties<vec3>(mesh, e.vec3_properties);
            details::add_face_properties<vec2>(mesh, e.vec2_properties);
            details::add_face_properties<float>(mesh, e.float_properties);
            details::add_face_properties<int>(mesh, e.int_properties);
            details::add_face_properties< std::vector<int> >(mesh, e.int_list_properties);
            details::add_face_properties< std::vector<float> >(mesh, e.float_list_properties);
        }
        else if (e.name == "edge") {
            details::add_edge_properties<vec3>(mesh, e.vec3_properties);
            details::add_edge_properties<vec2>(mesh, e.vec2_properties);
            details::add_edge_properties<float>(mesh, e.float_properties);
            details::add_edge_properties<int>(mesh, e.int_properties);
            details::add_edge_properties< std::vector<int> >(mesh, e.int_list_properties);
            details::add_edge_properties< std::vector<float> >(mesh, e.float_list_properties);
        }
        else {
            const std::string name = "element-" + e.name;
            auto prop = mesh->add_model_property<io::Element>(name, io::Element(""));
            prop.vector().push_back(e);
            LOG(WARNING) << "unknown element '" << e.name
                            << "' with the following properties has been stored as a model property '" << name << "'"
                            << e.property_statistics();
        }
    }

    builder.end_surface();

    if (Translator::instance()->status() == Translator::TRANSLATE_USE_FIRST_POINT) {
        auto& points = mesh->get_vertex_property<vec3>("v:point").vector();

        // the first point
        const vec3 p0 = points[0];
        const dvec3 origin(p0.data());
        Translator::instance()->set_translation(origin);

        for (auto& p: points)
            p -= p0;

        auto trans = mesh->add_model_property<dvec3>("translation", dvec3(0, 0, 0));
        trans[0] = origin;
        LOG(INFO) << "model translated w.r.t. the first vertex (" << origin
                    << "), stored as ModelProperty<dvec3>(\"translation\")";
    } else if (Translator::instance()->status() == Translator::TRANSLATE_USE_LAST_KNOWN_OFFSET) {
        const dvec3 &origin = Translator::instance()->translation();
        auto& points = mesh->get_vertex_property<vec3>("v:point").vector();
        for (auto& p: points) {
            p.x -= origin.x;
            p.y -= origin.y;
            p.z -= origin.z;
        }

        auto trans = mesh->add_model_property<dvec3>("translation", dvec3(0, 0, 0));
        trans[0] = origin;
        LOG(INFO) << "model translated w.r.t. last known reference point (" << origin
                    << "), stored as ModelProperty<dvec3>(\"translation\")";
    }

    return mesh;
}

void mine_ply_test(){
    std::vector<io::Element> elements;

    vector<vec3> points = {
        vec3(0,0,0),
        vec3(0,0,1),
        vec3(0,1,1),
        vec3(1,0,1)
    };

    vector<vec3> colors = {
        vec3(255,0,0),
        vec3(0,255,0),
        vec3(0,0,255),
        vec3(0,0,0)
    };

    int vertex_num = 4;

    vector<vector<int>> faces = {
        vector<int>({0,1,2}),
        vector<int>({3,2,1})
    };

    int face_num = 2;

    getElements(elements, points, colors, vertex_num, faces, face_num);

    SurfaceMesh* mesh = new SurfaceMesh;

    // elements2Mesh(elements,mesh);

    Viewer viewer("Test");

    viewer.add_model(mesh);



    viewer.run();
}

// int main(){
//     mine_ply_test();
// }