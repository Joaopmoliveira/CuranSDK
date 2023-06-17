#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include <iostream>

struct PhaseCreatedBox : public vsg::Inherit<curan::renderable::Renderable, PhaseCreatedBox>{
    vsg::ref_ptr<vsg::vec3Array> vertices;
    vsg::ref_ptr<vsg::vec3Array> normals;
    vsg::ref_ptr<vsg::vec2Array> texcoords;
    vsg::ref_ptr<vsg::ushortArray> indices;

    PhaseCreatedBox(){

        auto node = vsg::Group::create();

        vsg::vec3 v000(vsg::vec3(0.0,0.0,0.0));
        vsg::vec3 v100(vsg::vec3(0.0001,0.0,0.0));
        vsg::vec3 v010(vsg::vec3(0.0,0.0001,0.0));
        vsg::vec3 v001(vsg::vec3(0.0,0.0,0.0001));

        vsg::vec3 v110 = v100 + v010;
        vsg::vec3 v101 = v100 + v001;
        vsg::vec3 v111 = v100 + v010 + v001;
        vsg::vec3 v011 = v010 + v001;

        vsg::vec3 n0 = normalize(v000 - v111);
        vsg::vec3 n1 = normalize(v100 - v011);
        vsg::vec3 n2 = normalize(v110 - v001);
        vsg::vec3 n3 = normalize(v010 - v101);
        vsg::vec3 n4 = -n2;
        vsg::vec3 n5 = -n3;
        vsg::vec3 n6 = -n0;
        vsg::vec3 n7 = -n1;

        
        vsg::vec2 t00(0.0f, t_origin);
        vsg::vec2 t01(0.0f, t_top);
        vsg::vec2 t10(1.0f, t_origin);
        vsg::vec2 t11(1.0f, t_top);


                // set up vertex and index arrays
        vertices = vsg::vec3Array::create(
            {v000, v100, v110, v010,
             v001, v101, v111, v011});

        normals = vsg::vec3Array::create(
            {n0, n1, n2, n3,
             n4, n5, n6, n7});

        texcoords = vsg::vec2Array::create(
            {t00, t10, t11, t01,
             t00, t10, t11, t01});

        indices = vsg::ushortArray::create(
            {0, 1, 1, 2, 2, 3, 3, 0,
             0, 4, 1, 5, 2, 6, 3, 7,
             4, 5, 5, 6, 6, 7, 7, 4});

        vertices->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;
        normals->properties.dataVariance = vsg::DataVariance::DYNAMIC_DATA;

        // setup geometry
        auto vid = vsg::VertexIndexDraw::create();

        vsg::DataList arrays;
        arrays.push_back(vertices);
        arrays.push_back(normals);
        arrays.push_back(texcoords);
        vid->assignArrays(arrays);

        vid->assignIndices(indices);
        vid->indexCount = static_cast<uint32_t>(indices->size());
        vid->instanceCount = 1;

        node->addChild(vid);
    }
};

int main(int argc, char **argv) {
    try {
        curan::renderable::Window::Info info;
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{1000, 800};
        info.window_size = size;
        curan::renderable::Window window{info};
        window.run();
        window.transverse_identifiers(
            [](const std::unordered_map<std::string, vsg::ref_ptr<curan::renderable::Renderable>>
                   &map) {
                for (auto &p : map){
                    std::cout << "Object contained: " << p.first << '\n';
                }

            });

    } catch (const std::exception& e) {
        std::cerr << "Exception thrown : " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
