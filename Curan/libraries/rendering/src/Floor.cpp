#include "rendering/Floor.h"

namespace curan{
    namespace renderable{

        vsg::ref_ptr<vsg::Node> create_wired_floor(){
            double lim_x_min = -10;
            double lim_y_min = -10;

            double lim_x_max = 10;
            double lim_y_max = 10;

            double spacing = 0.5;

            size_t number_of_x_vertices = (lim_x_max-lim_x_min)/spacing;
            size_t number_of_y_vertices = (lim_y_max-lim_y_min)/spacing;
            size_t numVertices = number_of_x_vertices * number_of_y_vertices;

            vsg::StateInfo stateInfo;
            stateInfo.instance_positions_vec3 = true;
            stateInfo.wireframe = true;
            vsg::GeometryInfo geomInfo;
            geomInfo.dx.set(1.0f, 0.0f, 0.0f);
            geomInfo.dy.set(0.0f, 1.0f, 0.0f);
            geomInfo.dz.set(0.0f, 0.0f, 1.0f);
            
            auto positions = vsg::vec4Array::create(numVertices);

            size_t number_of_x_vert = std::ceil((lim_x_max-lim_x_min)/spacing);
            size_t number_of_y_vert = std::ceil((lim_y_max-lim_y_min)/spacing);
            std::vector<vsg::vec4> positions_vector;
            positions_vector.reserve(number_of_x_vert*number_of_y_vert);

            for(double y_pos = lim_y_min; y_pos < lim_y_max ; y_pos+=spacing ){
                vsg::vec4 local_pos;
                local_pos.y = y_pos;
                local_pos.z = 0;
                local_pos.w = 0;
                for(double x_pos = lim_x_min; x_pos < lim_x_max ; x_pos+=spacing )
                    local_pos.x = x_pos;
            }
            auto iterator = positions->begin();
            for(const auto & pos : positions_vector ){
                *iterator = pos;
            }

            geomInfo.positions = positions;

            auto builder = vsg::Builder::create();
            auto scene = vsg::Group::create();
            scene->addChild(builder->createQuad(geomInfo, stateInfo));

            return scene;

        }

    }
}