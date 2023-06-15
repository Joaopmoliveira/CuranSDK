#include "rendering/Floor.h"

namespace curan{
    namespace rendering{

        vsg::ref_ptr<vsg::Node> create_wired_floor(vsg::ref_ptr<vsg::Builder> builder){
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

            auto iterator = positions->begin();
            for(double y_pos = lim_y_min; y_pos < lim_y_max ; y_pos+=spacing ){
                (*iterator).y = y_pos;
                for(double x_pos = lim_x_min; x_pos < lim_x_max ; x_pos+=spacing ){
                    if(iterator==positions->end())
                        throw std::runtime_error("Iterator went past the maximum distance");
                    (*iterator).x = x_pos;
                    ++iterator;
                }
            }
            geomInfo.positions = positions;
        }

    }
}