#include "rendering/Floor.h"

namespace curan{
    namespace renderable{

        vsg::ref_ptr<vsg::Node> create_wired_floor(){
            float lim_x_min = -10;
            float lim_y_min = -10;
            float lim_x_max = 10;
            float lim_y_max = 10;
            float spacing = 0.5;

            vsg::StateInfo stateInfo;
            stateInfo.wireframe = true;
            vsg::GeometryInfo geomInfo;
            geomInfo.color = vsg::vec4(1.0f,1.0f,1.0f,1.0f);

            auto builder = vsg::Builder::create();
            auto scene = vsg::Group::create();

            for(float y_pos = lim_y_min; y_pos < lim_y_max ; y_pos+=spacing )
                for(float x_pos = lim_x_min; x_pos < lim_x_max ; x_pos+=spacing ){
                    geomInfo.dx.set(spacing,0.0f,0.0f);
                    geomInfo.dy.set(0.0f,spacing,0.0f);
                    geomInfo.dz.set(0.0f,0.0f,0.0f);
                    geomInfo.position.set(x_pos,y_pos,0.0f);
                    scene->addChild(builder->createQuad(geomInfo, stateInfo));
                }

            return scene;
        }

    }
}