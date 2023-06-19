#ifndef CURAN_PHASE_CREATED_BOX_RENDERABLE_HEADER_FILE_
#define CURAN_PHASE_CREATED_BOX_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"

namespace curan{
    namespace rendering{
        struct PhaseWiredBox : public vsg::Inherit<curan::renderable::Renderable, PhaseWiredBox>{
            vsg::ref_ptr<vsg::vec3Array> vertices;
            vsg::ref_ptr<vsg::vec3Array> normals;
            vsg::ref_ptr<vsg::vec2Array> texcoords;
            vsg::ref_ptr<vsg::ushortArray> indices;
            vsg::ref_ptr<vsg::vec4Array> color;
            static constexpr float epsilon = 0.0001f;

            PhaseWiredBox();

            static vsg::ref_ptr<Renderable> make();

            void update_frame(vsg::vec3 origin);

            void update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset);

            void update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset,vsg::vec3 ydiroffset);

            void update_frame(vsg::vec3 origin,vsg::vec3 xdiroffset,vsg::vec3 ydiroffset, vsg::vec3 zdiroffset);

            void update_frame_config(vsg::vec3 origin,vsg::vec3 xdir = {0.001f,0.0f,0.0f},vsg::vec3 ydir = {0.0f,0.001f,0.0f},vsg::vec3 zdir = {0.0f,0.0f,0.001f});
        };

    }
}

#endif