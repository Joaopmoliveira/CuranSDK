#ifndef CURAN_FLOOR_RENDERABLE_HEADER_FILE_
#define CURAN_FLOOR_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>

namespace curan{
    namespace rendering{

        vsg::ref_ptr<vsg::Node> create_wired_floor(vsg::ref_ptr<vsg::Builder> builder);

    }
}

#endif