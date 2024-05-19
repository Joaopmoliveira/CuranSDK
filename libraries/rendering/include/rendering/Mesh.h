#ifndef CURAN_MESH_RENDERABLE_HEADER_FILE_
#define CURAN_MESH_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>
#include <filesystem>

namespace curan{
    namespace renderable{

        struct Mesh : public vsg::Inherit<Renderable, Mesh> {
            struct Info {
                std::optional<std::string> identifier;
                std::filesystem::path mesh_path;
                vsg::CoordinateConvention convetion = vsg::CoordinateConvention::Z_UP;
            };

            Mesh(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
        
    }
}

#endif