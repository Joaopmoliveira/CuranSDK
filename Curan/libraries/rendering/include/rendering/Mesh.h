#ifndef CURAN_FLOOR_RENDERABLE_HEADER_FILE_
#define CURAN_FLOOR_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <optional>

namespace curan{
    namespace renderable{

        struct Mesh : public vsg::Inherit<vsg::Operation, Mesh> {
            struct Info {
                std::optional<std::string> identifier;
                std::filesystem::path json_path;
                vsg::CoordinateConvention convetion = vsg::CoordinateConvention::Z_UP;
            };

            Mesh(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        }
    }
}

#endif