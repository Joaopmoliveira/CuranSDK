#ifndef CURAN_CYLINDER_RENDERABLE_HEADER_FILE_
#define CURAN_CYLINDER_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>

namespace curan {
    namespace renderable {
        struct Cylinder : public vsg::Inherit<Renderable, Cylinder> {
            struct Info {
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            Cylinder(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
    }
}

#endif