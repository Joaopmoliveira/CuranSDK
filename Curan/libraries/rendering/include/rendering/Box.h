#ifndef CURAN_BOX_RENDERABLE_HEADER_FILE_
#define CURAN_BOX_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>

namespace curan {
    namespace renderable {
        struct Box : public vsg::Inherit<Renderable, Box> {
            struct Info {
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
                vsg::GeometryInfo geomInfo;
                vsg::StateInfo stateInfo;
            };

            Box(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
    }
}

#endif