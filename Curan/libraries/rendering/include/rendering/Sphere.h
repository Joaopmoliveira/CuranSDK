#ifndef CURAN_SPHERE_RENDERABLE_HEADER_FILE_
#define CURAN_SPHERE_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>

namespace curan {
    namespace renderable {
        struct Sphere : public vsg::Inherit<Renderable, Sphere> {
            struct Info {
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
                vsg::GeometryInfo geomInfo;
                vsg::StateInfo stateInfo;
            };

            Sphere(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
    }
}

#endif