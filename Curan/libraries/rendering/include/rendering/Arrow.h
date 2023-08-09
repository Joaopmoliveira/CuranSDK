#ifndef CURAN_ARROW_RENDERABLE_HEADER_FILE_
#define CURAN_ARROW_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>

namespace curan {
    namespace renderable {
        struct Arrow : public vsg::Inherit<Renderable, Arrow> {
            struct Info {
                std::optional<std::string> identifier;
                float relative_size = 0.01;
            };
            // we need two transforms, one connected to the cylinder, which we 
            // scale and another associated with the tip of the arrow
            vsg::ref_ptr<vsg::MatrixTransform> scale;
            vsg::ref_ptr<vsg::MatrixTransform> tip;
            
            Arrow(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);

            void norm(const double& in);
        };
    }
}

#endif