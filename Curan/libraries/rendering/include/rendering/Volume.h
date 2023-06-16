#ifndef CURAN_VOLUME_RENDERABLE_HEADER_FILE_
#define CURAN_VOLUME_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>

namespace curan {
    namespace renderable {
        struct Volume : public vsg::Inherit<Renderable, Volume>{
            struct Info {
                size_t width = 100;
                size_t height = 100;
                vsg::GeometryInfo geomInfo;
                vsg::StateInfo stateInfo;
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            Volume(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);

            using updater = std::function<void(vsg::vec4Array2D& image)>;

            void update_texture(updater&& update);
        }
    }
}

#endif