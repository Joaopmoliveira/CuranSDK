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
                size_t width = 100; //(in pixels)
                size_t height = 100; //(in pixels)
                size_t depth = 100; //(in pixels)
                double spacing_x = 1; //(in milimeters)
                double spacing_y = 1; //(in milimeters)
                double spacing_z = 1; //(in milimeters)
                std::optional<std::string> identifier;
            };

            vsg::ref_ptr<vsg::floatArray3D> textureData;

            Volume(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);

            using updater = std::function<void(vsg::floatArray3D& image)>;

            void update_texture(updater&& update);
        };
    }
}

#endif