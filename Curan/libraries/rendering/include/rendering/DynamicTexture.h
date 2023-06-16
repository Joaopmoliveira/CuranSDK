#ifndef CURAN_DYNAMIC_TEXTURE_RENDERABLE_HEADER_FILE_
#define CURAN_DYNAMIC_TEXTURE_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>

namespace curan {
    namespace renderable {
        struct DynamicTexture : public vsg::Inherit<Renderable, DynamicTexture> {

            size_t width;
            size_t height;
            vsg::ref_ptr<vsg::Data> textureData;

            struct Info {
                size_t width = 100;
                size_t height = 100;
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            DynamicTexture(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);

            using updater = std::function<void(vsg::vec4Array2D& image)>;

            void update_texture(updater&& update);
        };

    }
}

#endif