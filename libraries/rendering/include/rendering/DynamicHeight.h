#ifndef CURAN_DYNAMIC_HEIGHT_RENDERABLE_HEADER_FILE_
#define CURAN_DYNAMIC_HEIGHT_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>
#include <array>

namespace curan {
    namespace renderable {
        struct DynamicHeight : public vsg::Inherit<Renderable, DynamicHeight> {

            size_t width;
            size_t height;
            size_t depth;
            vsg::ref_ptr<vsg::Data> textureData;

            struct Info {
                size_t width = 100;
                size_t height = 100;
                size_t depth = 100;
                std::array<double,3> origin;
                std::array<double,3> spacing; 
                vsg::StateInfo stateInfo;
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            DynamicHeight(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);

            using updater = std::function<void(vsg::floatArray2D& image)>;

            void update_texture(updater&& update);
        };

    }
}

#endif