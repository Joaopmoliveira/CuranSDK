#ifndef CURAN_BOX_RENDERABLE_HEADER_FILE_
#define CURAN_BOX_RENDERABLE_HEADER_FILE_

namespace curan {
    namespace renderable {
        struct Box : public vsg::Inherit<Renderable, Box> {
            struct Info {
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            Box(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
    }
}

#endif