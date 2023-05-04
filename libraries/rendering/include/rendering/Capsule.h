#ifndef CURAN_CAPSULE_RENDERABLE_HEADER_FILE_
#define CURAN_CAPSULE_RENDERABLE_HEADER_FILE_

namespace curan {
    namespace renderable {
        struct Capsule : public vsg::Inherit<Renderable, Capsule> {
            struct Info {
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            Capsule(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
    }
}

#endif