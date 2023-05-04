#ifndef CURAN_SPHERE_RENDERABLE_HEADER_FILE_
#define CURAN_SPHERE_RENDERABLE_HEADER_FILE_

namespace curan {
    namespace renderable {
        struct Sphere : public vsg::Inherit<Renderable, Sphere> {
            struct Info {
                vsg::ref_ptr<vsg::Builder> builder;
                std::optional<std::string> identifier;
            };

            Sphere(Info& info);

            static vsg::ref_ptr<Renderable> make(Info& info);
        };
    }
}

#endif