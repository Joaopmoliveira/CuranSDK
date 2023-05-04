#ifndef CURAN_RENDERABLE_RENDERABLE_HEADER_FILE_
#define CURAN_RENDERABLE_RENDERABLE_HEADER_FILE_

namespace curan {
    namespace renderable {

        struct Renderable : public vsg::Inherit<vsg::Operation, Renderable> {

            struct Updatable {
                vsg::observer_ptr<vsg::Viewer> viewer;
                vsg::ref_ptr<vsg::Group> attachmentPoint;

                Updatable(vsg::observer_ptr<vsg::Viewer> viewer, vsg::ref_ptr<vsg::Group> attachmentPoint) : viewer{ viewer }, attachmentPoint{ attachmentPoint }
                {}

                Updatable()
                {}
            };

            std::string identifier;
            vsg::ref_ptr<vsg::Group> obj_contained;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            vsg::CompileResult result = vsg::CompileResult{};
            static size_t number;
            Updatable updateinfo;

            Renderable();

            inline void set_identifier(const std::string& ident) {
                identifier = ident;
            }

            inline void update_transform(const vsg::ref_ptr<vsg::MatrixTransform>& new_transform) {
                transform = new_transform;
            }

            void partial_async_attachment(const Updatable& update);

            void run() override;
        };

    }
}

#endif