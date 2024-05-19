#ifndef CURAN_RENDERABLE_RENDERABLE_HEADER_FILE_
#define CURAN_RENDERABLE_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <string>

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

            std::string _identifier;
            vsg::ref_ptr<vsg::Group> obj_contained;
            vsg::ref_ptr<vsg::MatrixTransform> transform;
            vsg::ref_ptr<vsg::Viewer> owner_viewer;
            vsg::CompileResult result = vsg::CompileResult{};
            static size_t number;
            Updatable updateinfo;

            Renderable();

            inline std::string identifier(){
                return _identifier;
            }

            inline void set_identifier(const std::string& ident) {
                _identifier = ident;
            }

            inline void update_transform(const vsg::dmat4& in_matrix) {
                transform->matrix = in_matrix;
            }

            void partial_async_attachment(const Updatable& update);

            void run() override;

            virtual void append(vsg::ref_ptr<Renderable> link_to_join);
        };

    }
}

#endif