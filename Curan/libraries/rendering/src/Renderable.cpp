#include "rendering/Renderable.h"

namespace curan {
namespace renderable {

Renderable::Renderable() : identifier{ "object" + std::to_string(number) } {
    ++number;
}

void Renderable::partial_async_attachment(const Updatable& update) {
    updateinfo = update;
    vsg::ref_ptr<vsg::Viewer> ref_viewer = updateinfo.viewer;

    if (!updateinfo.attachmentPoint)
        return;

    transform->addChild(obj_contained);

    result = ref_viewer->compileManager->compile(obj_contained);
}

void Renderable::run() {
    vsg::ref_ptr<vsg::Viewer> ref_viewer = updateinfo.viewer;
    if (ref_viewer)
        updateViewer(*ref_viewer, result);
    updateinfo.attachmentPoint->addChild(transform);
}

void Renderable::append(vsg::ref_ptr<Renderable> link_to_join, vsg::ref_ptr<vsg::MatrixTransform> relative_transformation){

}

size_t Renderable::number = 0;

}
}