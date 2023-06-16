#include "rendering/Renderable.h"

namespace curan {
namespace renderable {

Renderable::Renderable() : identifier{ "object" + std::to_string(number) } {
    ++number;
}

void Renderable::partial_async_attachment(const Updatable& update) {
    updateinfo = update;
    owner_viewer = updateinfo.viewer;

    if (!updateinfo.attachmentPoint)
        return;

    transform->addChild(obj_contained);

    result = owner_viewer->compileManager->compile(obj_contained);
}

void Renderable::run() {
    vsg::ref_ptr<vsg::Viewer> ref_viewer = updateinfo.viewer;
    if (ref_viewer)
        updateViewer(*ref_viewer, result);
    updateinfo.attachmentPoint->addChild(transform);
}

void Renderable::append(vsg::ref_ptr<Renderable> link_to_join, vsg::ref_ptr<vsg::MatrixTransform> relative_transformation){
    if (!link_to_join->obj_contained)
        return ;
    vsg::observer_ptr<vsg::Viewer> observer_viewer(owner_viewer);
    partial_async_attachment({ observer_viewer,obj_contained });
    owner_viewer->addUpdateOperation(link_to_join);
}

size_t Renderable::number = 0;

}
}