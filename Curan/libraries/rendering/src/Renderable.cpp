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

    vsg::ComputeBounds computeBounds;
    obj_contained->accept(computeBounds);

    vsg::dvec3 centre = (computeBounds.bounds.min + computeBounds.bounds.max) * 0.5;
    double radius = vsg::length(computeBounds.bounds.max - computeBounds.bounds.min) * 0.5;
    auto scale = vsg::MatrixTransform::create(vsg::scale(1.0 / radius, 1.0 / radius, 1.0 / radius) * vsg::translate(-centre));

    scale->addChild(obj_contained);
    transform->addChild(scale);

    result = ref_viewer->compileManager->compile(obj_contained);
}

void Renderable::run() {
    vsg::ref_ptr<vsg::Viewer> ref_viewer = updateinfo.viewer;
    if (ref_viewer)
        updateViewer(*ref_viewer, result);
    updateinfo.attachmentPoint->addChild(transform);
}

void Renderable::append(vsg::ref_ptr<Renderable> link_to_join, vsg::ref_ptr<vsg::MatrixTransform> relative_transformation){
    relative_transformation->addChild(link_to_join->transform);
    transform->addChild(relative_transformation);
}

size_t Renderable::number = 0;

}
}