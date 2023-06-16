#include "rendering/Box.h"

namespace curan {
namespace renderable {

Box::Box(Info& info) {
    transform = vsg::MatrixTransform::create();
    obj_contained = vsg::Group::create();
    auto node = info.builder->createBox(info.geomInfo, info.stateInfo);
    obj_contained->addChild(node);
    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Box::make(Info& info) {
    vsg::ref_ptr<Box> sphere_to_add = Box::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

}
}