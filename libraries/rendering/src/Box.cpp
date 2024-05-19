#include "rendering/Box.h"

namespace curan {
namespace renderable {

Box::Box(Info& info) {
    transform = vsg::MatrixTransform::create();
    scale = vsg::MatrixTransform::create();
    obj_contained = vsg::Group::create();
    auto node = info.builder->createBox(info.geomInfo, info.stateInfo);
    scale->addChild(node);
    obj_contained->addChild(scale);
    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Box::make(Info& info) {
    vsg::ref_ptr<Box> sphere_to_add = Box::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

void Box::set_scale(double sx, double sy, double sz){
    scale->matrix = vsg::scale(sx,sy,sz);
}

}
}