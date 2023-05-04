#include "rendering/Capsule.h"

namespace curan {
namespace renderable {

Capsule::Capsule(Info& info) {
    vsg::dvec3 position(0.0, 0.0, 0.0);
    transform = vsg::MatrixTransform::create(vsg::translate(position));

    obj_contained = vsg::Group::create();
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    auto node = info.builder->createCapsule(geomInfo, stateInfo);
    obj_contained->addChild(node);

    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Capsule::make(Info& info) {
    vsg::ref_ptr<Capsule> sphere_to_add = Capsule::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

}
}