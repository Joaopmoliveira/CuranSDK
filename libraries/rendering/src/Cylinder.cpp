#include "renderable/Cylinder.h"

namespace curan {
namespace renderable {

Cylinder::Cylinder(Info& info) {
    vsg::dvec3 position(0.0, 0.0, 0.0);
    transform = vsg::MatrixTransform::create(vsg::translate(position));

    obj_contained = vsg::Group::create();
    vsg::GeometryInfo geomInfo;
    vsg::StateInfo stateInfo;
    auto node = info.builder->createCylinder(geomInfo, stateInfo);
    obj_contained->addChild(node);

    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Cylinder::make(Info& info) {
    vsg::ref_ptr<Cylinder> sphere_to_add = Cylinder::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

}
}