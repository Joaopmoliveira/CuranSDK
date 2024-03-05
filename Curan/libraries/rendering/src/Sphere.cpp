#include "rendering/Sphere.h"

namespace curan {
namespace renderable {

Sphere::Sphere(Info& info) {
    vsg::dvec3 position(0.0, 0.0, 0.0);
    if (info.position)
        position = *info.position;
        
    transform = vsg::MatrixTransform::create(vsg::translate(position));

    obj_contained = vsg::Group::create();
    auto node = info.builder->createSphere(info.geomInfo, info.stateInfo);
    obj_contained->addChild(node);

    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Sphere::make(Info& info) {
    vsg::ref_ptr<Sphere> sphere_to_add = Sphere::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

}
}