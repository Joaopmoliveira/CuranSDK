#include "rendering/Arrow.h"

namespace curan {
namespace renderable {

Arrow::Arrow(Info& info) {
    transform = vsg::MatrixTransform::create();
    scale = vsg::MatrixTransform::create();
    tip = vsg::MatrixTransform::create();
    obj_contained = vsg::Group::create();

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(0.2f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.2f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 0.2f);

    vsg::StateInfo stateInfo;
    stateInfo.lighting = true;
    stateInfo.two_sided = true;
    auto builder = vsg::Builder::create();

    auto vectornorm = builder->createCylinder(geomInfo, stateInfo);

    geomInfo.dx.set(0.2f, 0.0f, 0.0f);
    geomInfo.dy.set(0.0f, 0.2f, 0.0f);
    geomInfo.dz.set(0.0f, 0.0f, 0.2f);
    auto orientationtip = builder->createCone(geomInfo,stateInfo);
    scale->addChild(vectornorm);
    tip->addChild(orientationtip);
    norm(0.2);
    obj_contained->addChild(scale);
    obj_contained->addChild(tip);

    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Arrow::make(Info& info) {
    vsg::ref_ptr<Arrow> sphere_to_add = Arrow::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

void Arrow::norm(const double& in){
    scale->matrix = vsg::scale(1.0,1.0,in);
    tip->matrix = vsg::translate(0.0,0.0,in);
}

}
}