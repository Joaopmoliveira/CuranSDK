#include "rendering/Mesh.h"

namespace curan{
namespace renderable{

Mesh::Mesh(Info& info){
    transform = vsg::MatrixTransform::create(vsg::translate(0.0,0.0,0.0));
    obj_contained = vsg::Group::create();

    vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->formatCoordinateConventions[".obj"] = info.convetion;
    options->sceneCoordinateConvention = info.convetion;
    vsg::ref_ptr<vsg::Node> link_mesh = vsg::read_cast<vsg::Node>(info.mesh_path.string(), options);
    if(!link_mesh)
        throw std::runtime_error("failed to load one of the links");
    obj_contained->addChild(link_mesh);
    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> Mesh::make(Info& info){
    vsg::ref_ptr<Mesh> arm = Mesh::create(info);
    vsg::ref_ptr<Renderable> val = arm.cast<Renderable>();
    return val;
}

}
}