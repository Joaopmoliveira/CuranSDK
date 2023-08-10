#include "rendering/Mesh.h"

namespace curan{
namespace renderable{

class ReplaceState : public vsg::Inherit<vsg::Visitor, ReplaceState>
{
public:
    vsg::ref_ptr<vsg::GraphicsPipeline> graphicsPipeline;

    ReplaceState(vsg::ref_ptr<vsg::GraphicsPipeline> gp) :
        graphicsPipeline(gp) {}

    void apply(vsg::Object& object) override
    {
        object.traverse(*this);
    }

    void apply(vsg::StateGroup& sg) override
    {
        for (auto& sc : sg.stateCommands) sc->accept(*this);

        sg.traverse(*this);
    }

    void apply(vsg::BindGraphicsPipeline& bgp) override
    {
        bgp.pipeline = graphicsPipeline;
    }
};

Mesh::Mesh(Info& info){
    transform = vsg::MatrixTransform::create(vsg::translate(0.0,0.0,0.0));
    obj_contained = vsg::Group::create();

    vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->formatCoordinateConventions[".obj"] = info.convetion;
    options->formatCoordinateConventions[".step"] = info.convetion;
    options->formatCoordinateConventions[".glb"] = info.convetion;
    options->sceneCoordinateConvention = info.convetion;
    vsg::ref_ptr<vsg::Node> link_mesh = vsg::read_cast<vsg::Node>(info.mesh_path.string(), options);
    options->shaderSets["pbr"] = vsg::createPhysicsBasedRenderingShaderSet(options);
    
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