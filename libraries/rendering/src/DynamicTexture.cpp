#include "rendering/DynamicTexture.h"

namespace curan {
namespace renderable {

DynamicTexture::DynamicTexture(Info& info) : width{ info.width }, height{ info.height } {
    vsg::dvec3 position(0.0, 0.0, 0.0);
    transform = vsg::MatrixTransform::create(vsg::translate(position));

    obj_contained = vsg::Group::create();

    textureData = vsg::vec4Array2D::create(width, height);
    textureData->properties.dataVariance = vsg::DYNAMIC_DATA;
    textureData->properties.format = VK_FORMAT_R32G32B32A32_SFLOAT;
    textureData->properties.origin = vsg::BOTTOM_LEFT;

    info.stateInfo.two_sided = true;
    info.stateInfo.greyscale = true;
    info.stateInfo.image = textureData;
    info.stateInfo.lighting = true;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx = vsg::vec3(static_cast<float>(info.spacing[0]*info.width),0.0f,0.0f);
    geomInfo.dy = vsg::vec3(0.0f, static_cast<float>(info.spacing[1]*info.height),0.0f);
    geomInfo.dz = vsg::vec3(0.0f,0.0f,0.0f);
    geomInfo.position = vsg::vec3(static_cast<float>(info.origin[0])+ static_cast<float>(info.spacing[0]*info.width)/2.0f, static_cast<float>(info.origin[1])+ static_cast<float>(info.spacing[1]*info.height)/2.0f, static_cast<float>(info.origin[2]));

    auto node = info.builder->createQuad(geomInfo, info.stateInfo);

    obj_contained->addChild(node);

    if (info.identifier)
        set_identifier(*info.identifier);
}

vsg::ref_ptr<Renderable> DynamicTexture::make(Info& info) {
    vsg::ref_ptr<DynamicTexture> sphere_to_add = DynamicTexture::create(info);
    vsg::ref_ptr<Renderable> val = sphere_to_add.cast<Renderable>();
    return val;
}

void DynamicTexture::update_texture(updater&& update) {
    struct execute_update : public vsg::Visitor {
        updater update;
        vsg::ref_ptr<vsg::Data> data;

        execute_update() {

        }

        void set_execution_data(updater&& in_update, vsg::ref_ptr<vsg::Data> in_data) {
            update = std::move(in_update);
            data = in_data;
        }

        // use the vsg::Visitor to safely cast to types handled by the UpdateImage class
        void apply(vsg::vec4Array2D& image) override {
            update(image);
            image.dirty();
        }

        void operator()()
        {
            data->accept(*this);
        }

    };

    struct async_execute : public vsg::Inherit<vsg::Operation, async_execute> {
        execute_update update_operation;

        async_execute() {

        }

        void run() override {
            update_operation();
        }
    };

    vsg::ref_ptr<async_execute> async_updater = async_execute::create();
    async_updater->update_operation.set_execution_data(std::move(update), textureData);
    owner_viewer->addUpdateOperation(async_updater);
}

}
}