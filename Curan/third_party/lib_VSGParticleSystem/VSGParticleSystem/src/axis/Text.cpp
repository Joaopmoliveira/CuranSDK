#include <vsgParticleSystem/axis/Text.h>

namespace vsgps {
Text::Text(std::string const &value, vsg::dvec3 const &pos,
           vsg::ref_ptr<vsg::Options> const &options, double fontSize)
    : options{options} {

    layout = vsg::StandardLayout::create();
    layout->horizontalAlignment = vsg::StandardLayout::CENTER_ALIGNMENT;
    layout->position = vsg::vec3(pos.x, pos.y, pos.z);
    layout->horizontal = vsg::vec3(fontSize, 0.0, 0.0);
    layout->vertical = vsg::vec3(0.0, fontSize, 0.0);
    layout->color = vsg::vec4(1.0, 1.0, 1.0, 1.0);
    layout->billboard = true;

    text = vsg::Text::create();
    text->technique = vsg::GpuLayoutTechnique::create();
    label = vsg::stringValue::create(value);
    text->text = label;
    text->font =
        vsg::read_cast<vsg::Font>(std::string("fonts/times.vsgb"), options);
    text->layout = layout;
    text->setup(20, options);
}

void Text::changeText(std::string const &newValue) {
    label->value() = newValue;
    text->setup(0, options);
}

void Text::moveText(vsg::dvec3 const &newPos) {
    layout->position = vsg::vec3(newPos.x, newPos.y, newPos.z);
    text->setup(0, options);
}

void Text::addToScene(vsg::ref_ptr<vsg::Group> const &scene) {
    scene->addChild(text);
}

void Text::addToScene(vsg::ref_ptr<vsg::Group> &scene, size_t idx) {
    scene->children[idx] = text;
}
}