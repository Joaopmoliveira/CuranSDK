#include "rendering/SequencialLinks.h"

namespace curan {
namespace renderable {

SequencialLinks::SequencialLinks(std::filesystem::path json_path,size_t number_of_links) : number_of_links{number_of_links} {
    std::filesystem::path models_dir = json_path.parent_path();
    nlohmann::json tableDH = nlohmann::json::parse(std::ifstream(json_path));

    this->transform = vsg::MatrixTransform::create(vsg::translate(0.0,0.0,0.0));
    this->obj_contained = vsg::Group::create();

    vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->shaderSets["pbr"] = vsg::createPhysicsBasedRenderingShaderSet(options);

    size_t transform_index = 0;
    assert(tableDH.size()>=number_of_links && "The number of links is larger then the number of links configured in the json file");
    
    this->sequencial_base_transform = vsg::MatrixTransform::create(vsg::translate(0.0,0.0,0.0));
    vsg::ref_ptr<vsg::MatrixTransform> previous_object = this->sequencial_base_transform;
    obj_contained->addChild(this->sequencial_base_transform);

    for (auto& denavit_parameter : tableDH) {
        int is_dynamic = denavit_parameter["dynamic"];
        std::string relative_path = denavit_parameter["path"];
        double alpha = denavit_parameter["alpha"];
        double x_offset = denavit_parameter["x_offset"];
        double phi = denavit_parameter["phi"];
        double d_offset = denavit_parameter["d_offset"];

        std::filesystem::path local_temp_path = models_dir;
        local_temp_path += "/" + relative_path;
        std::cout << local_temp_path << std::endl;
        vsg::ref_ptr<vsg::Node> link_mesh = vsg::read_cast<vsg::Node>(local_temp_path.string() , options);
        
        vsg::ref_ptr<vsg::MatrixTransform> denavit_offset = vsg::MatrixTransform::create(vsg::translate(0.0,0.0,0.0));
        denavit_offset->matrix = vsg::translate(vsg::dvec3(0.0,0.0,x_offset)) * vsg::rotate(vsg::radians(alpha), 0.0, 0.0, 1.0) * vsg::translate(vsg::dvec3(d_offset,0.0,0.0)) * vsg::rotate(vsg::radians(phi), 1.0, 0.0, 0.0);
        std::cout << denavit_offset->matrix << std::endl;
        if(!link_mesh)
            throw std::runtime_error("failed to load one of the links");
        denavit_offset->addChild(link_mesh);
        previous_object->addChild(denavit_offset);
        previous_object = denavit_offset;

        if(is_dynamic) {
            assert(transform_index<number_of_links && "the number of dynamic links is larger than what was compiled");
            vsg::ref_ptr<vsg::MatrixTransform> dynamic_angle = vsg::MatrixTransform::create();
            previous_object->addChild(dynamic_angle);
            previous_object = dynamic_angle;
            links_matrix_transform[transform_index] = dynamic_angle;
            ++transform_index; 
        }
    }
    if(transform_index!=number_of_links){
        std::cout << "The number of links is larger then the number of links configured in the json file\n";
    } 
}

vsg::ref_ptr<Renderable> SequencialLinks::make(std::filesystem::path json_path,size_t number_of_links) {
    vsg::ref_ptr<SequencialLinks> arm = SequencialLinks::create(json_path,number_of_links);
    vsg::ref_ptr<Renderable> val = arm.cast<Renderable>();
    return val;
}

void SequencialLinks::set(const std::vector<double>& qs) {
    for (auto& local_matrix : links_matrix_transform) {
        size_t currentIdx = &local_matrix - &links_matrix_transform[0];
        q[currentIdx] = qs[currentIdx];
        local_matrix->matrix = vsg::rotate(q[currentIdx], 0.0, 1.0, 0.0);
    }
}

void SequencialLinks::append(vsg::ref_ptr<Renderable> link_to_join, vsg::ref_ptr<vsg::MatrixTransform> relative_transformation) {
    relative_transformation->addChild(link_to_join->transform);
    links_matrix_transform.back()->addChild(relative_transformation);
}

}
}