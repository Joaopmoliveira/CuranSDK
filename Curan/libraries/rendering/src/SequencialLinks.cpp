#include "rendering/SequencialLinks.h"

namespace curan {
namespace renderable {

SequencialLinks::SequencialLinks(const Info& create_info) : number_of_links{create_info.number_of_links} {
    auto json_path = create_info.json_path;
    std::filesystem::path models_dir = json_path.parent_path();
    nlohmann::json tableDH = nlohmann::json::parse(std::ifstream(json_path));

    transform = vsg::MatrixTransform::create(vsg::translate(0.0,0.0,0.0));
    obj_contained = vsg::Group::create();

    vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->formatCoordinateConventions[".obj"] = create_info.convetion;
    options->sceneCoordinateConvention = create_info.convetion;
    assert(tableDH.size() == number_of_links && "The number of links is from the number of links configured in the json file");
    assert(number_of_links>0 && "The number of links must be larger than 0");
    
    vsg::ref_ptr<vsg::MatrixTransform> previousFramePose; 

    links_matrix_transform.resize(number_of_links);
    link_angles.resize(number_of_links);

    auto iterator = tableDH.begin();
    
    for (auto& rotational_matrix : links_matrix_transform) {
        std::string relative_path = (*iterator)["path"];
        double d_offset = (*iterator)["d_offset"];
        double theta = (*iterator)["theta"];
        double a_offset = (*iterator)["a_offset"];
        double alpha = (*iterator)["alpha"];

        std::filesystem::path local_temp_path = models_dir;
        local_temp_path += "/" + relative_path;

        std::cout << "loading... " << local_temp_path << std::endl;
        vsg::ref_ptr<vsg::Node> link_mesh = vsg::read_cast<vsg::Node>(local_temp_path.string() , options);
        if(!link_mesh)
            throw std::runtime_error("failed to load one of the links");
        std::cout << "loaded " << std::endl;
        rotational_matrix = vsg::MatrixTransform::create();

        if(iterator == tableDH.begin()){
            auto denavit_transformation = vsg::MatrixTransform::create();
            denavit_transformation->matrix = denavit_transformation->transform(vsg::rotate(vsg::radians(alpha), 1.0, 0.0, 0.0));
            denavit_transformation->matrix = denavit_transformation->transform(vsg::translate(a_offset,0.0,0.0)); 
            denavit_transformation->matrix = denavit_transformation->transform(vsg::rotate(vsg::radians(theta), 0.0, 0.0, 1.0));
            denavit_transformation->matrix = denavit_transformation->transform(vsg::translate(0.0,0.0,d_offset));
            denavit_transformation->addChild(rotational_matrix);
            obj_contained->addChild(link_mesh);
            obj_contained->addChild(denavit_transformation);
            previousFramePose = rotational_matrix;
        } else {
            auto denavit_transformation = vsg::MatrixTransform::create();
            denavit_transformation->matrix = denavit_transformation->transform(vsg::rotate(vsg::radians(alpha), 1.0, 0.0, 0.0));
            denavit_transformation->matrix = denavit_transformation->transform(vsg::translate(a_offset,0.0,0.0)); 
            denavit_transformation->matrix = denavit_transformation->transform(vsg::rotate(vsg::radians(theta), 0.0, 0.0, 1.0));
            denavit_transformation->matrix = denavit_transformation->transform(vsg::translate(0.0,0.0,d_offset));
            denavit_transformation->addChild(rotational_matrix);
            previousFramePose->addChild(link_mesh);
            previousFramePose->addChild(denavit_transformation);
            previousFramePose = rotational_matrix;
        }
        ++iterator;
    }
}

vsg::ref_ptr<Renderable> SequencialLinks::make(const Info& create_info) {
    vsg::ref_ptr<SequencialLinks> arm = SequencialLinks::create(create_info);
    vsg::ref_ptr<Renderable> val = arm.cast<Renderable>();
    return val;
}

void SequencialLinks::set(const std::vector<double>& new_angles) {
    assert(new_angles.size()==links_matrix_transform.size() && "The supplied angles are not the same size as the number of links");
    auto iterator_angles = link_angles.begin();
    auto iterator_matrices = links_matrix_transform.begin();
    for (const auto& link_angle : new_angles) {
        *iterator_angles = link_angle;
        (*iterator_matrices)->matrix = vsg::rotate(link_angle, 0.0, 0.0, 1.0);
        ++iterator_matrices;
        ++iterator_angles;
    }
}

void SequencialLinks::append(vsg::ref_ptr<Renderable> link_to_join) {
    assert(links_matrix_transform.size() > 0 && "The size of the links is not larger than zero as required");
    links_matrix_transform.back()->addChild(link_to_join->transform);
}

}
}