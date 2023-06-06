#ifndef CURAN_ROBOT_RENDERABLE_HEADER_FILE_
#define CURAN_ROBOT_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include <array>
#include <vsg/core/Inherit.h>
#include <vsg/core/ref_ptr.h>
#include <vsg/io/read.h>
#include <vsg/maths/transform.h>
#include <vsg/maths/vec3.h>
#include <vsg/nodes/MatrixTransform.h>
#include <vsg/nodes/Node.h>
#include <filesystem>
#include <iostream>

namespace curan {
namespace renderable {

template <size_t NumberOfLinks>
struct SequencialLinks : public vsg::Inherit<Renderable, SequencialLinks<NumberOfLinks>> {

std::array<double, NumberOfLinks> q;
std::array<vsg::ref_ptr<vsg::MatrixTransform>, NumberOfLinks> matrix_transform;
            
SequencialLinks(std::filesystem::path json_path) {
    std::filesystem::path models_dir = json_path.parent_path();
    nlohmann::json tableDH = nlohmann::json::parse(std::ifstream(json_path));

    transform = vsg::MatrixTransform::create(vsg::translate(vsg::dvec3(0.0, 0.0, 0.0)));
    obj_contained = vsg::Group::create();

    auto local_first_transform = vsg::MatrixTransform::create(vsg::translate(vsg::dvec3(0.0, 0.0, 0.0)));
    obj_contained->addChild(local_first_transform);

    vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
    options->add(vsgXchange::all::create());

    size_t transform_index = 0;
    assert(tableDH.size()>=NumberOfLinks && "The number of links is larger then the number of links configured in the json file");
    
    vsg::ref_ptr<vsg::MatrixTransform> previous_transformation = local_first_transform;

    for (auto& denavit_parameter : tableDH) {
        int is_dynamic = denavit_parameter["dynamic"];
        std::string relative_path = denavit_parameter["path"];
        float alpha = denavit_parameter["alpha"];
        float x_offset = denavit_parameter["x_offset"];
        float phi = denavit_parameter["phi"];
        float d_offset = denavit_parameter["d_offset"];

        std::filesystem::path local_temp_path = models_dir;
        local_temp_path += "/" + relative_path;
        std::cout << local_temp_path << std::endl;
        vsg::ref_ptr<vsg::Node> link_mesh = vsg::read_cast<vsg::Node>(local_temp_path.string() , options);
        vsg::ref_ptr<vsg::MatrixTransform> denavit_offset = vsg::MatrixTransform::create();
        denavit_offset->matrix = vsg::rotate(vsg::radians(alpha), 1.0f, 0.0f, 0.0f)*vsg::translate(vsg::vec3(x_offset,0.0,0.0)) * vsg::rotate(vsg::radians(phi), 0.0f, 0.0f, 1.0f)*vsg::translate(vsg::vec3(d_offset,0.0,0.0));
        
        
        denavit_offset->addChild(link_mesh);
        previous_transformation->addChild(denavit_offset);
        previous_transformation = denavit_offset;

        if(is_dynamic) {
            assert(transform_index<NumberOfLinks && "the number of dynamic links is larger than what was compiled");
            vsg::ref_ptr<vsg::MatrixTransform> dynamic_angle = vsg::MatrixTransform::create();
            dynamic_angle->matrix = vsg::rotate(vsg::radians(0.0), 0.0, 0.0, 1.0);
            previous_transformation->addChild(dynamic_angle);
            previous_transformation = dynamic_angle;
            matrix_transform[transform_index] = dynamic_angle;
            ++transform_index; 
        }
    }
    assert(transform_index==NumberOfLinks && "The number of links is larger then the number of links configured in the json file");
}

static vsg::ref_ptr<Renderable> make(std::filesystem::path json_path) {
    vsg::ref_ptr<SequencialLinks<NumberOfLinks>> arm = SequencialLinks<NumberOfLinks>::create(json_path);
    vsg::ref_ptr<Renderable> val = arm.cast<Renderable>();
    return val;
}

template <size_t n> inline double get() const {
    static_assert(n < NumberOfLinks);
    return q[n];
}

template <size_t n> inline void set(const double& new_q) {
    static_assert(n < NumberOfLinks);
    q[n] = new_q;
    matrix_transform[n]->matrix = vsg::rotate(new_q, 0.0, 1.0, 0.0);
}

void set_bulk(const std::array<double, NumberOfLinks>& qs) {
    for (auto& local_matrix : matrix_transform) {
        size_t currentIdx = &local_matrix - &matrix_transform[0];
        q[currentIdx] = qs[currentIdx];
        local_matrix->matrix = vsg::rotate(q[currentIdx], 0.0, 1.0, 0.0);
    }
}

void append(vsg::ref_ptr<Renderable> link_to_join, vsg::ref_ptr<vsg::MatrixTransform> relative_transformation) override {
    relative_transformation->addChild(link_to_join->transform);
    matrix_transform.back()->addChild(relative_transformation);
}

};

}
}

#endif