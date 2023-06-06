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
struct RobotArm : public vsg::Inherit<Renderable, RobotArm<NumberOfLinks>> {

std::array<double, NumberOfLinks> q;
std::array<vsg::ref_ptr<vsg::MatrixTransform>, NumberOfLinks> matrix_transform;
            
RobotArm(std::filesystem::path json_path) {
    std::filesystem::path models_dir = json_path.parent_path();
    nlohmann::json tableDH = nlohmann::json::parse(std::ifstream(json_path));

    transform = vsg::MatrixTransform::create(vsg::translate(vsg::dvec3(0.0, 0.0, 0.0)));
    obj_contained = vsg::Group::create();

    vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
    options->add(vsgXchange::all::create());

    vsg::ref_ptr<vsg::MatrixTransform> linkPosition = vsg::MatrixTransform::create();
    linkPosition->matrix =vsg::rotate((double)tableDH[0]["theta"], 1.0, 0.0, 0.0);
    std::string relative_path = tableDH[0]["path"];
    std::filesystem::path local_temp_path = models_dir;
    local_temp_path += "/" + relative_path;
    vsg::ref_ptr<vsg::Node> link = vsg::read_cast<vsg::Node>(local_temp_path.string() , options);

    if (!link)
        throw std::runtime_error("Couldn't load node: " + local_temp_path.string());

    if(tableDH.size()!=NumberOfLinks+1)
        throw std::runtime_error("The number of links in the json file is incorrect");

    linkPosition->addChild(link);
    obj_contained->addChild(linkPosition);

    for (auto& local_matrix : matrix_transform) {
        local_matrix = vsg::MatrixTransform::create();
        local_matrix->matrix = vsg::rotate(vsg::radians(0.0), 0.0, 1.0, 0.0);
        linkPosition->addChild(local_matrix);
        linkPosition = vsg::MatrixTransform::create();
        linkPosition->matrix = vsg::rotate((double)tableDH[&local_matrix - &matrix_transform[0] + 1]["theta"], 1.0, 0.0, 0.0);
        linkPosition->matrix = linkPosition->transform(vsg::translate( 0.0, -((double)tableDH[&local_matrix - &matrix_transform[0] + 1]["offset"]), 0.0));
        local_matrix->addChild(linkPosition);
        std::string local_relative_path = tableDH[&local_matrix - &matrix_transform[0] + 1]["path"];
        std::filesystem::path local_temp_path_inner = models_dir;
        local_temp_path_inner += "/" + local_relative_path;
        link = vsg::read_cast<vsg::Node>(local_temp_path_inner.string(),options);
        std::cout << local_temp_path_inner << std::endl;
        if (!link)
            throw std::runtime_error("Couldn't load node: " + local_temp_path_inner.string());
        linkPosition->addChild(link);
    }
}

static vsg::ref_ptr<Renderable> make(std::filesystem::path json_path) {
    vsg::ref_ptr<RobotArm> arm = RobotArm<NumberOfLinks>::create(json_path);
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

};

}
}

#endif