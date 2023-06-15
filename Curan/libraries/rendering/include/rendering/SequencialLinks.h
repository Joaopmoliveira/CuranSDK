#ifndef CURAN_ROBOT_RENDERABLE_HEADER_FILE_
#define CURAN_ROBOT_RENDERABLE_HEADER_FILE_

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include "Renderable.h"
#include <optional>
#include <string>
#include <nlohmann/json.hpp>
#include <vector>
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

struct SequencialLinks : public vsg::Inherit<Renderable, SequencialLinks> {

    struct Info{
        std::filesystem::path json_path;
        size_t number_of_links;
        vsg::CoordinateConvention convetion = vsg::CoordinateConvention::Z_UP;
    };

    std::vector<double> link_angles;
    std::vector<vsg::ref_ptr<vsg::MatrixTransform>> links_matrix_transform;
    size_t number_of_links = 0;
    
    SequencialLinks(const Info& create_info);

    static vsg::ref_ptr<Renderable> make(const Info& create_info);

    inline double get(const size_t& index) const{
        return link_angles.at(index);
    }

    inline void set(size_t index, const double& new_q) {
        assert(index < number_of_links);
        link_angles[index] = new_q;
        links_matrix_transform[index]->matrix = vsg::rotate(new_q, 0.0, 0.0, 1.0);
    }

    void set(const std::vector<double>& qs);

    void append(vsg::ref_ptr<Renderable> link_to_join, vsg::ref_ptr<vsg::MatrixTransform> relative_transformation) override;

};

}
}

#endif