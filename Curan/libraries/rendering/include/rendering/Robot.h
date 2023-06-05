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

namespace curan {
	namespace renderable {

        template <size_t NumberOfLinks,size_t MovableOffset>
        struct RobotArm : public vsg::Inherit<Renderable, RobotArm<NumberOfLinks,MovableOffset>> {

            // The joints' angles for the robot
            std::array<double, NumberOfLinks> q;
            // Array of transformation matrices between each joint
            std::array<vsg::ref_ptr<vsg::MatrixTransform>, NumberOfLinks> A;
            
            RobotArm(std::filesystem::path jsonPath) {
                std::filesystem::path modelsDir = jsonPath.parent_path();
                nlohmann::json tableDH = nlohmann::json::parse(std::ifstream(jsonPath));

                transform = vsg::MatrixTransform::create(vsg::translate(vsg::dvec3(0.0, 0.0, 0.0)));
                obj_contained = vsg::Group::create();

                vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
                options->add(vsgXchange::all::create());

                size_t iter_index = 0;
                for(const auto& link : tableDH){
                    link["theta"];
                    link["path"];
                    link["offset"];
                    int is_dinamic = link["dynamic"];
                    ++iter_index;
                }

                vsg::ref_ptr<vsg::MatrixTransform> linkPosition = vsg::MatrixTransform::create();
                linkPosition->matrix = vsg::rotate((double)tableDH[0]["theta"], 1.0, 0.0, 0.0);

                std::string local = tableDH[0]["path"];
                std::filesystem::path first_node = modelsDir;
                first_node /= local;

                vsg::ref_ptr<vsg::Node> link = vsg::read_cast<vsg::Node>(first_node.c_str(), options);
                if (!link)
                    throw std::runtime_error("Couldn't load node: " + first_node.string());
                linkPosition->addChild(link);
                obj_contained->addChild(linkPosition);

                assert(tableDH.size()==NumberOfLinks && "The number of parameters in the json file does not correspond to the number of joints compiled in the code");
                set_identifier("my_robot_friend");
            }

            static vsg::ref_ptr<Renderable> make(std::filesystem::path jsonPath) {
                vsg::ref_ptr<RobotArm> arm_to_add = RobotArm<NumberOfLinks,MovableOffset>::create(jsonPath);
                vsg::ref_ptr<Renderable> val = arm_to_add.template cast<Renderable>();
                return val;
            }

            template <size_t n> inline double get() const {
                static_assert(n < NumberOfLinks);
                return q[n];
            }

            template <size_t n> inline void set(const double& new_q) {
                static_assert(n < NumberOfLinks);
                q[n] = new_q;
                A[n]->matrix = vsg::rotate(new_q, 0.0, 1.0, 0.0);
            }

            void set(const std::array<double, NumberOfLinks-MovableOffset>& qs) {
                static_assert(qd.size()==NumberOfLinks-MovableOffset && "The number of movable links is not the same and the supplied size")
                for (auto& a : A) {
                    size_t currentIdx = &a - &A[0];
                    q[currentIdx] = qs[currentIdx];
                    a->matrix = vsg::rotate(q[currentIdx], 0.0, 1.0, 0.0);
                }
            }
        };

	}
}

#endif