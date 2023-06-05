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


namespace curan {
	namespace renderable {

        void replaceAll(
            std::string& mainString,
            std::string const findString,
            std::string const replaceString
        ) {
            std::size_t positionFinded = mainString.find(findString);
            if (positionFinded == std::string::npos) return;
            mainString.replace(positionFinded, findString.length(), replaceString);
        }

        template <size_t N>
        struct RobotArm : public vsg::Inherit<Renderable, RobotArm<N>> {
            // The joints' angles for the robot
            std::array<double, N> q;
            // Array of transformation matrices between each joint
            std::array<vsg::ref_ptr<vsg::MatrixTransform>, N> A;
            
            RobotArm(std::string jsonPath) {
                replaceAll(jsonPath, "\\", "/");
                std::string modelsDir = jsonPath.substr(0, jsonPath.find_last_of("/"));
                nlohmann::json tableDH = nlohmann::json::parse(std::ifstream(jsonPath));

                this->transform = vsg::MatrixTransform::create(
                    vsg::translate(vsg::dvec3(0.0, 0.0, 0.0)));
                this->obj_contained = vsg::Group::create();

                vsg::ref_ptr<vsg::Options> options = vsg::Options::create();
                options->add(vsgXchange::all::create());

                vsg::ref_ptr<vsg::MatrixTransform> linkPosition =
                    vsg::MatrixTransform::create();
                linkPosition->matrix =
                    vsg::rotate((double)tableDH[0]["theta"], 1.0, 0.0, 0.0);
                vsg::ref_ptr<vsg::Node> link = vsg::read_cast<vsg::Node>(
                    modelsDir + "/" + ((std::string)tableDH[0]["path"]), options);
                if (!link)
                    throw std::runtime_error("Couldn't load node: " + modelsDir + "/" +
                        ((std::string)tableDH[0]["path"]));
                linkPosition->addChild(link);
                this->obj_contained->addChild(linkPosition);

                for (auto& a : A) {
                    a = vsg::MatrixTransform::create();
                    a->matrix = vsg::rotate(vsg::radians(0.0), 0.0, 1.0, 0.0);
                    linkPosition->addChild(a);
                    linkPosition = vsg::MatrixTransform::create();
                    linkPosition->matrix = vsg::rotate(
                        (double)tableDH[&a - &A[0] + 1]["theta"], 1.0, 0.0, 0.0);
                    linkPosition->matrix = linkPosition->transform(vsg::translate(
                        0.0, -((double)tableDH[&a - &A[0] + 1]["offset"]), 0.0));
                    a->addChild(linkPosition);
                    link = vsg::read_cast<vsg::Node>(
                        modelsDir + "/" + (std::string)tableDH[&a - &A[0] + 1]["path"],
                        options);
                    if (!link)
                        std::runtime_error(
                            "Couldn't load node: " + modelsDir +
                            (std::string)tableDH[&a - &A[0] + 1]["path"]);
                    linkPosition->addChild(link);
                }
            }

            static vsg::ref_ptr<Renderable> make(std::string jsonPath) {
                vsg::ref_ptr<RobotArm> arm_to_add = RobotArm<N>::create(jsonPath);
                vsg::ref_ptr<Renderable> val = arm_to_add.template cast<Renderable>();
                return val;
            }

            template <size_t n> inline double get() const {
                static_assert(n < N);
                return q[n];
            }

            template <size_t n> inline void set(const double& new_q) {
                static_assert(n < N);
                q[n] = new_q;
                A[n]->matrix = vsg::rotate(new_q, 0.0, 1.0, 0.0);
            }

            void set_bulk(const std::array<double, N>& qs) {
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