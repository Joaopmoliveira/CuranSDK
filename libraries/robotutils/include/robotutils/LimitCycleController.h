#ifndef CURAN_LIMIT_CYCLE_CONTROLLER_AUDE_
#define CURAN_LIMIT_CYCLE_CONTROLLER_AUDE_

#include "LBRController.h"
#include "gaussianmixtures/GMR.h"

namespace curan
{
    namespace robotic
    {

        struct Transformation
        {
            Eigen::Matrix<double, 3, 3> f_rotation;
            Eigen::Matrix<double, 3, 1> f_translation;

            Transformation(Eigen::Matrix<double, 3, 3> rotation, Eigen::Matrix<double, 3, 1> translation) : f_rotation{rotation}, f_translation{translation}
            {
            }

            Transformation(const Transformation &other) : f_rotation{other.f_rotation}, f_translation{other.f_translation}
            {
            }

            Transformation()
            {
                f_rotation = Eigen::Matrix<double, 3, 3>::Identity();
                f_translation = Eigen::Matrix<double, 3, 1>::Zero();
            }

            inline auto desired_rotation() const
            {
                return f_rotation;
            }

            inline auto desired_translation() const
            {
                return f_translation;
            }
        };

        struct LimitCycleController : public UserData
        {
            LimitCycleController(const std::string &model_file, const std::string &transform_file);

            EigenState &&update(const RobotModel<number_of_joints> &iiwa, EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians) override;

            curan::gaussian::GMR<2, 2> model;
            Transformation transformation_to_model_coordinates;
        };

    }
}

#endif