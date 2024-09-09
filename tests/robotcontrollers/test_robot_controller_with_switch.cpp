#include "robotutils/LBRController.h"
#include "robotutils/SimulateModel.h"

#include "utils/Logger.h"
#include "utils/TheadPool.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <csignal>
#include <Eigen/Geometry>

#include <functional>

void custom_interface(vsg::CommandBuffer &cb, std::atomic<curan::robotic::State> &atomic_access)
{
    auto state = atomic_access.load(std::memory_order_relaxed);
    {
        ImGui::Begin("Cart pos"); // Create a window called "Hello, world!" and append into it.
        static std::array<curan::renderable::ScrollingBuffer, 6> error_buffers;
        static float t = 0;
        t += ImGui::GetIO().DeltaTime;

        static float history = 10.0f;
        ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

        static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            for (size_t index = 0; index < 6; ++index)
            {
                std::string loc = "error" + std::to_string(index);
                error_buffers[index].AddPoint(t, (float)state.user_defined[index]);
                ImPlot::PlotLine(loc.data(), &error_buffers[index].Data[0].x, &error_buffers[index].Data[0].y, error_buffers[index].Data.size(), 0, error_buffers[index].Offset, 2 * sizeof(float));
            }
            ImPlot::EndPlot();
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }

    {
        ImGui::Begin("Cart interpol"); // Create a window called "Hello, world!" and append into it.
        static std::array<curan::renderable::ScrollingBuffer, 6> error_buffers;
        static float t = 0;
        t += ImGui::GetIO().DeltaTime;

        static float history = 10.0f;
        ImGui::SliderFloat("History", &history, 1, 30, "%.1f s");

        static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

        if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(NULL, NULL, flags, flags);
            ImPlot::SetupAxisLimits(ImAxis_X1, t - history, t, ImGuiCond_Always);
            ImPlot::SetupAxisLimits(ImAxis_Y1, -30, 30);
            ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
            for (size_t index = 0; index < 6; ++index)
            {
                std::string loc = "error" + std::to_string(index);
                error_buffers[index].AddPoint(t, (float)state.user_defined2[index]);
                ImPlot::PlotLine(loc.data(), &error_buffers[index].Data[0].x, &error_buffers[index].Data[0].y, error_buffers[index].Data.size(), 0, error_buffers[index].Offset, 2 * sizeof(float));
            }
            ImPlot::EndPlot();
        }
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();
    }
}

curan::robotic::RobotLBR *robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;
std::atomic<bool> keep_running = true;
void signal_handler(int signal)
{
    if (robot_pointer)
        robot_pointer->cancel();
    keep_running = false;
}

struct InterpolatedPose
{
private:
    Eigen::Matrix<double, 4, 4> target_pose;
    Eigen::Matrix<double, 4, 4> initial_pose;
    Eigen::Matrix<double, 4, 4> interpolated_pose;
    double t;
    double interpolation_velocity;
    std::function<Eigen::Matrix<double, 4, 4>()> executed_once_interpolation_finished;

public:
    InterpolatedPose(Eigen::Matrix<double, 4, 4> in_initial_pose,
                     Eigen::Matrix<double, 4, 4> in_target_pose,
                     double in_interpolation_velocity) : initial_pose{in_initial_pose},
                                                         target_pose{in_target_pose},
                                                         interpolated_pose{in_initial_pose},
                                                         t{0},
                                                         interpolation_velocity{in_interpolation_velocity}
    {
    }

    InterpolatedPose(Eigen::Matrix<double, 4, 4> in_initial_pose,
                     Eigen::Matrix<double, 4, 4> in_target_pose,
                     double in_interpolation_velocity,
                     std::function<Eigen::Matrix<double, 4, 4>()> in_executed_once_interpolation_finished) : initial_pose{in_initial_pose},
                                                                                                             target_pose{in_target_pose},
                                                                                                             interpolated_pose{in_initial_pose},
                                                                                                             t{0},
                                                                                                             interpolation_velocity{in_interpolation_velocity},
                                                                                                             executed_once_interpolation_finished{in_executed_once_interpolation_finished}
    {
    }

    InterpolatedPose(double in_interpolation_velocity) : t{0},
                                                         interpolation_velocity{in_interpolation_velocity}
    {
        interpolated_pose = Eigen::Matrix<double, 4, 4>::Identity();
        target_pose = Eigen::Matrix<double, 4, 4>::Identity();
        initial_pose = Eigen::Matrix<double, 4, 4>::Identity();
    }

    bool advance()
    {
        t += interpolation_velocity;
        if (t > 1.0)
        {
            interpolated_pose = (executed_once_interpolation_finished) ? executed_once_interpolation_finished() : target_pose;
            t = 1.0;
            return false;
        }

        double q = 3.0 * t * t - 2.0 * t * t * t;
        interpolated_pose.block<3, 1>(0, 3) = q * target_pose.block<3, 1>(0, 3) + (1 - q) * initial_pose.block<3, 1>(0, 3);
        Eigen::Quaternion<double> quat1{initial_pose.block<3, 3>(0, 0)};
        Eigen::Quaternion<double> quat2{target_pose.block<3, 3>(0, 0)};
        Eigen::Quaternion<double> qres = quat1.slerp(q, quat2);
        if (qres.w() >= -1e-6)
            qres = Eigen::Quaternion<double>{-qres.w(), -qres.x(), -qres.y(), -qres.z()};
        interpolated_pose.block<3, 3>(0, 0) = qres.toRotationMatrix();
        return true;
    };

    inline double step_size()
    {
        return interpolation_velocity;
    }

    inline const Eigen::Matrix<double, 4, 4> &interpolated() const
    {
        return interpolated_pose;
    };
};

struct ControllerSwitcher : public curan::robotic::UserData
{
    enum ControlModes
    {
        POSITION_HOLD,
        NEEDLE_POSITIONING,
        FREE_HAND
    };

    ControlModes current_mode = ControlModes::FREE_HAND;
    std::atomic<bool> transitioning = false;

    std::mutex mut;
    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> diagonal_damping;

    InterpolatedPose position_hold_interpolator;

    ControllerSwitcher(std::initializer_list<double> in_stiffness_diagonal_gains,
                       std::initializer_list<double> in_diagonal_damping,
                       double in_interpolation_velocity) : position_hold_interpolator{in_interpolation_velocity}
    {
        if (in_stiffness_diagonal_gains.size() != 6)
            throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
        if (in_diagonal_damping.size() != 6)
            throw std::runtime_error("the dimensions of the diagonal matrix must be 6x6");
        stiffness = Eigen::Matrix<double, 6, 6>::Identity();
        diagonal_damping = Eigen::Matrix<double, 6, 6>::Identity();
        auto stiffness_entry_value = in_stiffness_diagonal_gains.begin();
        auto damping_entry_value = in_diagonal_damping.begin();
        for (size_t entry = 0; entry < 6; ++entry, ++stiffness_entry_value, ++damping_entry_value)
        {
            if (*damping_entry_value < 0.0 || *damping_entry_value > 1.0)
                throw std::runtime_error("the damping matrix must be positive definite (in [0 1]) : (" + std::to_string(entry) + ") -> (" + std::to_string(*damping_entry_value) + ")");
            if (*stiffness_entry_value < 0.0)
                throw std::runtime_error("the stiffness matrix must be positive definite : (" + std::to_string(entry) + ") -> (" + std::to_string(*stiffness_entry_value) + ")");
            diagonal_damping(entry, entry) = *damping_entry_value;
            stiffness(entry, entry) = *stiffness_entry_value;
        }
    }

    bool free_hand_control(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
    {
        state.cmd_tau = -iiwa.mass() * iiwa.velocities();
        return false;
    }

    bool position_hold_control(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
    {
        using namespace curan::robotic;
        state.cmd_tau = -iiwa.mass() * iiwa.velocities();
        bool is_transitioning = false;
        Eigen::Matrix<double, 4, 4> interpolated_pose;
        {
            std::lock_guard<std::mutex> g{mut};
            is_transitioning = position_hold_interpolator.advance();
            interpolated_pose = position_hold_interpolator.interpolated();
        }
        Eigen::Matrix<double, 6, 6> Lambda;
        {
            Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{iiwa.jacobian() * iiwa.invmass() * iiwa.jacobian().transpose(), Eigen::ComputeFullU};
            Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
            auto singular_values = svdofsitff.singularValues();
            for (auto &diag : singular_values)
                diag = (diag < 0.02) ? 0.02 : diag;
            Lambda = (P * singular_values.asDiagonal() * P.transpose()).inverse();
        }

        Eigen::Matrix<double, curan::robotic::number_of_joints, curan::robotic::number_of_joints> nullspace_projector = Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity() - iiwa.jacobian().transpose() * (iiwa.invmass() * iiwa.jacobian().transpose() * Lambda).transpose();
        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();

        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * interpolated_pose.block<3, 3>(0, 0));

        error.block<3, 1>(0, 0) = (interpolated_pose.block<3, 1>(0, 3) - iiwa.translation());
        static auto previous_error = E_AxisAngle.axis();
        previous_error = ((E_AxisAngle.axis() - previous_error).norm() > (-E_AxisAngle.axis() - previous_error).norm()) ? -E_AxisAngle.axis() : E_AxisAngle.axis();
        double magnitude_of_error = ((E_AxisAngle.axis() - previous_error).norm() > (-E_AxisAngle.axis() - previous_error).norm()) ? -E_AxisAngle.angle() : E_AxisAngle.angle();
        error.block<3, 1>(3, 0) = magnitude_of_error * iiwa.rotation() * previous_error;

        {
            Eigen::AngleAxisd interpolated_ori(interpolated_pose.block<3, 3>(0, 0));
            state.user_defined.block<3, 1>(3, 0) = error.block<3, 1>(3, 0);
            state.user_defined2.block<3, 1>(3, 0) = interpolated_ori.angle() * interpolated_ori.axis();

            state.user_defined.block<3, 1>(0, 0) = error.block<3, 1>(0, 0);
            state.user_defined2.block<3, 1>(0, 0) = interpolated_pose.block<3, 1>(0, 3);
        }
        Eigen::Matrix<double, number_of_joints, 1> error_in_nullspace = -iiwa.joints();
        /*
        We use the decomposition introduced in
        "Cartesian Impedance Control of Redundant and Flexible-Joint Robots" page 37
        but we use the nomenclature of the book in "Matrix Algebra From a Statistician's Perspective"
        */

        /*
        We have two controllers running simultaneously, the first controller, the so called cartersian impedance controller
        has a stiffness specified by the user, while the second impedance controller focuses on bringing all joints to their
        home configuration.
        */
        Eigen::Matrix<double, 6, 6> B0_cartesian;
        Eigen::Matrix<double, 6, 6> Q_cartesian;
        {
            Eigen::LLT<Eigen::Matrix<double, 6, 6>> lltOfLambda(Lambda);
            Eigen::Matrix<double, 6, 6> L = lltOfLambda.matrixL();
            Eigen::Matrix<double, 6, 6> R = L.inverse();
            Eigen::Matrix<double, 6, 6> C = R * stiffness * R.transpose();
            Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{C, Eigen::ComputeFullU};
            Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
            Q_cartesian = R.inverse() * P;
            Eigen::Matrix<double, 6, 6> Qinv = Q_cartesian.inverse();
            B0_cartesian = Qinv * stiffness * Qinv.transpose();
        }
        Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_stiffness = 3.0 * Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity();
        Eigen::Matrix<double, number_of_joints, number_of_joints> B0_nullspace;
        Eigen::Matrix<double, number_of_joints, number_of_joints> Q_nullspace;
        {
            Eigen::LLT<Eigen::Matrix<double, number_of_joints, number_of_joints>> lltOfLambda(iiwa.mass());
            Eigen::Matrix<double, number_of_joints, number_of_joints> L = lltOfLambda.matrixL();
            Eigen::Matrix<double, number_of_joints, number_of_joints> R = L.inverse();
            Eigen::Matrix<double, number_of_joints, number_of_joints> C = R * nullspace_stiffness * R.transpose();
            Eigen::JacobiSVD<Eigen::Matrix<double, number_of_joints, number_of_joints>> svdofsitff{C, Eigen::ComputeFullU};
            Eigen::Matrix<double, number_of_joints, number_of_joints> P = svdofsitff.matrixU();
            Q_nullspace = R.inverse() * P;
            Eigen::Matrix<double, number_of_joints, number_of_joints> Qinv = Q_nullspace.inverse();
            B0_nullspace = Qinv * nullspace_stiffness * Qinv.transpose();
        }

        Eigen::Matrix<double, 6, 6> damping_cartesian = 2 * Q_cartesian * diagonal_damping * B0_cartesian.diagonal().array().sqrt().matrix().asDiagonal() * Q_cartesian.transpose();
        // it is assumed that the nullspace has a diagonal damping ratio of 1.0.
        Eigen::Matrix<double, number_of_joints, number_of_joints> damping_nullspace = 2 * Q_nullspace * B0_nullspace.diagonal().array().sqrt().matrix().asDiagonal() * Q_nullspace.transpose();

        /*
        We need to filer out the velocity because in steady stady state they can become problematic
        */
        static Eigen::Matrix<double, number_of_joints, 1> filtered_velocity = iiwa.velocities();
        auto val = 0.8 * filtered_velocity + 0.2 * iiwa.velocities();
        filtered_velocity = val;

        static auto previous_jacobian = iiwa.jacobian();
        static Eigen::Matrix<double, 6, curan::robotic::number_of_joints> jacobian_derivative = (iiwa.jacobian() - previous_jacobian) / iiwa.sample_time();
        jacobian_derivative = (0.8 * jacobian_derivative + 0.2 * (iiwa.jacobian() - previous_jacobian) / iiwa.sample_time()).eval();
        previous_jacobian = iiwa.jacobian();
        auto conditioned_matrix = iiwa.jacobian().transpose() * iiwa.jacobian();
        Eigen::JacobiSVD<Eigen::Matrix<double, curan::robotic::number_of_joints, curan::robotic::number_of_joints>> svdjacobian{conditioned_matrix, Eigen::ComputeFullU};
        auto U = svdjacobian.matrixU();
        auto singular_values = svdjacobian.singularValues();
        for (auto &diag : singular_values)
            diag = (diag < 0.02) ? 0.02 : diag;

        auto properly_conditioned_matrix = (U * singular_values.asDiagonal() * U.transpose());
        auto inverse_jacobian = properly_conditioned_matrix.inverse() * iiwa.jacobian().transpose();
        // these values can becomes quite high with large accelerations
        // Eigen::Matrix<double, 6, 1> forces_caused_by_coordinate_change = inverse_jacobian.transpose() * iiwa.mass() * inverse_jacobian * jacobian_derivative * inverse_jacobian * iiwa.jacobian() * filtered_velocity;
        Eigen::Matrix<double, 6, 1> forces_caused_by_coordinate_change = Eigen::Matrix<double, 6, 1>::Zero();
        // normalize the error to an upper bound
        state.cmd_tau = iiwa.jacobian().transpose() * (stiffness * error - damping_cartesian * iiwa.jacobian() * filtered_velocity - forces_caused_by_coordinate_change) + nullspace_projector * (nullspace_stiffness * error_in_nullspace - damping_nullspace * filtered_velocity - 10.0 * iiwa.mass() * filtered_velocity);
        return is_transitioning;
        return false;
    }

    // [returns true if the barrier has been shut on and then off]
    bool barrier_function_for_needle_controller(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, Eigen::Matrix<double, 7, 1> &task_torque, Eigen::Matrix<double, 4, 4> target_pose)
    {
        Eigen::Matrix<double, 3, 1> plane_point = target_pose.block<3, 1>(0, 3);
        Eigen::Matrix<double, 3, 1> direction_along_valid_region = target_pose.block<3, 1>(0, 2);
        double Dmax = -direction_along_valid_region.transpose() * plane_point;
        double D = direction_along_valid_region.transpose() * iiwa.translation() + Dmax;
        Dmax = 0;

        constexpr double max_accel = 1;
        constexpr double max_vel = 1;

        Eigen::VectorXd vel = iiwa.jacobian() * iiwa.velocities();
        Eigen::Vector3d vel_pos = vel.block(0, 0, 3, 1);
        double dotD = (direction_along_valid_region.transpose() * vel_pos)(0, 0);

        constexpr double scalling_factor = 15.0;
        double dtvar = scalling_factor * iiwa.sample_time();
        if (dtvar < 0.001)
            dtvar = scalling_factor * 0.001;
        double dt2 = dtvar;
        double wallTopD = Dmax - D;

        double lowestdtfactor = 10;

        if (wallTopD < 0.1)
        {
            if (wallTopD < 0.0)
                wallTopD = 0.0;
            dt2 = (lowestdtfactor + sqrt(lowestdtfactor * wallTopD)) * dtvar;
            if (dt2 < lowestdtfactor * dtvar)
                dt2 = lowestdtfactor * dtvar;
        }

        double dotDMaxFromD = (Dmax - D) / dt2;
        double dotDMaxFormdotdotD = ((Dmax - D) < 0.0) ? 1000000.0 : sqrt(2 * max_accel * (Dmax - D));

        Eigen::Vector3d vMaxVector{{max_vel, dotDMaxFormdotdotD, dotDMaxFromD}};
        double dotDMaxFinal = vMaxVector.minCoeff();

        double aMaxDotD = (dotDMaxFinal - dotD) / dtvar;
        double aMaxD = 2 * (Dmax - D - dotD * dt2) / (std::pow(dt2, 2));

        Eigen::Vector3d aMaxVector{{1000000, aMaxDotD, aMaxD}};
        double dotdotDMaxFinal = aMaxVector.minCoeff();

        Eigen::Matrix<double, 1, 7> jacobianPos = direction_along_valid_region.transpose() * iiwa.jacobian().block(0, 0, 3, 7);

        double LambdaInvPos = (jacobianPos * iiwa.invmass() * jacobianPos.transpose())(0, 0) + (std::pow(0.3, 2));
        double lambdaPos = 1 / LambdaInvPos;
        Eigen::Matrix<double, 7, 1> JsatBar = iiwa.invmass() * jacobianPos.transpose() * lambdaPos;

        static bool CreateTaskSat = false;

        Eigen::Matrix<double, 7, 7> Psat = Eigen::Matrix<double, 7, 7>::Identity();

        Eigen::Matrix<double, 6, 1> linear_acceleration_cartesian = iiwa.jacobian() * iiwa.invmass() * task_torque;
        Eigen::Matrix<double, 3, 1> translation_acceleration = linear_acceleration_cartesian.block(0, 0, 3, 1);
        double linear_acceleration = translation_acceleration.transpose() * direction_along_valid_region;

        bool already_triggered_function_once = false;

        if (dotdotDMaxFinal + 0.001 < linear_acceleration)
        {
            CreateTaskSat = true;
        }
        else
        {
            if (CreateTaskSat)
                already_triggered_function_once = true;
            CreateTaskSat = false;
        }

        Eigen::Matrix<double, 7, 1> tauS = Eigen::Matrix<double, 7, 1>::Zero();

        if (CreateTaskSat)
        {
            Psat = Eigen::Matrix<double, 7, 7>::Identity() - jacobianPos.transpose() * JsatBar.transpose();
            tauS = jacobianPos.transpose() * lambdaPos * dotdotDMaxFinal;
        }

        auto projected_control = tauS + Psat * task_torque;
        task_torque = projected_control;
        return already_triggered_function_once;
    };

    bool needle_control(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
    {
        using namespace curan::robotic;
        bool is_transitioning = false;
        Eigen::Matrix<double, 4, 4> interpolated_pose;
        {
            std::lock_guard<std::mutex> g{mut};
            is_transitioning = position_hold_interpolator.advance();
            interpolated_pose = position_hold_interpolator.interpolated();
        }

        Eigen::Matrix<double, 6, 6> Lambda;
        {
            Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{iiwa.jacobian() * iiwa.invmass() * iiwa.jacobian().transpose(), Eigen::ComputeFullU};
            Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
            auto singular_values = svdofsitff.singularValues();
            for (auto &diag : singular_values)
                diag = (diag < 0.02) ? 0.02 : diag;
            Lambda = (P * singular_values.asDiagonal() * P.transpose()).inverse();
        }

        Eigen::Matrix<double, curan::robotic::number_of_joints, curan::robotic::number_of_joints> nullspace_projector = Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity() - iiwa.jacobian().transpose() * (iiwa.invmass() * iiwa.jacobian().transpose() * Lambda).transpose();
        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * interpolated_pose.block<3, 3>(0, 0));
        auto translation_error = interpolated_pose.block<3, 1>(0, 3) - iiwa.translation();

        static auto previous_error = E_AxisAngle.axis();
        previous_error = ((E_AxisAngle.axis() - previous_error).norm() > (-E_AxisAngle.axis() - previous_error).norm()) ? -E_AxisAngle.axis() : E_AxisAngle.axis();
        double magnitude_of_error = ((E_AxisAngle.axis() - previous_error).norm() > (-E_AxisAngle.axis() - previous_error).norm()) ? -E_AxisAngle.angle() : E_AxisAngle.angle();
        error.block<3, 1>(3, 0) = magnitude_of_error * iiwa.rotation() * previous_error;

        {
            Eigen::AngleAxisd interpolated_ori(interpolated_pose.block<3, 3>(0, 0));
            state.user_defined.block<3, 1>(3, 0) = error.block<3, 1>(3, 0);
            state.user_defined2.block<3, 1>(3, 0) = interpolated_ori.angle() * interpolated_ori.axis();

            state.user_defined.block<3, 1>(0, 0) = error.block<3, 1>(0, 0);
            state.user_defined2.block<3, 1>(0, 0) = interpolated_pose.block<3, 1>(0, 3);
        }
        Eigen::Matrix<double, number_of_joints, 1> error_in_nullspace = -iiwa.joints();
        /*
        We use the decomposition introduced in
        "Cartesian Impedance Control of Redundant and Flexible-Joint Robots" page 37
        but we use the nomenclature of the book in "Matrix Algebra From a Statistician's Perspective"
        */

        /*
        We have two controllers running simultaneously, the first controller, the so called cartersian impedance controller
        has a stiffness specified by the user, while the second impedance controller focuses on bringing all joints to their
        home configuration.
        */
        Eigen::Matrix<double, 6, 6> B0_cartesian;
        Eigen::Matrix<double, 6, 6> Q_cartesian;
        {
            Eigen::LLT<Eigen::Matrix<double, 6, 6>> lltOfLambda(Lambda);
            Eigen::Matrix<double, 6, 6> L = lltOfLambda.matrixL();
            Eigen::Matrix<double, 6, 6> R = L.inverse();
            Eigen::Matrix<double, 6, 6> C = R * stiffness * R.transpose();
            Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> svdofsitff{C, Eigen::ComputeFullU};
            Eigen::Matrix<double, 6, 6> P = svdofsitff.matrixU();
            Q_cartesian = R.inverse() * P;
            Eigen::Matrix<double, 6, 6> Qinv = Q_cartesian.inverse();
            B0_cartesian = Qinv * stiffness * Qinv.transpose();
        }
        Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_stiffness = 3.0 * Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity();
        Eigen::Matrix<double, number_of_joints, number_of_joints> B0_nullspace;
        Eigen::Matrix<double, number_of_joints, number_of_joints> Q_nullspace;
        {
            Eigen::LLT<Eigen::Matrix<double, number_of_joints, number_of_joints>> lltOfLambda(iiwa.mass());
            Eigen::Matrix<double, number_of_joints, number_of_joints> L = lltOfLambda.matrixL();
            Eigen::Matrix<double, number_of_joints, number_of_joints> R = L.inverse();
            Eigen::Matrix<double, number_of_joints, number_of_joints> C = R * nullspace_stiffness * R.transpose();
            Eigen::JacobiSVD<Eigen::Matrix<double, number_of_joints, number_of_joints>> svdofsitff{C, Eigen::ComputeFullU};
            Eigen::Matrix<double, number_of_joints, number_of_joints> P = svdofsitff.matrixU();
            Q_nullspace = R.inverse() * P;
            Eigen::Matrix<double, number_of_joints, number_of_joints> Qinv = Q_nullspace.inverse();
            B0_nullspace = Qinv * nullspace_stiffness * Qinv.transpose();
        }

        Eigen::Matrix<double, 6, 6> damping_cartesian = 2 * Q_cartesian * diagonal_damping * B0_cartesian.diagonal().array().sqrt().matrix().asDiagonal() * Q_cartesian.transpose();
        // it is assumed that the nullspace has a diagonal damping ratio of 1.0.
        Eigen::Matrix<double, number_of_joints, number_of_joints> damping_nullspace = 2 * Q_nullspace * B0_nullspace.diagonal().array().sqrt().matrix().asDiagonal() * Q_nullspace.transpose();

        /*
        We need to filer out the velocity because in steady stady state they can become problematic
        */
        static Eigen::Matrix<double, number_of_joints, 1> filtered_velocity = iiwa.velocities();
        auto val = 0.8 * filtered_velocity + 0.2 * iiwa.velocities();
        filtered_velocity = val;

        static auto previous_jacobian = iiwa.jacobian();
        static Eigen::Matrix<double, 6, curan::robotic::number_of_joints> jacobian_derivative = (iiwa.jacobian() - previous_jacobian) / iiwa.sample_time();
        jacobian_derivative = (0.8 * jacobian_derivative + 0.2 * (iiwa.jacobian() - previous_jacobian) / iiwa.sample_time()).eval();
        previous_jacobian = iiwa.jacobian();
        auto conditioned_matrix = iiwa.jacobian().transpose() * iiwa.jacobian();
        Eigen::JacobiSVD<Eigen::Matrix<double, curan::robotic::number_of_joints, curan::robotic::number_of_joints>> svdjacobian{conditioned_matrix, Eigen::ComputeFullU};
        auto U = svdjacobian.matrixU();
        auto singular_values = svdjacobian.singularValues();
        for (auto &diag : singular_values)
            diag = (diag < 0.02) ? 0.02 : diag;

        auto properly_conditioned_matrix = (U * singular_values.asDiagonal() * U.transpose());
        auto inverse_jacobian = properly_conditioned_matrix.inverse() * iiwa.jacobian().transpose();
        // these values can becomes quite high with large accelerations
        // Eigen::Matrix<double, 6, 1> forces_caused_by_coordinate_change = inverse_jacobian.transpose() * iiwa.mass() * inverse_jacobian * jacobian_derivative * inverse_jacobian * iiwa.jacobian() * filtered_velocity;
        Eigen::Matrix<double, 6, 1> forces_caused_by_coordinate_change = Eigen::Matrix<double, 6, 1>::Zero();
        // normalize the error to an upper bound
        // now its the tricky part

        static bool already_triggered_function_once = false;
        if (is_transitioning || already_triggered_function_once)
        { // if we are transitioning, then we just need to follow the interpolated point using the full impedance controller
            error.block<3, 1>(0, 0) = translation_error;
            state.cmd_tau = iiwa.jacobian().transpose() * (stiffness * error - damping_cartesian * iiwa.jacobian() * filtered_velocity - forces_caused_by_coordinate_change) + nullspace_projector * (nullspace_stiffness * error_in_nullspace - damping_nullspace * filtered_velocity - 10.0 * iiwa.mass() * filtered_velocity);
        }
        else
        {
            double numerator = translation_error.transpose() * interpolated_pose.block<3, 3>(0, 0).block<3, 1>(0, 2);
            double denominator = interpolated_pose.block<3, 3>(0, 0).block<3, 1>(0, 2).transpose() * interpolated_pose.block<3, 3>(0, 0).block<3, 1>(0, 2);
            double scaling_along_axis = numerator / denominator;
            error.block<3, 1>(0, 0) = scaling_along_axis * interpolated_pose.block<3, 3>(0, 0).block<3, 1>(0, 2);
            state.cmd_tau = iiwa.jacobian().transpose() * (stiffness * error - damping_cartesian * iiwa.jacobian() * filtered_velocity - forces_caused_by_coordinate_change) + nullspace_projector * (nullspace_stiffness * error_in_nullspace - damping_nullspace * filtered_velocity - 10.0 * iiwa.mass() * filtered_velocity);
            //already_triggered_function_once = barrier_function_for_needle_controller(iiwa, state.cmd_tau, interpolated_pose);
        }
        return is_transitioning;
    }

    curan::robotic::EigenState &&update(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians) override
    {
        ControlModes mode;
        static double currentTime = 0.0;

        {
            std::lock_guard<std::mutex> g{mut};
            mode = current_mode;
        }

        switch (mode)
        {
        case POSITION_HOLD:
            transitioning = position_hold_control(iiwa, state, composed_task_jacobians);
            break;
        case FREE_HAND:
            transitioning = free_hand_control(iiwa, state, composed_task_jacobians);
            break;
        case NEEDLE_POSITIONING:
            transitioning = needle_control(iiwa,state,composed_task_jacobians);
            break;
        default:
            throw std::runtime_error("selected a control mode that is unavailable");
            break;
        }
        state.cmd_q = iiwa.joints() + Eigen::Matrix<double, curan::robotic::number_of_joints, 1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));
        currentTime += iiwa.sample_time();
        return std::move(state);
    }

    void async_hand_guidance()
    {
        std::lock_guard<std::mutex> g{mut};
        current_mode = ControlModes::FREE_HAND;
    }

    void async_position_hold(Eigen::Matrix<double, 4, 4> in_target_pose)
    {
        std::lock_guard<std::mutex> g{mut};
        current_mode = ControlModes::POSITION_HOLD;
        Eigen::Matrix<double, 4, 4> in_initial_pose = position_hold_interpolator.interpolated();
        position_hold_interpolator = InterpolatedPose{in_initial_pose, in_target_pose, position_hold_interpolator.step_size()};
    }

    /*
    This is very special function. It assumes that we take the position of the target
    From this position and with the minimum offset we compute the projection of the
    current interpolated position towards the new interpolated position which is at
    this minimum distance or further away.

    Once the interpolator reaches the destination, then we move the target towards the target pose along
    the needle line. From this point we allow the surgeon to move the robot along the needle path freely.
    Once the needle traverses the barrier defined by the plane, we trigger a limiting algorithm to push
    the robot behind the barrier function. As soon as this happens, then the barrier algorithm is untriggered,
    and the algorithm switches to a classic impedance controller that attempts to hold its position
    */
    void async_needle(Eigen::Matrix<double, 4, 4> in_target_pose, double minimum_offset)
    {
        std::lock_guard<std::mutex> g{mut};
        current_mode = ControlModes::NEEDLE_POSITIONING;
        Eigen::Matrix<double, 4, 4> in_initial_pose = position_hold_interpolator.interpolated();
        auto needle_direction = in_target_pose.block<3, 1>(0, 2);
        auto intermediate_target_pose = in_target_pose;
        double projection_of_flange_along_needle_line = (1 / needle_direction.norm()) * needle_direction.transpose() * (in_initial_pose.block<3, 1>(0, 3) - in_target_pose.block<3, 1>(0, 3));
        intermediate_target_pose.block<3, 1>(0, 3) = projection_of_flange_along_needle_line * needle_direction + in_target_pose.block<3, 1>(0, 3);
        position_hold_interpolator = InterpolatedPose{in_initial_pose, intermediate_target_pose, position_hold_interpolator.step_size(), [=]()
                                                      { return in_target_pose; }};
    }
};

int bar()
{
    Eigen::Matrix<double, 4, 4> initial_pose = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 4> target_pose = Eigen::Matrix<double, 4, 4>::Identity();
    initial_pose.block<3, 3>(0, 0) = Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();
    target_pose.block<3, 3>(0, 0) = Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();

    initial_pose.block<3, 1>(0, 3) = Eigen::Matrix<double, 3, 1>::Random();
    target_pose.block<3, 1>(0, 3) = Eigen::Matrix<double, 3, 1>::Random();
    InterpolatedPose position_hold_interpolator{initial_pose, target_pose, 0.0001};

    std::cout << "initial_pose : \n"
              << initial_pose << std::endl;
    std::cout << "target_pose : \n"
              << target_pose << std::endl;
    std::cout << position_hold_interpolator.interpolated() << std::endl;

    for (size_t i = 0; i < 10; ++i)
    {
        position_hold_interpolator.advance();
        std::cout << position_hold_interpolator.interpolated() << std::endl;
    }

    std::cout << position_hold_interpolator.interpolated() << std::endl;

    return 0;
}

int foo()
{
try{
    curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);
    curan::robotic::State state;
    for (size_t i = 0; i < curan::robotic::number_of_joints; ++i)
        state.q[i] = 1.0;
    robot_model.update(state);

    using namespace curan::robotic;

    std::unique_ptr<ControllerSwitcher> handguinding_controller = std::make_unique<ControllerSwitcher>(std::initializer_list<double>{500.0, 500.0, 500.0, 50.0, 50.0, 50.0}, std::initializer_list<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, 0.0005);
    Eigen::Matrix<double, 4, 4> initial_pose = Eigen::Matrix<double, 4, 4>::Identity();
    Eigen::Matrix<double, 4, 4> target_pose = Eigen::Matrix<double, 4, 4>::Identity();

    Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();
    initial_pose.block<3, 1>(0, 3) = curan::robotic::convert(state.translation);
    initial_pose.block<3, 3>(0, 0) = curan::robotic::convert(state.rotation).reshaped(3, 3);
    target_pose.block<3, 1>(0, 3) = initial_pose.block<3, 1>(0, 3);

    target_pose.block<3, 3>(0, 0) = initial_pose.block<3, 3>(0, 0) * Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();
    double time = 0.0;
    size_t counter = 1;

    std::cout << "initial config: \n" << robot_model.homogenenous_transformation() << std::endl;;

    while (keep_running.load())
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        if (time < 5)
        {
        }
        else if (time < 10 && counter == 1)
        {
            std::cout << "switches to target pose";
            ++counter;
            handguinding_controller->async_position_hold(target_pose);
        }
        else if (time < 15 && counter == 2)
        {
            std::cout << "switches to needle pose";
            ++counter;
            target_pose.block<3, 3>(0, 0) = initial_pose.block<3, 3>(0, 0) * Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();
            target_pose.block<3, 1>(0, 3) += Eigen::Matrix<double, 3, 1>::Ones() * 0.1;
            handguinding_controller->async_needle(target_pose, 0.1);
            time = 0.0;
        }
        state = curan::robotic::simulate_next_timestamp(robot_model, handguinding_controller.get(), sample_time, state, external_torque);
        time += std::chrono::duration<double>(sample_time).count();
        external_torque = Eigen::Matrix<double, 7, 1>::Ones()*std::sin(time); 
    }
    return 0;
} catch(std::runtime_error& e){
    std::cout << "exception throw:\n" << e.what() << std::endl;
    return 1;
} catch(...){
    std::cout << "unknown exception\n" << std::endl;
    return 2;
}
}

int main()
{
    std::atomic<curan::robotic::State> atomic_state;
    curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);

    curan::robotic::State state;
    for (size_t i = 0; i < curan::robotic::number_of_joints; ++i)
        state.q[i] = 1.0;
    robot_model.update(state);
    atomic_state.store(state);

    using namespace curan::robotic;
    auto interface_callable = [&](vsg::CommandBuffer &cb)
    {
        custom_interface(cb, atomic_state);
    };

    curan::renderable::ImGUIInterface::Info info_gui{interface_callable};
    auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    info.imgui_interface = ui_interface;
    curan::renderable::Window::WindowSize size{2000, 1200};
    info.window_size = size;
    curan::renderable::Window window{info};

    std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
    curan::renderable::SequencialLinks::Info create_info;
    create_info.convetion = vsg::CoordinateConvention::Y_UP;
    create_info.json_path = robot_path;
    create_info.number_of_links = 8;
    auto robot = curan::renderable::SequencialLinks::make(create_info);
    window << robot;

    std::cout << "initial config: \n" << robot_model.homogenenous_transformation() << std::endl;

    Eigen::AngleAxis<double> perturbation_from_initial{10,Eigen::Matrix<double,3,1>::Ones()};

    std::atomic<bool> keep_running = true;
    std::unique_ptr<ControllerSwitcher> handguinding_controller = std::make_unique<ControllerSwitcher>(std::initializer_list<double>{500.0, 500.0, 500.0, 50.0, 50.0, 50.0}, std::initializer_list<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, 0.0005);
    
    {
        auto pool = curan::utilities::ThreadPool::create(2);
        pool->submit(curan::utilities::Job{"value", [&]()
                                           {
                                               std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                                                Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               Eigen::Matrix<double, 4, 4> initial_pose = robot_model.homogenenous_transformation();
                                               Eigen::Matrix<double, 4, 4> target_pose1 = initial_pose;
                                               Eigen::Matrix<double, 4, 4> target_pose2 = initial_pose;
                                               Eigen::Matrix<double, 6, 1> cartesian_force = Eigen::Matrix<double, 6, 1>::Zero();
                                               cartesian_force(2,0) = 1;
                                               handguinding_controller->position_hold_interpolator = InterpolatedPose{initial_pose, initial_pose, handguinding_controller->position_hold_interpolator.step_size()};
                                               target_pose2.block<3, 1>(0, 3) += 0.1*initial_pose.block<3,1>(0,1)+0.1*initial_pose.block<3,1>(0,2);
                                               double time = 0.0;
                                               size_t counter = 1;
                                               while (keep_running.load())
                                               {
                                                   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                                   static bool first_time_pose1 = true;
                                                   static bool first_time_pose2 = true;
                                                   if ( time < 5 && first_time_pose1)
                                                   {
                                                       std::cout << "switches to target pose 1\ns";
                                                       handguinding_controller->async_position_hold(target_pose1);
                                                       first_time_pose1 = false;
                                                   }
                                                   else if ( time >5 && time < 10 && first_time_pose2)
                                                   {
                                                        /*
                                                       std::cout << "switches to needle pose";
                                                       ++counter;
                                                       target_pose.block<3, 3>(0, 0) = initial_pose.block<3, 3>(0, 0) * Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();
                                                       target_pose.block<3, 1>(0, 3) += Eigen::Matrix<double, 3, 1>::Ones() * 0.1;
                                                       handguinding_controller->async_needle(target_pose, 0.1);
                                                       time = 0.0;
                                                       */
                                                       std::cout << "switches to target pose 2\n";
                                                       handguinding_controller->async_needle(target_pose2, 0.1);
                                                       //external_torque = robot_model.jacobian().transpose()*(cartesian_force);
                                                       //handguinding_controller->async_position_hold(target_pose2);
                                                       first_time_pose2 = false;
                                                       
                                                   } else if(time > 10 ) {
                                                        external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                                        first_time_pose1 = true;
                                                        first_time_pose2 = true;
                                                        time = 0.0;
                                                   }
                                                   state = curan::robotic::simulate_next_timestamp(robot_model, handguinding_controller.get(), sample_time, state, external_torque);
                                                   atomic_state.store(state, std::memory_order_relaxed);
                                                   time += std::chrono::duration<double>(sample_time).count();
                                               }
                                           }});

        while (window.run_once() && keep_running)
        {
            auto current_state = atomic_state.load();
            for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
                robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, current_state.q[joint_index]);
        }
        keep_running = false;
    }
    /*
        auto now = std::chrono::system_clock::now();
        auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        std::string filename{CURAN_COPIED_RESOURCE_PATH "/controller" + std::to_string(UTC) + ".json"};
        std::cout << "creating filename with measurments :" << filename << std::endl;
        std::ofstream o(filename);
        o << recording_of_states;
    */
    return 0;
}