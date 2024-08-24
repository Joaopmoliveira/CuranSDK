#include "robotutils/LBRController.h"
#include "robotutils/SimulateModel.h"

#include "utils/Logger.h"
#include "utils/TheadPool.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <csignal>
#include <Eigen/Geometry>

struct ScrollingBuffer
{
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000)
    {
        MaxSize = max_size;
        Offset = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y)
    {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x, y));
        else
        {
            Data[Offset] = ImVec2(x, y);
            Offset = (Offset + 1) % MaxSize;
        }
    }
    void Erase()
    {
        if (Data.size() > 0)
        {
            Data.shrink(0);
            Offset = 0;
        }
    }
};

void custom_interface(vsg::CommandBuffer &cb, std::atomic<curan::robotic::State> &atomic_access)
{
    auto state = atomic_access.load(std::memory_order_relaxed);
    ImGui::Begin("Cart error"); // Create a window called "Hello, world!" and append into it.
    static std::array<ScrollingBuffer, 6> error_buffers;
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

curan::robotic::RobotLBR *robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal)
{
    if (robot_pointer)
        robot_pointer->cancel();
}

struct InterpolatedPose
{
    InterpolatedPose(Eigen::Matrix<double, 4, 4> in_initial_pose,
                     Eigen::Matrix<double, 4, 4> in_target_pose,
                     double in_interpolation_velocity) : initial_pose{in_initial_pose},
                                                         target_pose{in_target_pose},
                                                         t{0},
                                                         interpolation_velocity{in_interpolation_velocity}
    {
        current_pose = Eigen::Matrix<double, 4, 4>::Identity();
    }

    InterpolatedPose(double in_interpolation_velocity) : t{0},
                                                         interpolation_velocity{in_interpolation_velocity}
    {
        current_pose = Eigen::Matrix<double, 4, 4>::Identity();
        target_pose = Eigen::Matrix<double, 4, 4>::Identity();
        initial_pose = Eigen::Matrix<double, 4, 4>::Identity();
    }

    Eigen::Matrix<double, 4, 4> target_pose;
    Eigen::Matrix<double, 4, 4> initial_pose;
    Eigen::Matrix<double, 4, 4> current_pose;
    double t;
    double interpolation_velocity;

    bool advance()
    {
        bool interpolating = false;
        t += interpolation_velocity;
        if (t > 1.0)
            t = 1.0;
        else
            interpolating = true;
        current_pose.block<3, 1>(0, 3) = t * target_pose.block<3, 1>(0, 3) + (1 - t) * initial_pose.block<3, 1>(0, 3);
        Eigen::Quaternion<double> quat1{initial_pose.block<3, 3>(0, 0)};
        Eigen::Quaternion<double> quat2{target_pose.block<3, 3>(0, 0)};
        Eigen::Quaternion<double> qres = quat1.slerp(t, quat2);
        if (qres.w() > 0.0)
            qres = Eigen::Quaternion<double>{-qres.w(), -qres.x(), -qres.y(), -qres.z()};
        current_pose.block<3, 3>(0, 0) = qres.toRotationMatrix();
        return interpolating;
    };
};

struct ControllerSwitcher : public curan::robotic::UserData
{
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

    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> diagonal_damping;

    InterpolatedPose position_hold_interpolator;

    bool free_hand_control(bool is_transitioning, const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
    {
        state.cmd_tau = -iiwa.mass() * iiwa.velocities();
        return false;
    }

    bool position_hold_control(bool is_transitioning, const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians)
    {
        using namespace curan::robotic;
        is_transitioning = position_hold_interpolator.advance();
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

        Eigen::AngleAxisd E_AxisAngle(iiwa.rotation().transpose() * position_hold_interpolator.current_pose.block<3, 3>(0, 0));
        error.block<3, 1>(0, 0) = (position_hold_interpolator.current_pose.block<3, 1>(0, 3) - iiwa.translation());
        error.block<3, 1>(3, 0) = E_AxisAngle.angle() * iiwa.rotation() * E_AxisAngle.axis();
        state.user_defined.block<6, 1>(0, 0) = error;

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
        Eigen::Matrix<double, number_of_joints, number_of_joints> nullspace_stiffness = 10.0 * Eigen::Matrix<double, number_of_joints, number_of_joints>::Identity();
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
        {
            diag = (diag < 0.02) ? 0.02 : diag;
        }
        auto properly_conditioned_matrix = (U * singular_values.asDiagonal() * U.transpose());
        auto inverse_jacobian = properly_conditioned_matrix.inverse() * iiwa.jacobian().transpose();
        // these values can becomes quite high with large accelerations
        Eigen::Matrix<double, 6, 1> forces_caused_by_coordinate_change = inverse_jacobian.transpose() * iiwa.mass() * inverse_jacobian * jacobian_derivative * inverse_jacobian * iiwa.jacobian() * filtered_velocity;

        // normalize the error to an upper bound
        state.cmd_tau = iiwa.jacobian().transpose() * (stiffness * error - damping_cartesian * iiwa.jacobian() * filtered_velocity - forces_caused_by_coordinate_change) + nullspace_projector * (nullspace_stiffness * error_in_nullspace - damping_nullspace * filtered_velocity - 10.0 * iiwa.mass() * filtered_velocity);
        // std::cout << "in torque: " << state.cmd_tau.transpose() << std::endl;
        return is_transitioning;
    }
    enum ControlModes
    {
        POSITION_HOLD,
        FREE_HAND
    };

    ControlModes current_mode = ControlModes::FREE_HAND;
    bool transitioning = false;

    curan::robotic::EigenState &&update(const curan::robotic::RobotModel<curan::robotic::number_of_joints> &iiwa, curan::robotic::EigenState &&state, Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &composed_task_jacobians) override
    {
        static double currentTime = 0.0;
        switch (current_mode)
        {
        case POSITION_HOLD:
            transitioning = position_hold_control(transitioning, iiwa, state, composed_task_jacobians);
            break;
        case FREE_HAND:
            transitioning = free_hand_control(transitioning, iiwa, state, composed_task_jacobians);
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
        transitioning = true;
        current_mode = ControlModes::FREE_HAND;
    }

    void async_position_hold(Eigen::Matrix<double, 4, 4> in_initial_pose,
                             Eigen::Matrix<double, 4, 4> in_target_pose)
    {
        transitioning = true;
        current_mode = ControlModes::POSITION_HOLD;
        position_hold_interpolator = InterpolatedPose{in_initial_pose, in_target_pose, position_hold_interpolator.interpolation_velocity};
    }
};

int main()
{
    std::atomic<curan::robotic::State> atomic_state;
    curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);
    auto initial_config = Eigen::Matrix<double, 7, 1>::Random();
    curan::robotic::State state;
    for (size_t i = 0; i < curan::robotic::number_of_joints; ++i)
        state.q[i] = initial_config[i];
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

    std::atomic<bool> keep_running = true;
    std::unique_ptr<ControllerSwitcher> handguinding_controller = std::make_unique<ControllerSwitcher>(std::initializer_list<double>{500.0, 500.0, 500.0, 50.0, 50.0, 50.0}, std::initializer_list<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, 0.001);
    std::list<curan::robotic::State> recording_of_states;
    {
        auto pool = curan::utilities::ThreadPool::create(2);
        pool->submit(curan::utilities::Job{"value", [&]()
                                           {
                                               Eigen::Matrix<double, 4, 4> initial_pose = Eigen::Matrix<double, 4, 4>::Identity();
                                               Eigen::Matrix<double, 4, 4> target_pose = Eigen::Matrix<double, 4, 4>::Identity();

                                               Eigen::Matrix<double, 7, 1> external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                               double time = 0.0;
                                               while (keep_running.load())
                                               {
                                                   std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
                                                   if (time > 10 && time < 15)
                                                   {
                                                       external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                                       // external_torque[6] = 10;
                                                   }
                                                   else if (time > 15)
                                                   {
                                                       initial_pose.block<3, 1>(0, 3) = curan::robotic::convert(atomic_state.load().translation);
                                                       initial_pose.block<3, 3>(0, 0) = curan::robotic::convert(atomic_state.load().rotation).reshaped(3, 3);
                                                       target_pose.block<3, 1>(0, 3) = curan::robotic::convert(atomic_state.load().translation); //+Eigen::Matrix<double,3,1>::Random();
                                                       target_pose.block<3, 3>(0, 0) = initial_pose.block<3, 3>(0, 0) * Eigen::Quaternion<double>::UnitRandom().toRotationMatrix();
                                                       handguinding_controller->async_position_hold(initial_pose, target_pose);
                                                       std::cout << target_pose << std::endl;
                                                       time = 0.0;
                                                       external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                                   }
                                                   else
                                                   {
                                                       external_torque = Eigen::Matrix<double, 7, 1>::Zero();
                                                   }
                                                   state = curan::robotic::simulate_next_timestamp(robot_model, handguinding_controller.get(), sample_time, state, external_torque);
                                                   recording_of_states.push_back(state);
                                                   atomic_state.store(state, std::memory_order_relaxed);
                                                   time += std::chrono::duration<double>(sample_time).count();
                                               }
                                           }});

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        while (window.run_once())
        {
            auto current_state = atomic_state.load();
            for (size_t joint_index = 0; joint_index < curan::robotic::number_of_joints; ++joint_index)
                robot->cast<curan::renderable::SequencialLinks>()->set(joint_index, current_state.q[joint_index]);
        }
        keep_running = false;
    }

    auto now = std::chrono::system_clock::now();
    auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    std::string filename{CURAN_COPIED_RESOURCE_PATH "/controller" + std::to_string(UTC) + ".json"};
    std::cout << "creating filename with measurments :" << filename << std::endl;
    std::ofstream o(filename);
    o << recording_of_states;
    return 0;
}