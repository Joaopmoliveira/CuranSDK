#include "robotutils/LBRController.h"
#include "robotutils/ImpedanceController.h"
#include "robotutils/SimulateModel.h"

int main()
{
    Eigen::Matrix<double, 3, 3> desired_rotation = Eigen::Matrix<double, 3, 3>::Identity();
    desired_rotation << 1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0;
    Eigen::Matrix<double, 3, 1> desired_translation = Eigen::Matrix<double, 3, 1>::Zero();
    desired_translation << -0.66809, -0.00112052, 0.443678;
    curan::robotic::Transformation equilibrium{desired_rotation, desired_translation};
    std::unique_ptr<curan::robotic::ImpedanceController> handguinding_controller = std::make_unique<curan::robotic::ImpedanceController>(equilibrium,
                                                                                                         std::initializer_list<double>({800.0, 800.0, 800.0, 100.0, 100.0, 100.0}),
                                                                                                         std::initializer_list<double>({1.0, 1.0, 1.0, 1.0, 1.0, 1.0}));

    curan::robotic::RobotModel<7> robot_model{CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_mass_data.json", CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/robot_kinematic_limits.json"};
    constexpr auto sample_time = std::chrono::milliseconds(1);
    curan::robotic::State state;
    Eigen::Matrix<double, 7, 1> external_torque = 10*Eigen::Matrix<double, 7, 1>::Ones();
    double time = 0.0;
    std::list<curan::robotic::State> list_of_states;
    for(size_t i = 0; i< 3; ++i)
    {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        state = curan::robotic::simulate_next_timestamp(robot_model, handguinding_controller.get(), sample_time, state, external_torque);
        std::cout << "jacobian:\n" << robot_model.jacobian() << "\n\n";
        std::cout << "mass:\n" << robot_model.mass() << "\n\n";
        std::cout << "rotation :\n" << robot_model.rotation() << "\n\n";
        list_of_states.push_back(state);
        time += std::chrono::duration<double>(sample_time).count();
    }

    auto now = std::chrono::system_clock::now();
	auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
	size_t index = 0;
	std::string filename{CURAN_COPIED_RESOURCE_PATH"/measurments_for_jacobian_check.json"};
	std::cout << "creating filename with measurments :" << filename << std::endl;
	std::ofstream o(filename);
	o << list_of_states;
    return 0;
}