#include "robotutils/LBRController.h"
#include "robotutils/JointVelocityController.h"
#include "utils/Logger.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <csignal>

curan::robotic::RobotLBR* robot_pointer = nullptr;
constexpr unsigned short DEFAULT_PORTID = 30200;

void signal_handler(int signal){
	if(robot_pointer)
        robot_pointer->cancel();
}

enum FixedPlane{
    PLANE_X,
    PLANE_Y,
    PLANE_Z
};

struct TrajecGeneration{
    const double c_theta;
    const double c_radius;
    const double c_flat;
    const double nominal_radius;
    const double nominal_flatten;

    TrajecGeneration(double in_c_theta,
                     double in_c_radius,
                     double in_c_flatten,
                     double in_nominal_radius,
                     double in_nominal_flatten) : 
                        c_theta{in_c_theta},
                        c_radius{in_c_radius}, 
                        c_flat{in_c_flatten} , 
                        nominal_radius{in_nominal_radius},
                        nominal_flatten{in_nominal_flatten}
    {

    }

    template<FixedPlane plane>
    Eigen::Matrix<double,3,1> compute(Eigen::Matrix<double,3,1> position){
        if constexpr (plane == FixedPlane::PLANE_X){
            double radius = std::sqrt(position[0]*position[0]+position[1]*position[1]);
            double flat = position[2];
            double radial_vel = -c_radius*(radius-nominal_radius);
            double angular_vel = c_theta;
            double flatten_vel = -c_flat*(flat-nominal_flatten);
            Eigen::Matrix<double,3,1> velocity_cylindrical{{radial_vel,angular_vel,flatten_vel}};
            Eigen::Matrix<double,3,3> jacobian_representation{{0.0 , 0.0 , 0.0 , 
                                                               0.0 , 0.0 , 0.0 ,
                                                               0.0 , 0.0 , 0.0}};
            Eigen::Matrix<double,3,1> cartesian_velocity = jacobian_representation*velocity_cylindrical;
            return cartesian_velocity;
        }
        if constexpr (plane == FixedPlane::PLANE_Y){
            double radius = std::sqrt(position[0]*position[0]+position[1]*position[1]);
            double flat = position[2];
            double radial_vel = -c_radius*(radius-nominal_radius);
            double angular_vel = c_theta;
            double flatten_vel = -c_flat*(flat-nominal_flatten);
            Eigen::Matrix<double,3,1> velocity_cylindrical{{radial_vel,angular_vel,flatten_vel}};
            Eigen::Matrix<double,3,3> jacobian_representation{{0.0 , 0.0 , 0.0 , 
                                                               0.0 , 0.0 , 0.0 ,
                                                               0.0 , 0.0 , 0.0}};
            Eigen::Matrix<double,3,1> cartesian_velocity = jacobian_representation*velocity_cylindrical;
            return cartesian_velocity;
        }
        if constexpr (plane == FixedPlane::PLANE_Z){
            double radius = std::sqrt(position[0]*position[0]+position[1]*position[1]);
            double flat = position[2];
            double radial_vel = -c_radius*(radius-nominal_radius);
            double angular_vel = c_theta;
            double flatten_vel = -c_flat*(flat-nominal_flatten);
            Eigen::Matrix<double,3,1> velocity_cylindrical{{radial_vel,angular_vel,flatten_vel}};
            Eigen::Matrix<double,3,3> jacobian_representation{{0.0 , 0.0 , 0.0 , 
                                                               0.0 , 0.0 , 0.0 ,
                                                               0.0 , 0.0 , 0.0}};
            Eigen::Matrix<double,3,1> cartesian_velocity = jacobian_representation*velocity_cylindrical;
            return cartesian_velocity;
        }
    };
};

struct RhytmicMotion : public curan::robotic::UserData{

    RhytmicMotion(){

    }

    curan::robotic::EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, curan::robotic::EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override{
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;


        /*
        The Java controller has two values which it reads, namely: 
        1) commanded_joint_position 
        2) commanded_torque 
        The torque is computed in the previous line, but the position can remain empty. One problem with this approach is that if the deviation
        between the reference position and the commanded position is larger than 5 degrees the robot triggers a safety stop 1. To avoid this
        the first solution is to set the commanded position to be equal to the current position. This approach has a drawback where the error between
        both commanded and current position is always zero, which results in the friction compensator being "shut off". We avoid this problem
        by adding a small perturbation to the reference position with a relative high frequency. 
        */
        state.cmd_q = state.q + Eigen::Matrix<double,7,1>::Constant(0.5 / 180.0 * M_PI * sin(2 * M_PI * 10 * currentTime));

        currentTime += state.sampleTime;
        return std::move(state);
    }

};

int main(){
    std::unique_ptr<RhytmicMotion> handguinding_controller = std::make_unique<RhytmicMotion>();
    curan::robotic::RobotLBR client{handguinding_controller.get()};
    const auto& access_point = client.atomic_acess();
	try
	{
        std::list<curan::robotic::State> list_of_recorded_states;
		curan::utilities::cout << "Lauching robot control thread\n";
		
		KUKA::FRI::UdpConnection connection;
		KUKA::FRI::ClientApplication app(connection, client);
		bool success = app.connect(DEFAULT_PORTID, NULL);
		success = app.step();
		while (success && client){
			success = app.step();
            list_of_recorded_states.push_back(access_point.load());
        }
		app.disconnect();
        auto now = std::chrono::system_clock::now();
		auto UTC = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
		std::string filename{CURAN_COPIED_RESOURCE_PATH"/measurments"+std::to_string(UTC)+".json"};
		std::cout << "creating filename with measurments :" << filename << std::endl;
		std::ofstream o(filename);
		o << list_of_recorded_states;
		return 0;
	}
	catch (...)
	{
		std::cout << "robot control exception\n";
		return 1;
	}
}