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

constexpr double delta_min = 1e-5;
const double pi = std::acos(-1);

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
    const Eigen::Matrix<double,3,1> nominal_pos;

    TrajecGeneration(double in_c_theta,
                     double in_c_radius,
                     double in_c_flatten,
                     double in_nominal_radius,
                     Eigen::Matrix<double,3,1> in_nominal_pos) : 
                        c_theta{in_c_theta},
                        c_radius{in_c_radius}, 
                        c_flat{in_c_flatten} , 
                        nominal_radius{in_nominal_radius},
                        nominal_pos{in_nominal_pos}
    {

    }

    template<FixedPlane plane>
    Eigen::Matrix<double,3,1> compute(Eigen::Matrix<double,3,1> in_position){
        Eigen::Matrix<double,3,1> translated_position = in_position-nominal_pos;

        Eigen::Matrix<double,3,3> rotation_mat;
        if constexpr (plane == FixedPlane::PLANE_X)
            rotation_mat << 0.0, 0.0 ,-1.0 ,
                            0.0, 1.0 , 0.0 ,
                            1.0, 0.0 , 0.0;
        else if constexpr (plane == FixedPlane::PLANE_Y)
            rotation_mat << 1.0, 0.0 , 0.0 ,
                            0.0, 0.0 ,-1.0 ,
                            0.0, 1.0 , 1.0;
        else 
            rotation_mat << 1.0, 0.0 , 0.0 ,
                            0.0, 1.0 , 0.0 ,
                            0.0, 0.0 , 1.0;

        Eigen::Matrix<double,3,1> position = rotation_mat*translated_position;

        double radius = std::sqrt(position[0]*position[0]+position[1]*position[1]);
        double angle = (radius > delta_min) ? std::atan2(position[1],position[0]) : 0.0 ;
        double flat = position[2];
        
        double radial_vel = -c_radius*(radius-nominal_radius);
        double angular_vel = c_theta;
        double flatten_vel = -c_flat*flat;
        Eigen::Matrix<double,3,1> velocity_cylindrical{{radial_vel,angular_vel,flatten_vel}};
        
        Eigen::Matrix<double,3,3> jacobian_representation;
        jacobian_representation << std::cos(angle) , -radius*std::sin(angle) , 0.0 , 
                                   std::sin(angle) ,  radius*std::cos(angle) , 0.0 ,
                                        0.0 ,                 0.0 ,            1.0;
        Eigen::Matrix<double,3,1> cartesian_velocity = rotation_mat.transpose()*jacobian_representation*velocity_cylindrical;
        return cartesian_velocity;
    };
};

struct RhytmicMotion : public curan::robotic::UserData{

    Eigen::DiagonalMatrix<double, 6> gain;

    TrajecGeneration generator; 

    RhytmicMotion() : generator{1.0 , 1.0 , 3.0 , 0.1 ,Eigen::Matrix<double,3,1>{{-0.63,0.0,0.294}}} , gain{20,20,20,20,20,20}{

    }

    curan::robotic::EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, curan::robotic::EigenState&& state, Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>& composed_task_jacobians) override{
        static double currentTime = 0.0;
        /*
        We remove some energy from the system whilst moving the robot in free space. Thus we guarantee that the system is passive
        */
       
        //state.cmd_tau = -iiwa->M * 10 * iiwa->qDot;

        static Eigen::Matrix<double,7,1> equilibrium_joint_pos = state.q;
        
        
        Eigen::Matrix<double,3,1> velocity_translation = generator.compute<FixedPlane::PLANE_X>(state.translation);
        Eigen::Matrix<double,3,3> desired_rotation_mat;
        desired_rotation_mat << 0.0, 1.0 , 0.0 ,
                                1.0, 0.0 , 0.0 ,
                                0.0, 0.0 ,-1.0;
        Eigen::Matrix<double,3,3> error_matrix = desired_rotation_mat.transpose()*state.rotation;
        Eigen::AngleAxisd E_AxisAngle(error_matrix);
        Eigen::Matrix<double,3,1> velocity_rotation = -E_AxisAngle.angle()*state.rotation*E_AxisAngle.axis();

        Eigen::Matrix<double,6,1> desired_velocity = Eigen::Matrix<double,6,1>::Zero();
        desired_velocity.block(0,0,3,1) = velocity_translation;
        desired_velocity.block(3,0,3,1) = velocity_rotation;

        Eigen::Matrix<double,6,1> current_velocity = state.jacobian*state.dq;

        Eigen::Matrix<double,6,6> lambda = (state.jacobian*state.invmassmatrix*state.jacobian.transpose()+Eigen::Matrix<double,6,6>::Identity()*0.1*0.3).inverse();
	    Eigen::Matrix<double,7,6> jbar = state.invmassmatrix * state.jacobian.transpose() * lambda;
	    Eigen::Matrix<double,7,7> nullSpaceTranslation = Eigen::Matrix<double,7,7>::Identity() - state.jacobian.transpose() * jbar.transpose();

        state.user_defined.block(0,0,6,1) = desired_velocity;
        state.user_defined2 = state.jacobian.transpose()*lambda*gain*(desired_velocity-current_velocity)-nullSpaceTranslation*(state.massmatrix * 10 * equilibrium_joint_pos);
        state.cmd_tau = state.user_defined2;

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
    std::signal(SIGINT, signal_handler);
    std::unique_ptr<RhytmicMotion> handguinding_controller = std::make_unique<RhytmicMotion>();
    curan::robotic::RobotLBR client{handguinding_controller.get()};
    const auto& access_point = client.atomic_acess();
    robot_pointer = &client;
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
		std::string filename{CURAN_COPIED_RESOURCE_PATH"/ds_actuation"+std::to_string(UTC)+".json"};
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