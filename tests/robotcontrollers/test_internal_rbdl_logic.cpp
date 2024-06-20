#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include "utils/Reader.h"
#include "robotutils/LBRController.h"

std::tuple<double,size_t,size_t,size_t,size_t,double> eval_model_mass(RigidBodyDynamics::Model* model, RigidBodyDynamics::Math::VectorNd Q){
	RigidBodyDynamics::Math::VectorNd QDot = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);
	RigidBodyDynamics::Math::VectorNd QDDot = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);
	RigidBodyDynamics::Math::VectorNd Tau = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);
	RigidBodyDynamics::Math::VectorNd TauInv = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);

	RigidBodyDynamics::Math::MatrixNd H1 = RigidBodyDynamics::Math::MatrixNd::Constant ((size_t) model->dof_count, (size_t) model->dof_count, 0.);
    RigidBodyDynamics::Math::MatrixNd H2 = RigidBodyDynamics::Math::MatrixNd::Constant ((size_t) model->dof_count, (size_t) model->dof_count, 0.);
	RigidBodyDynamics::Math::VectorNd C = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);
	RigidBodyDynamics::Math::VectorNd QDDot_zero = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);
	RigidBodyDynamics::Math::VectorNd QDDot_crba = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model->dof_count, 0.);
    
    H1.setZero();
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    CompositeRigidBodyAlgorithm(*model, Q, H1, false);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    size_t difference_with_aprox =  std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();

    H2.setZero();
    begin = std::chrono::steady_clock::now();
    CompositeRigidBodyAlgorithm(*model, Q, H2, true);
    end = std::chrono::steady_clock::now();
    size_t difference_without_aprox = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();

    Eigen::Matrix<double,7,7> value = H2;
    begin = std::chrono::steady_clock::now();
    auto inverse_1 = value.inverse();
    end = std::chrono::steady_clock::now();
    size_t inversion_with_fixed_matrix = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();

    begin = std::chrono::steady_clock::now();
    auto inverse_2 = H2.inverse();
    end = std::chrono::steady_clock::now();
    size_t inversion_with_variable_matrix = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
    return {(H2-H1).norm(),difference_with_aprox,difference_without_aprox,inversion_with_variable_matrix,inversion_with_fixed_matrix,(inverse_1-inverse_2).norm()};
}

void print_body(const RigidBodyDynamics::Body& body){
    std::cout << "body:\n";
    std::cout << "center of mass: \n" << body.mCenterOfMass << std::endl;
    std::cout << "inertia: \n" << body.mInertia << std::endl;
    std::cout << "mass: \n" << body.mMass << std::endl;
}

class Tool{
	double mass;
	RigidBodyDynamics::Math::Vector3d center_of_mass;
	RigidBodyDynamics::Math::Matrix3d inertia;

	
	RigidBodyDynamics::Math::Vector3d axis_origin;
};

template<size_t model_joints>
class RobotModel{
    private:
    // Data describing the robot
    std::array<double,model_joints> mass;
    std::array<double,model_joints> center_of_mass;
    std::array<RigidBodyDynamics::Math::Matrix3d,model_joints> inertia;
    std::array<RigidBodyDynamics::Math::Vector3d,model_joints> axis_direction;
    std::array<RigidBodyDynamics::Math::Vector3d,model_joints> axis_origin;

    // Parameters for computation of dynamics
	std::array<unsigned int,model_joints>body_id;
	std::array<RigidBodyDynamics::Body,model_joints> body;   
	std::array<RigidBodyDynamics::Joint,model_joints> joint;   
	RigidBodyDynamics::Model model{};  

    // Parameters to store current state of the robot
    Eigen::Matrix<double,model_joint,model_joint> massmatrix;
    Eigen::Matrix<double,model_joint,1> coriolis;
    Eigen::Matrix<double,model_joint,1> gravity;
    Eigen::Matrix<double,model_joint,1> q;
    Eigen::Matrix<double,model_joint,1> dq;
    Eigen::Matrix<double,model_joint,1> ddq;
    Eigen::Vector3d pointPosition;

    // Variable that signals if the control loop should continue
    std::atomic<bool> continue_robot_motion = true;

    public:
    //delete all robot copy operators so that we do not have to deal with nasty copies
    RobotModel(const RobotModel& other) = delete;
    RobotModel(RobotModel&& other) = delete;
    RobotModel& operator=(RobotModel&& other) = delete;
    RobotModel& operator=(const RobotModel& other) = delete; 

    RobotModel(std::filesystem::path models_data_directory){
        nlohmann::json table_data = nlohmann::json::parse(std::ifstream(models_data_directory));
        if(table_data.size()!=model_joints)
            throw std::runtime_error("the supplied model has a different number of parameters from the compiled model");

        auto iterator = tableDH.begin();

        for(size_t joint = 0; joint < model_joints; ++joint,++iterator){
            mass[joint] = (*iterator)["mass"];
            {
                std::stringstream ss{(*iterator)["center_of_mass"];};
                center_of_mass[joint] = curan::utilities::convert_matrix(ss,',');
            }
            {
                std::stringstream ss{(*iterator)["inertia"];};
                inertia[joint] = curan::utilities::convert_matrix(ss,',');
            }
            {
                std::stringstream ss{(*iterator)["axis_direction"];};
                axis_direction[joint] = curan::utilities::convert_matrix(ss,',');
            }
            {
                std::stringstream ss{(*iterator)["axis_origin"];};
                axis_origin[joint] = curan::utilities::convert_matrix(ss,',');
            }
        }

        model.gravity = RigidBodyDynamics::Math::Vector3d (0.0, 0.0, -9.81);

        body[0] = RigidBodyDynamics::Body(mass[0], com[0], inertia[0]);     
	    joint[0] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,axisDirection[0]);  
	    body_id[0] = model.AddBody(body_id[0], RigidBodyDynamics::Math::Xtrans(axisOrigin[0]), joint[0], body[0], std::to_string(0));

        for(size_t joint = 1; joint < model_joint; ++joint){
            body[joint] = RigidBodyDynamics::Body(mass[joint], com[joint], inertia[joint]);     
	        joint[joint] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,axisDirection[joint]);  
	        body_id[joint] = model.AddBody(body_id[joint-1], RigidBodyDynamics::Math::Xtrans(axisOrigin[joint]), joint[joint], body[joint], std::to_string(joint));
        }

    }

    inline operator bool() const {
        return continue_robot_motion.load(std::memory_order_relaxed); 
    }

    inline void cancel(){
        continue_robot_motion.store(false,std::memory_order_relaxed);
    }
};

int main(){

    return 0;
}

int fooo(){
	unsigned int body1_id, body2_id, body3_id, body4_id, body5_id, body6_id, body7_id;
	RigidBodyDynamics::Body body1, body2, body3, body4, body5, body6, body7;   
	RigidBodyDynamics::Joint joint1, joint2, joint3, joint4, joint5, joint6, joint7;   

	
	RigidBodyDynamics::Model model{};  

	std::vector<double> mass{{2.7426,4.9464,2.5451,4.6376,1.7140,2.4272,0.4219}};
    std::vector<RigidBodyDynamics::Math::Vector3d> com {{RigidBodyDynamics::Math::Vector3d{0.0   , -18.7e-3,  101.6e-3},
                                                         RigidBodyDynamics::Math::Vector3d{-0.21e-3,  25.0e-3,   82.5e-3},
                                                         RigidBodyDynamics::Math::Vector3d{ -0.20e-3,  19.5e-3,   98.4e-3},
                                                         RigidBodyDynamics::Math::Vector3d{ -0.21e-3, -20.1e-3,   86.0e-3},
                                                         RigidBodyDynamics::Math::Vector3d{-0.04e-3, -13.5e-3,   66.0e-3},
                                                         RigidBodyDynamics::Math::Vector3d{-0.35e-3,  51.4e-3,   17.1e-3},
                                                         RigidBodyDynamics::Math::Vector3d{-0.01e-3,   0.1e-3,   11.0e-3}}};


    std::vector<RigidBodyDynamics::Math::Matrix3d> inertia {{RigidBodyDynamics::Math::Matrix3d{0.24, 0.0,   0.0, 
				                                                                           0.0 , 0.024, 0.0,
								                                                           0.0,  0.0,   0.0128},
                                                         RigidBodyDynamics::Math::Matrix3d{0.0468, 0.0,    0.0,
				                                                                           0.0,   0.0282, 0.0,
								                                                           0.0,   0.0,    0.0101},
                                                         RigidBodyDynamics::Math::Matrix3d{0.02,  0.0,    0.0, 
				                                                                           0.0,   0.02,   0.0,
								                                                           0.0,   0.0,    0.06},
                                                         RigidBodyDynamics::Math::Matrix3d{0.04,  0.0,    0.0, 
				                                                                           0.0,   0.027,   0.0,
								                                                           0.0,   0.0,    0.01},
                                                         RigidBodyDynamics::Math::Matrix3d{0.019,  0.0,    0.0, 
				                                                                           0.0,    0.016,  0.0,
								                                                           0.0,    0.0,    0.012},
                                                         RigidBodyDynamics::Math::Matrix3d{0.007,  0.0,     0.0, 
				                                                                           0.0,    0.006,   0.0,
								                                                           0.0,    0.0,     0.005},
                                                         RigidBodyDynamics::Math::Matrix3d{0.0003, 0.0,     0.0,  
				                                                                           0.0,    0.0003,  0.0,
								                                                           0.0,    0.0,     0.0005}}};

    std::vector<RigidBodyDynamics::Math::Vector3d> axisDirection {{RigidBodyDynamics::Math::Vector3d{0.0,  0.0,  1.0},
                                                                    RigidBodyDynamics::Math::Vector3d{0.0,  1.0,  0.0},
                                                                    RigidBodyDynamics::Math::Vector3d{0.0,  0.0,  1.0},
                                                                    RigidBodyDynamics::Math::Vector3d{0.0, -1.0,  0.0},
                                                                    RigidBodyDynamics::Math::Vector3d{0.0,  0.0,  1.0},
                                                                    RigidBodyDynamics::Math::Vector3d{0.0,  1.0,  0.0},
                                                                    RigidBodyDynamics::Math::Vector3d{0.0,  0.0,  1.0}}};

    std::vector<RigidBodyDynamics::Math::Vector3d> axisOrigin {{RigidBodyDynamics::Math::Vector3d{0,   0,    152.5e-3},
                                                                    RigidBodyDynamics::Math::Vector3d{0, -11e-3, 187.5e-3},
                                                                    RigidBodyDynamics::Math::Vector3d{0, +11e-3, 212.5e-3},
                                                                    RigidBodyDynamics::Math::Vector3d{0, +11e-3, 187.5e-3},
                                                                    RigidBodyDynamics::Math::Vector3d{0, -11e-3, 212.5e-3},
                                                                    RigidBodyDynamics::Math::Vector3d{0, -62e-3, 187.5e-3},
                                                                    RigidBodyDynamics::Math::Vector3d{0, +62e-3,  79.6e-3}}};

	model.gravity = RigidBodyDynamics::Math::Vector3d (0.0, 0.0, -9.81);   
	
	body1 = RigidBodyDynamics::Body(mass[0], com[0], inertia[0]);     
    //print_body(body1);
	joint1 = RigidBodyDynamics::Joint(     RigidBodyDynamics::JointTypeRevolute,axisDirection[0]);  
	body1_id = model.AddBody(0, RigidBodyDynamics::Math::Xtrans(axisOrigin[0]), joint1, body1, "1");    
    //print_model_mass(model);

	body2 = RigidBodyDynamics::Body(mass[1], com[1], inertia[1]);     
    //print_body(body2);
	joint2 = RigidBodyDynamics::Joint (     RigidBodyDynamics::JointTypeRevolute,  axisDirection[1]);    
	body2_id = model.AddBody(1, RigidBodyDynamics::Math::Xtrans(axisOrigin[1]), joint2, body2, "2"); 
    //print_model_mass(model);    

	body3 = RigidBodyDynamics::Body(mass[2], com[2],inertia[2]);     
    //print_body(body3);
	joint3 = RigidBodyDynamics::Joint (     RigidBodyDynamics::JointTypeRevolute,axisDirection[2]);    
	body3_id = model.AddBody(2, RigidBodyDynamics::Math::Xtrans(axisOrigin[2]), joint3, body3, "3");    
    //print_model_mass(model); 

	body4 = RigidBodyDynamics::Body(mass[3], com[3], inertia[3]);   
    //print_body(body4);  
	joint4 = RigidBodyDynamics::Joint (     RigidBodyDynamics::JointTypeRevolute,axisDirection[3]);    
	body4_id = model.AddBody(3, RigidBodyDynamics::Math::Xtrans(axisOrigin[3]), joint4, body4, "4");   
    //print_model_mass(model);  

	body5 = RigidBodyDynamics::Body(mass[4], com[4], inertia[4]);  
    //print_body(body5);   
	joint5 = RigidBodyDynamics::Joint (     RigidBodyDynamics::JointTypeRevolute,axisDirection[4]);    
	body5_id = model.AddBody(4, RigidBodyDynamics::Math::Xtrans(axisOrigin[4]), joint5, body5, "5"); 
    //print_model_mass(model);    

	body6 = RigidBodyDynamics::Body(mass[5], com[5], inertia[5]);  
    //print_body(body6);      
	joint6 = RigidBodyDynamics::Joint (     RigidBodyDynamics::JointTypeRevolute,axisDirection[5]);    
	body6_id = model.AddBody(5, RigidBodyDynamics::Math::Xtrans(axisOrigin[5]), joint6, body6, "6");  
    //print_model_mass(model);   

	body7 = RigidBodyDynamics::Body(mass[6], com[6], inertia[6]);   
    //print_body(body7);      
	joint7 = RigidBodyDynamics::Joint (     RigidBodyDynamics::JointTypeRevolute,axisDirection[6]);    
	body7_id = model.AddBody(6, RigidBodyDynamics::Math::Xtrans(axisOrigin[6]), joint7, body7, "7");    

    constexpr size_t number_of_max_evals = 10000;
    Eigen::Vector<double,number_of_max_evals> norm_error_computation = Eigen::Vector<double,number_of_max_evals>::Zero();
    Eigen::Vector<double,number_of_max_evals> time_taken_aprox = Eigen::Vector<double,number_of_max_evals>::Zero();
    Eigen::Vector<double,number_of_max_evals> time_taken_noaprox = Eigen::Vector<double,number_of_max_evals>::Zero();
    Eigen::Vector<double,number_of_max_evals> time_taken_to_invert_matrix_with_fixed_size = Eigen::Vector<double,number_of_max_evals>::Zero();
    Eigen::Vector<double,number_of_max_evals> time_taken_to_invert_matrix_with_dynamic_size = Eigen::Vector<double,number_of_max_evals>::Zero();
    
    double time = 0.0;
    for(size_t evals = 0; evals < number_of_max_evals; ++evals,time+=0.001){
        RigidBodyDynamics::Math::VectorNd Q = RigidBodyDynamics::Math::VectorNd::Constant ((size_t) model.dof_count, std::sin(time));
        auto evaluation_data = eval_model_mass(&model,Q);
        norm_error_computation[evals] = std::get<0>(evaluation_data);
        time_taken_aprox[evals] = std::get<1>(evaluation_data);
        time_taken_noaprox[evals] = std::get<2>(evaluation_data);
        time_taken_to_invert_matrix_with_dynamic_size[evals] = std::get<3>(evaluation_data);
        time_taken_to_invert_matrix_with_fixed_size[evals] = std::get<4>(evaluation_data);
    }
    
    {
        auto centered = norm_error_computation;
        auto mean_value = norm_error_computation.mean();
        centered.rowwise() -= Eigen::Matrix<double,1,1>(mean_value);
        std::cout << "average error in norm computation: \n" <<  mean_value << std::endl;
        std::cout << "variance in norm computation: \n" << std::sqrt((centered.transpose()*centered)(0,0)/centered.rows()) << std::endl;
    }

    {
        auto centered = time_taken_aprox;
        auto mean_value = time_taken_aprox.mean();
        centered.rowwise() -= Eigen::Matrix<double,1,1>(mean_value);
        std::cout << "average time with aprox: \n" <<  mean_value << " nanoseconds" << std::endl;
        std::cout << "variance with aprox:\n" << std::sqrt((centered.transpose()*centered)(0,0)/centered.rows()) << std::endl;
    }

    {
        auto centered = time_taken_noaprox;
        auto mean_value = time_taken_noaprox.mean();
        centered.rowwise() -= Eigen::Matrix<double,1,1>(mean_value);
        std::cout << "average time without aprox: \n" <<  mean_value << " nanoseconds" << std::endl;
        std::cout << "variance without aprox: \n" << std::sqrt((centered.transpose()*centered)(0,0)/centered.rows()) << std::endl;
    }

    {
        auto centered = time_taken_to_invert_matrix_with_dynamic_size;
        auto mean_value = time_taken_to_invert_matrix_with_dynamic_size.mean();
        centered.rowwise() -= Eigen::Matrix<double,1,1>(mean_value);
        std::cout << "average time to invert with variable matrix: \n" <<  mean_value << " nanoseconds" << std::endl;
        std::cout << "variance with variable matrix: \n" << std::sqrt((centered.transpose()*centered)(0,0)/centered.rows()) << std::endl;
    }

    {
        auto centered = time_taken_to_invert_matrix_with_fixed_size;
        auto mean_value = time_taken_to_invert_matrix_with_fixed_size.mean();
        centered.rowwise() -= Eigen::Matrix<double,1,1>(mean_value);
        std::cout << "average time to invert with fixed matrix: \n" <<  mean_value << " nanoseconds" << std::endl;
        std::cout << "variance with fixed matrix: \n" << std::sqrt((centered.transpose()*centered)(0,0)/centered.rows()) << std::endl;
    }
}