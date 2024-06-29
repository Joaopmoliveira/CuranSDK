#include "rbdl/Logging.h"
#include "rbdl/Model.h"
#include "rbdl/Dynamics.h"
#include <nlohmann/json.hpp>
#include <chrono>
#include "utils/Reader.h"
#include "robotutils/LBRController.h"
#include <fstream>
#include <sstream>




namespace robotutils{

class Tool{
	double mass;
	RigidBodyDynamics::Math::Vector3d center_of_mass;
	RigidBodyDynamics::Math::Matrix3d inertia;

	
	RigidBodyDynamics::Math::Vector3d axis_origin;
};

template<size_t model_joints>
class KinematicLimits{

    Eigen::Matrix<double,model_joints,1> f_min_q;
    Eigen::Matrix<double,model_joints,1> f_max_q;
    Eigen::Matrix<double,model_joints,1> f_min_dq;
    Eigen::Matrix<double,model_joints,1> f_max_dq;
    Eigen::Matrix<double,model_joints,1> f_min_ddq;
    Eigen::Matrix<double,model_joints,1> f_max_ddq;
public:
    KinematicLimits(std::filesystem::path robot_limits_directory)
    {
        f_min_q = -Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_max_q = Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_min_dq = -Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_max_dq = Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_min_ddq = -Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_max_ddq = Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();

        // Parse mass data from input file
        nlohmann::json table_data = nlohmann::json::parse(std::ifstream(robot_limits_directory));
        if(table_data.size()!=model_joints)
            throw std::runtime_error("the supplied model has a different number of parameters from the compiled model");

        auto iterator = table_data.begin();

        for(size_t ind = 0; ind < model_joints; ++ind,++iterator){
            f_min_q[ind] = (*iterator)["min_q"];       
            f_max_q[ind] = (*iterator)["max_q"];          
            f_min_dq[ind] = (*iterator)["min_dq"]; 
            f_max_dq[ind] = (*iterator)["max_dq"]; 
            f_min_ddq[ind] = (*iterator)["min_ddq"]; 
            f_max_ddq[ind] = (*iterator)["max_ddq"];    
        }
    }

    KinematicLimits()
    {
        f_min_q = -Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_max_q = Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_min_dq = -Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_max_dq = Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_min_ddq = -Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
        f_max_ddq = Eigen::Matrix<double,model_joints,1>::Ones()*std::numeric_limits<double>::max();
    }

    template<size_t model_joints>
    friend std::ostream& operator<<(std::ostream& o, const KinematicLimits<model_joints>& obj){
        std::cout << "qMin: " << obj.f_min_q.transpose() << std::endl; 
        std::cout << "qMax: " << obj.f_max_q.transpose() << std::endl; 
        std::cout << "qDotMin: " << obj.f_min_dq.transpose() << std::endl; 
        std::cout << "qDotMax: " << obj.f_max_dq.transpose() << std::endl; 
        std::cout << "qDotDotMax: " << obj.f_max_ddq.transpose() << std::endl; 
        std::cout << "qDotDotMin: " << obj.f_min_ddq.transpose() << std::endl; 
        return o;
    }

    inline const Eigen::Matrix<double,model_joints,1>& qmax() const {
        return f_max_q;
    }

    inline const Eigen::Matrix<double,model_joints,1>& qmin() const {
        return f_min_q;
    }

    inline const Eigen::Matrix<double,model_joints,1>& dqmax() const {
        return f_max_dq;
    }

    inline const Eigen::Matrix<double,model_joints,1>& dqmin() const {
        return f_min_dq;
    }

    inline const Eigen::Matrix<double,model_joints,1>& ddqmax() const {
        return f_max_ddq;
    }

    inline const Eigen::Matrix<double,model_joints,1>& ddqmin() const {
        return f_min_ddq;
    }

};

template<size_t model_joints>
class RobotModel{
    private:
    // Data describing the robot
    std::array<double,model_joints> f_mass;
    std::array<RigidBodyDynamics::Math::Vector3d,model_joints> f_center_of_mass;
    std::array<RigidBodyDynamics::Math::Matrix3d,model_joints> f_inertia;
    std::array<RigidBodyDynamics::Math::Vector3d,model_joints> f_axis_direction;
    std::array<RigidBodyDynamics::Math::Vector3d,model_joints> f_axis_origin;

    // Parameters for computation of dynamics
	std::array<unsigned int,model_joints> f_body_id;
	std::array<RigidBodyDynamics::Body,model_joints> f_body;   
	std::array<RigidBodyDynamics::Joint,model_joints> f_joint;   
	RigidBodyDynamics::Model f_model{};  

    // Parameters to store current state of the robot
    Eigen::Matrix<double,model_joints,model_joints> f_massmatrix;
    Eigen::Matrix<double,model_joints,model_joints> f_inverse_massmatrix;
    Eigen::Matrix<double,model_joints,1> f_coriolis;
    Eigen::Matrix<double,model_joints,1> f_gravity;
    Eigen::Matrix<double,model_joints,1> f_q;
    Eigen::Matrix<double,model_joints,1> f_dq;
    Eigen::Matrix<double,model_joints,1> f_ddq;
    Eigen::Matrix<double,6,model_joints> f_jacobian;
    Eigen::Matrix<double,4,4> f_end_effector;

    // Variable that signals if the control loop should continue
    std::atomic<bool> f_continue_robot_motion = true;

    //Flange position
    Eigen::Vector3d f_flange_position;

    const KinematicLimits<model_joints> f_kinematic_limits;

    void initialize_all(const std::filesystem::path& models_data_directory){
        // Initialize to zero everything
        f_massmatrix = Eigen::Matrix<double,model_joints,model_joints>::Zero();
        f_coriolis = Eigen::Matrix<double,model_joints,1>::Zero();
        f_gravity = Eigen::Matrix<double,model_joints,1>::Zero();
        f_q = Eigen::Matrix<double,model_joints,1>::Zero();
        f_dq = Eigen::Matrix<double,model_joints,1>::Zero();
        f_ddq = Eigen::Matrix<double,model_joints,1>::Zero();
        f_end_effector = Eigen::Matrix<double,4,4>::Zero();
        f_jacobian = Eigen::Matrix<double,6,model_joints>::Zero();

        // except the electrical flange which we set to this offset
        f_flange_position= Vector3d(0, 0, 0.045); 

        // Parse mass data from input file
        nlohmann::json table_data = nlohmann::json::parse(std::ifstream(models_data_directory));
        if(table_data.size()!=model_joints)
            throw std::runtime_error("the supplied model has a different number of parameters from the compiled model");

        auto iterator = table_data.begin();

        for(size_t ind = 0; ind < model_joints; ++ind,++iterator){
            f_mass[ind] = (*iterator)["mass"];       
            {
                std::string content = (*iterator)["center_of_mass"];
                std::stringstream ss{content};
                f_center_of_mass[ind] = curan::utilities::convert_matrix(ss,',');
            }
            {
                std::string content = (*iterator)["inertia"];
                std::stringstream ss{content};
                f_inertia[ind] = curan::utilities::convert_matrix(ss,',');
            }
            {
                std::string content = (*iterator)["axis_direction"];
                std::stringstream ss{content};
                f_axis_direction[ind] = curan::utilities::convert_matrix(ss,',');
            }
            {
                std::string content = (*iterator)["axis_origin"]; 
                std::stringstream ss{content};
                f_axis_origin[ind] = curan::utilities::convert_matrix(ss,',');
            }
        }

        f_model.gravity = RigidBodyDynamics::Math::Vector3d (0.0, 0.0, -9.81);

        f_body[0] = RigidBodyDynamics::Body(f_mass[0], f_center_of_mass[0], f_inertia[0]);; 
	    f_joint[0] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,f_axis_direction[0]);  
	    f_body_id[0] = f_model.AddBody(0, RigidBodyDynamics::Math::Xtrans(f_axis_origin[0]), f_joint[0], f_body[0], std::to_string(0));


        for(size_t ind = 1; ind < model_joints; ++ind){
            f_body[ind] = RigidBodyDynamics::Body(f_mass[ind], f_center_of_mass[ind], f_inertia[ind]);     
	        f_joint[ind] = RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute,f_axis_direction[ind]);  
	        f_body_id[ind] = f_model.AddBody(f_body_id[ind-1], RigidBodyDynamics::Math::Xtrans(f_axis_origin[ind]), f_joint[ind], f_body[ind], std::to_string(ind));
        }

    }

    public:
    //delete all robot copy operators so that we do not have to deal with nasty copies
    RobotModel(const RobotModel& other) = delete;
    RobotModel(RobotModel&& other) = delete;
    RobotModel& operator=(RobotModel&& other) = delete;
    RobotModel& operator=(const RobotModel& other) = delete; 

    RobotModel(const std::filesystem::path& models_data_directory,
               const std::filesystem::path& kinematic_limits_directory) : f_kinematic_limits{kinematic_limits_directory}{
        initialize_all(models_data_directory);
    }

    RobotModel(const std::filesystem::path& models_data_directory){
        initialize_all(models_data_directory);
    }

    inline operator bool() const {
        return f_continue_robot_motion.load(std::memory_order_relaxed); 
    }

    inline void cancel(){
        f_continue_robot_motion.store(false,std::memory_order_relaxed);
    }

    void update(const curan::robotic::State& in_state){
        f_q = curan::robotic::convert(in_state.q);
        f_dq = curan::robotic::convert(in_state.dq);
        f_ddq = curan::robotic::convert(in_state.ddq);

        {
            static RigidBodyDynamics::Math::VectorNd rbdl_q = RigidBodyDynamics::Math::VectorNd::Zero(model_joints);
            static RigidBodyDynamics::Math::VectorNd rbdl_dq = RigidBodyDynamics::Math::VectorNd::Zero(model_joints);
            static RigidBodyDynamics::Math::VectorNd rbdl_ddq = RigidBodyDynamics::Math::VectorNd::Zero(model_joints);
            rbdl_q = f_q;
            rbdl_dq = f_dq;
            rbdl_ddq = f_ddq;
            RigidBodyDynamics::UpdateKinematics(f_model,rbdl_q,rbdl_dq,rbdl_ddq);
        }
        

        // now we want the kinematic data 
        f_end_effector.block<3,1>(0,3) = RigidBodyDynamics::CalcBodyToBaseCoordinates(f_model, f_q , f_body_id[model_joints-1], f_flange_position, false);
        f_end_effector.block<3,3>(0,0) = RigidBodyDynamics::CalcBodyWorldOrientation(f_model, f_q, f_body_id[model_joints-1], false).transpose();
	    
        {
            RigidBodyDynamics::Math::MatrixNd J0 = RigidBodyDynamics::Math::MatrixNd::Zero(6, model_joints);
            RigidBodyDynamics::Math::MatrixNd JP = RigidBodyDynamics::Math::MatrixNd::Zero(6, model_joints);
        
            RigidBodyDynamics::CalcBodySpatialJacobian(f_model, f_q, f_body_id[model_joints-1], J0, false);

            RigidBodyDynamics::Math::MatrixNd rotate_jacobian = RigidBodyDynamics::Math::MatrixNd::Identity(6,6);
            RigidBodyDynamics::Math::MatrixNd translate_jacobian = RigidBodyDynamics::Math::MatrixNd::Identity(6,6);

            RigidBodyDynamics::Math::Vector3d r = f_flange_position;
        
            rotate_jacobian.block(0,0,3,3) = f_end_effector.block<3,3>(0,0);
            rotate_jacobian.block(3,3,3,3) = f_end_effector.block<3,3>(0,0);
        

            translate_jacobian(4,0) = -r[2];
            translate_jacobian(5,0) = +r[1];
            translate_jacobian(3,1) = +r[2];
            translate_jacobian(5,1) = -r[0];
            translate_jacobian(3,2) = -r[1];
            translate_jacobian(4,2) = +r[0];
        

            JP = rotate_jacobian*translate_jacobian*J0;
            f_jacobian.block(0,0,3,model_joints) = JP.block(3,0,3, model_joints);
            f_jacobian.block(3,0,3,model_joints) = JP.block(0,0,3, model_joints);
        }

        
        {
            static RigidBodyDynamics::Math::MatrixNd mass_wrapper = RigidBodyDynamics::Math::MatrixNd::Zero(model_joints,model_joints);
            //according to RBDL we need to zero out the mass matrix 
            mass_wrapper.setZero();
            RigidBodyDynamics::CompositeRigidBodyAlgorithm(f_model, f_q, mass_wrapper, true);
            f_massmatrix = mass_wrapper;
        }

        
        {
            // compute the gravity dependent terms
            static RigidBodyDynamics::Math::VectorNd gravity_wrapper = RigidBodyDynamics::Math::VectorNd::Zero(model_joints);
            RigidBodyDynamics::InverseDynamics(f_model, f_q, RigidBodyDynamics::Math::VectorNd::Zero(model_joints), RigidBodyDynamics::Math::VectorNd::Zero(model_joints), gravity_wrapper);
            f_gravity = gravity_wrapper;
        }
                
        
        {
            // compute terms dependent on the velocity as well, e.g. Centrifugal, Coriolis, Gravity
            static RigidBodyDynamics::Math::VectorNd full_nonlinear_terms = RigidBodyDynamics::Math::VectorNd::Zero(model_joints);
            RigidBodyDynamics::InverseDynamics(f_model, f_q, f_dq, RigidBodyDynamics::Math::VectorNd::Zero(model_joints), full_nonlinear_terms);
            f_coriolis = full_nonlinear_terms-f_gravity;
        }

        f_inverse_massmatrix = f_massmatrix.inverse();
    }

    inline const Eigen::Matrix<double,6,model_joints>& jacobian() const {
        return f_jacobian;
    }

    inline const Eigen::Matrix<double,model_joints,model_joints>& mass() const {
        return f_massmatrix;
    }

    inline const Eigen::Matrix<double,model_joints,model_joints>& invmass() const {
        return f_inverse_massmatrix;
    }

    inline const Eigen::Matrix<double,model_joints,1>& joints() const {
        return f_q;
    }

    inline const Eigen::Matrix<double,model_joints,1>& velocities() const {
        return f_dq;
    }

    inline const Eigen::Matrix<double,model_joints,1>& accelerations() const {
        return f_ddq;
    }

    inline Eigen::Matrix<double,3,1> translation() const {
        return f_end_effector.block<3,1>(0,3);
    }

    inline Eigen::Matrix<double,3,3> rotation() const {
        return f_end_effector.block<3,3>(0,0);
    }

    inline const KinematicLimits<model_joints>& kinematic_limits() const {
        return f_kinematic_limits;
    }

};

    /*
    The function preallocates all the memory once, the first time it is ran, then the memory will never need to be reallocated
    because it already exists, plus it all packed together, being cache friendly. 
    */

   constexpr double deg2rad(double in){
        return in*M_PI / 180.0;
   };

   constexpr double rad2deg(double in){
        return in* 180.0/M_PI;
   };

    template<size_t number_of_joints>
    Eigen::Matrix<double,number_of_joints,1> add_constraints(const RobotModel<number_of_joints>& model,const Eigen::Matrix<double,number_of_joints,1>& tauStack, const double& dt){
        static Eigen::Matrix<double,number_of_joints,1> dt2 = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> dtvar = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDownBar = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qTopBar = Eigen::Matrix<double,number_of_joints,1>::Zero();

        static Eigen::Matrix<double,number_of_joints,1> qDotMaxFromQ = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDotMinFromQ = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDotMaxFormQDotDot = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDotMinFormQDotDot = Eigen::Matrix<double,number_of_joints,1>::Zero();

        static Eigen::Matrix<double,3,1> vMaxVector = Eigen::Matrix<double,3,1>::Zero(3);
        static Eigen::Matrix<double,3,1> vMinVector = Eigen::Matrix<double,3,1>::Zero(3);

        static Eigen::Matrix<double,number_of_joints,1> qDotMaxFinal = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDotMinFinal =Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> aMaxqDot = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> aMinqDot = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> aMaxQ = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> aMinQ = Eigen::Matrix<double,number_of_joints,1>::Zero();

        static Eigen::Matrix<double,3,1> aMaxVector = Eigen::Matrix<double,3,1>::Zero(3);
        static Eigen::Matrix<double,3,1> aMinVector = Eigen::Matrix<double,3,1>::Zero(3);

        static Eigen::Matrix<double,number_of_joints,1> qDotDotMaxFinal = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDotDotMinFinal = Eigen::Matrix<double,number_of_joints,1>::Zero();

        static Eigen::Matrix<double,number_of_joints,number_of_joints> Iden = Eigen::Matrix<double,number_of_joints,number_of_joints>::Identity();

        static Eigen::Matrix<double,number_of_joints,1> TauBar = Eigen::Matrix<double,number_of_joints,1>::Zero();
        static Eigen::Matrix<double,number_of_joints,1> qDotDotGot = Eigen::Matrix<double,number_of_joints,1>::Zero();
       

        constexpr double lowestdtFactor = 10.0;

        qDownBar = model.joints() - model.kinematic_limits().qmin();
        qTopBar = model.kinematic_limits().qmax() - model.joints();
        dtvar[0] = 3 * dt;
        dtvar[1] = 3 * dt;
        dtvar[2] = 2 * dt;
        dtvar[3] = 3 * dt;
        dtvar[4] = dt;
        dtvar[5] = dt;
        dtvar[6] = dt;

        for (Eigen::Index i = 0; i < number_of_joints; ++i){
            dt2[i] = dtvar[i];
            //limit the lowest value that qDownBar and qTopBar can have
            qDownBar[i] = std::max(qDownBar[i],0.0);
            qTopBar[i] = std::max(qTopBar[i],0.0);
            
            // recompute the delta time to reach the boundary condition
            dt2[i] = (qDownBar[i] < deg2rad(10.0)) ? (lowestdtFactor + std::sqrt(lowestdtFactor) * std::sqrt(rad2deg(qDownBar[i]))) * dtvar[i] : dt2[i];
            dt2[i] = (qTopBar[i] < deg2rad(10.0)) ? (lowestdtFactor + std::sqrt(lowestdtFactor) * std::sqrt(rad2deg(qTopBar[i]))) * dtvar[i] : dt2[i];
            
            // impose a lower bound on this delta time
            dt2[i] = ( ((qDownBar[i] < deg2rad(10) ) || (qTopBar[i] < deg2rad(10) )) && (dt2[i] < lowestdtFactor * dtvar[i])) ? lowestdtFactor * dtvar[i] : dt2[i];

            // compute maximum velocity given the boundary condition
            qDotMaxFromQ[i] = (model.kinematic_limits().qmax()[i] - model.joints()[i]) / dt2[i];
            qDotMinFromQ[i] = (model.kinematic_limits().qmin()[i] - model.joints()[i]) / dt2[i];

            // compute maximum acceleration given this geometric constraint
            qDotMaxFormQDotDot[i] = std::sqrt(2.0 * model.kinematic_limits().ddqmax()[i] * (model.kinematic_limits().qmax()[i] - model.joints()[i]));
            qDotMaxFormQDotDot[i] = (model.kinematic_limits().qmax()[i] - model.joints()[i] < 0.0) ? 1000000.0 : qDotMaxFormQDotDot[i];

            // compute maximum acceleration given this geometric constraint
            qDotMinFormQDotDot[i] = -std::sqrt(2.0 * model.kinematic_limits().ddqmax()[i] * (model.joints()[i] - model.kinematic_limits().qmin()[i]));
            qDotMinFormQDotDot[i] = (model.joints()[i] - model.kinematic_limits().qmin()[i] < 0.0) ?-1000000.0 : qDotMinFormQDotDot[i];
                
            
            vMaxVector = Vector3d(model.kinematic_limits().dqmax()[i], qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
            qDotMaxFinal[i] = vMaxVector.minCoeff();
            vMinVector = Vector3d(model.kinematic_limits().dqmin()[i], qDotMinFromQ[i], qDotMinFormQDotDot[i]);
            qDotMinFinal[i] = vMinVector.maxCoeff();

            aMaxqDot[i] = (qDotMaxFinal[i] - model.velocities()[i]) / dtvar[i];
            aMinqDot[i] = (qDotMinFinal[i] - model.velocities()[i]) / dtvar[i];
            
            aMaxQ[i] = 2.0 * (model.kinematic_limits().qmax()[i] - model.joints()[i] - model.velocities()[i] * dt2[i]) / std::pow(dt2[i], 2);
            aMinQ[i] = 2.0 * (model.kinematic_limits().qmin()[i] - model.joints()[i] - model.velocities()[i] * dt2[i]) / std::pow(dt2[i], 2);

            aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000.0);
            qDotDotMaxFinal[i] = aMaxVector.minCoeff();
            aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000.0);
            qDotDotMinFinal[i] = aMinVector.maxCoeff();

            if (qDotDotMaxFinal[i] < qDotDotMinFinal[i])
            {
                vMaxVector = Vector3d(std::numeric_limits<double>::max(), qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
                qDotMaxFinal[i] = vMaxVector.minCoeff();

                vMinVector = Vector3d(-std::numeric_limits<double>::max(), qDotMinFromQ[i], qDotMinFormQDotDot[i]);
                qDotMinFinal[i] = vMinVector.maxCoeff();

                aMaxqDot[i] = (qDotMaxFinal[i] - model.velocities()[i]) / dtvar[i];
                aMinqDot[i] = (qDotMinFinal[i] - model.velocities()[i]) / dtvar[i];

                aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000.0);
                qDotDotMaxFinal[i] = aMaxVector.minCoeff();
                aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000.0);
                qDotDotMinFinal[i] = aMinVector.maxCoeff();
            }
        }

        Eigen::Matrix<double,number_of_joints,1> qDotDotS = Eigen::Matrix<double,number_of_joints,1>::Zero();
        Eigen::Matrix<double,number_of_joints,1> tauS = Eigen::Matrix<double,number_of_joints,1>::Zero();
        Eigen::Matrix<double,number_of_joints,number_of_joints> Psat = Iden;
        bool violation_detected = true;
        bool CreateTaskSat = false;
        Eigen::Index NumSatJoints = 0;
        Eigen::Vector<Eigen::Index,Eigen::Dynamic> theMostCriticalOld = Eigen::Vector<Eigen::Index,Eigen::Dynamic>::Zero(number_of_joints);
        theMostCriticalOld.conservativeResize(1);
        theMostCriticalOld[0] = 100;
        bool isThere = false;
        int iO = 0;
        int cycle = 0;
        Eigen::Matrix<double,Eigen::Dynamic,number_of_joints> Js =Eigen::Matrix<double,Eigen::Dynamic,number_of_joints>::Zero(3,number_of_joints); 
        while (violation_detected){
            violation_detected = false;
            if (CreateTaskSat){
                Js.conservativeResize(NumSatJoints, number_of_joints);
                for (Eigen::Index i = 0; i < NumSatJoints; i++){
                    for (Eigen::Index k = 0; k < number_of_joints; k++)
                            Js(i, k) = 0;
                    Js(i,theMostCriticalOld[i]) = 1;
                }
                Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> LambdaSatInv = Js * model.invmass() * Js.transpose();
                Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> LambdaSatInv_aux = LambdaSatInv * LambdaSatInv.transpose();
                Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> LambdaSat_aux = LambdaSatInv_aux.inverse();
                Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> LambdaSat = LambdaSatInv.transpose() * LambdaSat_aux;

                Eigen::Matrix<double,number_of_joints,Eigen::Dynamic> JsatBar = model.invmass() * Js.transpose() * LambdaSat;
                Psat = Iden - Js.transpose() * JsatBar.transpose();
                VectorNd xDotDot_s = Js * qDotDotS;
                tauS = Js.transpose() * (LambdaSat * xDotDot_s);
            }

            TauBar = tauS + Psat * tauStack;
            qDotDotGot = model.invmass() * (TauBar);
            isThere = false;
            for (Eigen::Index i = 0; i < number_of_joints; ++i){
                if ((qDotDotMaxFinal[i] + 0.001 < qDotDotGot[i]) || (qDotDotGot[i] < qDotDotMinFinal[i] - 0.001)){
                    violation_detected = true;
                    CreateTaskSat = true;
                    for (Eigen::Index k = 0; k < theMostCriticalOld.size(); ++k)
                        if (i == theMostCriticalOld[k])
                            isThere = true;
                    if (!isThere){
                        theMostCriticalOld.conservativeResize(iO + 1);
                        theMostCriticalOld[iO] = i;
                        iO += 1;
                    }
                }
            }

            if (violation_detected){
                NumSatJoints = iO;
                theMostCriticalOld.conservativeResize(iO);
                cycle += 1;
                if (cycle > 8)
                    violation_detected = false;

                for (size_t i = 0; i < theMostCriticalOld.size(); ++i){
                    Eigen::Index jM = theMostCriticalOld[i];
                    if (qDotDotGot[jM] > qDotDotMaxFinal[jM])
                        qDotDotS[jM] = qDotDotMaxFinal[jM];

                    if (qDotDotGot[jM] < qDotDotMinFinal[jM])
                        qDotDotS[jM] = qDotDotMinFinal[jM];
                }
            }
        }
        return TauBar;
    }
}

struct RobotLimits
{
    VectorNd qMin;
    VectorNd qMax;
    VectorNd qDotMin;
    VectorNd qDotMax;
    VectorNd qDotDotMax;
    VectorNd qDotDotMin;

    RobotLimits()
    {
        qMin = VectorNd::Zero(7, 1);
        qMax = VectorNd::Zero(7, 1);
        qDotMin = VectorNd::Zero(7, 1);
        qDotMax = VectorNd::Zero(7, 1);
        qDotDotMax = VectorNd::Zero(7, 1);
        qDotDotMin = VectorNd::Zero(7, 1);

    }


    friend std::ostream& operator<<(std::ostream& o,const RobotLimits& limits){
        std::cout << "qMin: " << limits.qMin.transpose() << std::endl; 
        std::cout << "qMax: " << limits.qMax.transpose() << std::endl; 
        std::cout << "qDotMin: " << limits.qDotMin.transpose() << std::endl; 
        std::cout << "qDotMax: " << limits.qDotMax.transpose() << std::endl; 
        std::cout << "qDotDotMax: " << limits.qDotDotMax.transpose() << std::endl; 
        std::cout << "qDotDotMin: " << limits.qDotDotMin.transpose() << std::endl;
        return o; 
    }

};

VectorNd addConstraints(RobotParameters* iiwa,RobotLimits& myIIWALimits,const VectorNd& tauStack, double dt)
{
    VectorNd dt2 = VectorNd::Zero(7, 1);
    VectorNd dtvar = VectorNd::Zero(7, 1);
    VectorNd qDownBar = VectorNd::Zero(7, 1);
    VectorNd qTopBar = VectorNd::Zero(7, 1);

    VectorNd qDotMaxFromQ = VectorNd::Zero(7, 1);
    VectorNd qDotMinFromQ = VectorNd::Zero(7, 1);
    VectorNd qDotMaxFormQDotDot = VectorNd::Zero(7, 1);
    VectorNd qDotMinFormQDotDot = VectorNd::Zero(7, 1);
    VectorNd vMaxVector = Vector3d::Zero(3);
    VectorNd vMinVector = Vector3d::Zero(3);
    VectorNd qDotMaxFinal = VectorNd::Zero(7, 1);
    VectorNd qDotMinFinal = VectorNd::Zero(7, 1);
    VectorNd aMaxqDot = VectorNd::Zero(7, 1);
    VectorNd aMinqDot = VectorNd::Zero(7, 1);
    VectorNd aMaxQ = VectorNd::Zero(7, 1);
    VectorNd aMinQ = VectorNd::Zero(7, 1);
    VectorNd aMaxVector = Vector3d::Zero(3);
    VectorNd aMinVector = Vector3d::Zero(3);
    VectorNd qDotDotMaxFinal = VectorNd::Zero(7, 1);
    VectorNd qDotDotMinFinal = VectorNd::Zero(7, 1);
    MatrixNd Iden = MatrixNd::Identity(7, 7);
    VectorNd TauBar = VectorNd::Zero(7, 1);
    VectorNd qDotDotGot = VectorNd::Zero(7, 1);
    MatrixNd Js = MatrixNd::Zero(3, 7);

    double lowestdtFactor = 10;

    qDownBar = iiwa->q - myIIWALimits.qMin;
    qTopBar = myIIWALimits.qMax - iiwa->q;
    dtvar[0] = 3 * dt;
    dtvar[1] = 3 * dt;
    dtvar[2] = 2 * dt;
    dtvar[3] = 3 * dt;
    dtvar[4] = dt;
    dtvar[5] = dt;
    dtvar[6] = dt;

    for (int i = 0; i < 7; i++)
    {
        dt2[i] = dtvar[i];
        if (qDownBar[i] < 10 * M_PI / 180)
        {

            if (qDownBar[i] < 0)
                qDownBar[i] = 0;

            dt2[i] = ((lowestdtFactor) + (sqrt(lowestdtFactor) * sqrt(qDownBar[i] * 180 / M_PI))) * dtvar[i];
            if (dt2[i] < lowestdtFactor * dtvar[i])
                dt2[i] = lowestdtFactor * dtvar[i];
        }
        if (qTopBar[i] < 10 * M_PI / 180)
        {

            if (qTopBar[i] < 0)
                qTopBar[i] = 0;
            dt2[i] = (lowestdtFactor + (sqrt(lowestdtFactor) * sqrt(qTopBar[i] * 180 / M_PI))) * dtvar[i];
            if (dt2[i] < lowestdtFactor * dtvar[i])
                dt2[i] = lowestdtFactor * dtvar[i];

        }

        qDotMaxFromQ[i] = (myIIWALimits.qMax[i] - iiwa->q[i]) / dt2[i];
        qDotMinFromQ[i] = (myIIWALimits.qMin[i] - iiwa->q[i]) / dt2[i];
        qDotMaxFormQDotDot[i] = sqrt(2 * myIIWALimits.qDotDotMax[i] * (myIIWALimits.qMax[i] - iiwa->q[i]));
        qDotMinFormQDotDot[i] = -sqrt(2 * myIIWALimits.qDotDotMax[i] * (iiwa->q[i] - myIIWALimits.qMin[i]));

        if (myIIWALimits.qMax[i] - iiwa->q[i] < 0)
            qDotMaxFormQDotDot[i] = 1000000;

        if (iiwa->q[i] - myIIWALimits.qMin[i] < 0)
            qDotMinFormQDotDot[i] = -1000000;

        vMaxVector = Vector3d(myIIWALimits.qDotMax[i], qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
        qDotMaxFinal[i] = vMaxVector.minCoeff();


        vMinVector = Vector3d(myIIWALimits.qDotMin[i], qDotMinFromQ[i], qDotMinFormQDotDot[i]);
        qDotMinFinal[i] = vMinVector.maxCoeff();

        aMaxqDot[i] = (qDotMaxFinal[i] - iiwa->qDot[i]) / dtvar[i];
        aMinqDot[i] = (qDotMinFinal[i] - iiwa->qDot[i]) / dtvar[i];

        aMaxQ[i] = 2 * (myIIWALimits.qMax[i] - iiwa->q[i] - iiwa->qDot[i] * dt2[i]) / pow(dt2[i], 2);
        aMinQ[i] = 2 * (myIIWALimits.qMin[i] - iiwa->q[i] - iiwa->qDot[i] * dt2[i]) / pow(dt2[i], 2);

        aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000);
        qDotDotMaxFinal[i] = aMaxVector.minCoeff();
        aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000);
        qDotDotMinFinal[i] = aMinVector.maxCoeff();

        if (qDotDotMaxFinal[i] < qDotDotMinFinal[i])
        {
            vMaxVector = Vector3d(INFINITY, qDotMaxFromQ[i], qDotMaxFormQDotDot[i]);
            qDotMaxFinal[i] = vMaxVector.minCoeff();

            vMinVector = Vector3d(-INFINITY, qDotMinFromQ[i], qDotMinFormQDotDot[i]);
            qDotMinFinal[i] = vMinVector.maxCoeff();

            aMaxqDot[i] = (qDotMaxFinal[i] - iiwa->qDot[i]) / dtvar[i];
            aMinqDot[i] = (qDotMinFinal[i] - iiwa->qDot[i]) / dtvar[i];

            aMaxVector = Vector3d(aMaxQ[i], aMaxqDot[i], 10000000);
            qDotDotMaxFinal[i] = aMaxVector.minCoeff();
            aMinVector = Vector3d(aMinQ[i], aMinqDot[i], -10000000);
            qDotDotMinFinal[i] = aMinVector.maxCoeff();
        }
    }


    VectorNd qDotDotS = VectorNd::Zero(7);
    VectorNd tauS = VectorNd::Zero(7);
    MatrixNd Psat = Iden;
    bool LimitedExceeded = true;
    bool CreateTaskSat = false;
    int NumSatJoints = 0;
    Eigen::Vector<Eigen::Index,Eigen::Dynamic> theMostCriticalOld = Eigen::Vector<Eigen::Index,Eigen::Dynamic>::Zero(7);
    theMostCriticalOld.conservativeResize(1);
    theMostCriticalOld[0] = 100;
    bool isThere = false;
    int iO = 0;
    int cycle = 0;
    while (LimitedExceeded)
    {
        LimitedExceeded = false;
        if (CreateTaskSat)
        {
            Js.conservativeResize(NumSatJoints, 7);
            for (int i = 0; i < NumSatJoints; i++)
            {
                
                for (int k = 0; k < 7; k++)
                {
                    Js(i, k) = 0;
                }
                Js(i, (int)theMostCriticalOld[i]) = 1;
            }

            MatrixNd LambdaSatInv = Js * iiwa->Minv * Js.transpose();
            MatrixNd LambdaSatInv_aux = LambdaSatInv * LambdaSatInv.transpose();
            MatrixNd LambdaSat_aux = LambdaSatInv_aux.inverse();
            MatrixNd LambdaSat = LambdaSatInv.transpose() * LambdaSat_aux;

            MatrixNd JsatBar = iiwa->Minv * Js.transpose() * LambdaSat;
            Psat = Iden - Js.transpose() * JsatBar.transpose();
            VectorNd xDotDot_s = Js * qDotDotS;
            tauS = Js.transpose() * (LambdaSat * xDotDot_s);
        }

        TauBar = tauS + Psat * tauStack;
        qDotDotGot = iiwa->Minv * (TauBar); // it should -g -c

        isThere = false;
        for (int i = 0; i < 7; i++)
        {
            if ((qDotDotMaxFinal[i] + 0.001 < qDotDotGot[i]) || (qDotDotGot[i] < qDotDotMinFinal[i] - 0.001))
            {
                LimitedExceeded = true;
                CreateTaskSat = true;

                for (int k = 0; k < theMostCriticalOld.size(); k++)
                {
                    if (i == theMostCriticalOld[k])
                    {
                        isThere = true;
                    }
                }
                if (isThere == false)
                {

                    theMostCriticalOld.conservativeResize(iO + 1);
                    theMostCriticalOld[iO] = i;
                    iO += 1;
                }
            }
        }

        if (LimitedExceeded == true)
        {
            NumSatJoints = iO;
            theMostCriticalOld.conservativeResize(iO);
            cycle += 1;
            if (cycle > 8)
                LimitedExceeded = false;

            for (int i = 0; i < theMostCriticalOld.size(); i++)
            {
                Eigen::Index jM = theMostCriticalOld[i];

                if (qDotDotGot[jM] > qDotDotMaxFinal[jM])
                    qDotDotS[jM] = qDotDotMaxFinal[jM];

                if (qDotDotGot[jM] < qDotDotMinFinal[jM])
                    qDotDotS[jM] = qDotDotMinFinal[jM];
            }
        }
    }
    VectorNd SJSTorque = TauBar;
    return SJSTorque;
};

auto predict_mass_classic(RobotParameters* iiwa){
    return iiwa->M;
}

auto predict_mass_mine(robotutils::RobotModel<7>& robot){
    return robot.mass();
}

auto predict_actuation_torque_classic(curan::robotic::State& current_state, RobotParameters* iiwa,RobotLimits& myIIWALimits){
    return addConstraints(iiwa,myIIWALimits,curan::robotic::convert(current_state.cmd_tau), 0.005);
}

auto predict_actuation_torque_mine(curan::robotic::State& current_state,robotutils::RobotModel<7>& robot){
    return robotutils::add_constraints<7>(robot,curan::robotic::convert(current_state.cmd_tau), 0.005);
}

std::tuple<double,double> compare_predictions(curan::robotic::State state,robotutils::RobotModel<7>& robot,RobotParameters* iiwa,kuka::Robot* robot_ptr,RobotLimits& myIIWALimits){
    static curan::robotic::State current_state;
    static Vector3d pointPosition = Vector3d(0, 0, 0.045); 
    current_state.differential(state);

    current_state.update_iiwa(iiwa,robot_ptr,pointPosition);
    robot.update(current_state);
    auto mass_from_iiwa = predict_mass_classic(iiwa);
    auto mass_mine = predict_mass_mine(robot);
    //std::cout << "mass 1: " << mass_from_iiwa << std::endl;
    //std::cout << "mass 2: " << mass_mine << std::endl;
    //std::cout << "difference: " << mass_from_iiwa-mass_mine << std::endl;
    auto difference_masses = (mass_from_iiwa-mass_mine).squaredNorm();
    auto prediction = predict_actuation_torque_classic(current_state,iiwa,myIIWALimits);
    auto prediction2 = predict_actuation_torque_mine(current_state,robot);
    //std::cout << "desired torque: " << curan::robotic::convert(current_state.cmd_tau).transpose() << std::endl;
    //std::cout << "fixed torque 1: " << prediction.transpose() << std::endl;
    //std::cout << "fixed torque 2: " << prediction2.transpose() << std::endl;
    Eigen::VectorXd prediction3 = Eigen::VectorXd::Zero(7);
    for(size_t index = 0; index < 7; ++index)
        prediction3[index] = prediction2[index];
    auto difference_torques = (prediction-prediction3).squaredNorm()/prediction.squaredNorm();
    return {difference_masses,difference_torques};

}

int main(){
    kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);
    auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
    auto iiwa = std::make_unique<RobotParameters>();
    robotutils::RobotModel<7> robot_model{"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
    auto myIIWALimits = RobotLimits();
    myIIWALimits.qMax[0] = 163 * M_PI / 180;
    myIIWALimits.qMax[1] = 113 * M_PI / 180;
    myIIWALimits.qMax[2] = 163 * M_PI / 180;
    myIIWALimits.qMax[3] = 115 * M_PI / 180;
    myIIWALimits.qMax[4] = 160 * M_PI / 180;
    myIIWALimits.qMax[5] = 110 * M_PI / 180;
    myIIWALimits.qMax[6] = 165 * M_PI / 180;

    myIIWALimits.qMin[0] = -163 * M_PI / 180;
    myIIWALimits.qMin[1] = -113 * M_PI / 180;
    myIIWALimits.qMin[2] = -163 * M_PI / 180;
    myIIWALimits.qMin[3] = -115 * M_PI / 180;
    myIIWALimits.qMin[4] = -160 * M_PI / 180;
    myIIWALimits.qMin[5] = -110 * M_PI / 180;
    myIIWALimits.qMin[6] = -165 * M_PI / 180;

    myIIWALimits.qDotMax[0] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[1] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[2] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[3] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[4] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[5] = 150 * M_PI / 180;
    myIIWALimits.qDotMax[6] = 155 * M_PI / 180;

    myIIWALimits.qDotMin[0] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[1] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[2] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[3] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[4] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[5] = -150 * M_PI / 180;
    myIIWALimits.qDotMin[6] = -155 * M_PI / 180;

    myIIWALimits.qDotDotMax[0] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[1] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[2] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[3] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[4] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[5] = 300 * M_PI / 180;
    myIIWALimits.qDotDotMax[6] = 300 * M_PI / 180;

    myIIWALimits.qDotDotMin[0] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[1] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[2] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[3] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[4] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[5] = -300 * M_PI / 180;
    myIIWALimits.qDotDotMin[6] = -300 * M_PI / 180;

    std::cout << "myIIWALimits:\n" << myIIWALimits <<  std::endl;

    std::cout << "mine limits:\n" << robot_model.kinematic_limits() << std::endl;

    double time = 0.1;
    curan::robotic::State state;
    state.q = std::array<double,7>{};
    state.dq = std::array<double,7>{};
    state.ddq = std::array<double,7>{};
    std::list<std::tuple<double,double>> list_of_comparisons;
    for(size_t iter = 0 ; iter < 100000; ++iter,time+=0.1){
        for(size_t index = 0; index < 7; ++index){
            state.q[index] = 3*std::sin(time);
            state.cmd_tau[index] = 8*std::cos(time);
        }
        list_of_comparisons.push_back(compare_predictions(state,robot_model,iiwa.get(),robot.get(),myIIWALimits));
    }
    std::tuple<double,double,size_t> accumulator = std::make_tuple(0,0,0);
    double max = -std::numeric_limits<double>::max();
    size_t index = 0;
    size_t max_index = 0;
    //double min = std::numeric_limits<double>::max();
    for(const auto & val : list_of_comparisons){
        if(max<std::get<1>(val)){
            max = std::get<1>(val);
            max_index = index;
        }
        accumulator = std::make_tuple(std::get<0>(accumulator)+std::get<0>(val),std::get<1>(accumulator)+std::get<1>(val),index);
        ++index;
    }
    auto final_mean = std::make_tuple(std::get<0>(accumulator)/list_of_comparisons.size(),std::get<1>(accumulator)/list_of_comparisons.size());
    std::cout << "difference: \nmass: " << std::get<0>(final_mean) << "\ntorque: " << std::get<1>(final_mean) << "max difference: "<< max << std::endl << "max index: " << max_index << std::endl;
    return 0;
}

int main2(){
    try{
        curan::robotic::State state;
        state.q = std::array<double,7>{};
        robotutils::RobotModel<7> robot_model{"C:/Dev/Curan/resources/models/lbrmed/robot_mass_data.json","C:/Dev/Curan/resources/models/lbrmed/robot_kinematic_limits.json"};
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        robot_model.update(state);
        auto computed_torque = robotutils::add_constraints<7>(robot_model,Eigen::Matrix<double,7,1>::Ones(),0.005);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        auto first_round = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        begin = std::chrono::steady_clock::now();
        computed_torque = robotutils::add_constraints<7>(robot_model,Eigen::Matrix<double,7,1>::Ones(),0.002);
        end = std::chrono::steady_clock::now();

        auto second_round = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        begin = std::chrono::steady_clock::now();
        computed_torque = robotutils::add_constraints<7>(robot_model,Eigen::Matrix<double,7,1>::Ones(),0.001);
        end = std::chrono::steady_clock::now();

        auto third_round = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        std::cout << "joint: " << robot_model.joints().transpose() << std::endl;
        std::cout << "vel: " << robot_model.velocities().transpose() << std::endl;
        std::cout << "accel: " << robot_model.accelerations().transpose() << std::endl;
        std::cout << "translation: " << robot_model.translation().transpose() << std::endl;
        std::cout << "mass: " << robot_model.mass() << std::endl;
        std::cout << "time taken (first round): " << first_round << std::endl;
        std::cout << "time taken (second round): " << second_round << std::endl;
        std::cout << "time taken (third round): " << third_round << std::endl;
        //std::cout << "torque constrained: " << computed_torque.transpose() << std::endl;
    } catch(std::runtime_error& e){
        std::cout << "exception thrown : " << e.what() << std::endl;
        return 1; 
    } catch(...){
        std::cout << "unknown exception thrown" << std::endl;
        return 2;
    }
    return 0;
}





/*
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
    
    RigidBodyDynamics::UpdateKinematics(*model,Q,QDot,QDDot);
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


int twomain(){
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


    return 0;
}
*/