#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"
#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/SafeQueue.h"

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

constexpr unsigned short DEFAULT_PORTID = 30200;


void robot_control(std::shared_ptr<SharedState> shared_state,std::shared_ptr<curan::utilities::Flag> flag) {
	try{
	curan::utilities::cout << "Lauching robot control thread\n";
	MyLBRClient client = MyLBRClient(shared_state);
	KUKA::FRI::UdpConnection connection;
	KUKA::FRI::ClientApplication app(connection, client);
	app.connect(DEFAULT_PORTID, NULL);
	bool success = app.step();
	if(success)
		curan::utilities::cout << "Established first message between robot and host\n";
	else
		curan::utilities::cout << "Failed to established first message between robot and host\n";
	try
	{
		while (success && flag->value())
		{
			success = app.step();
		}
	}
	catch (std::exception& e) {
		return;
	}
	app.disconnect();
	return;
	} catch(...){
		std::cout << "robot control exception\n";
		return;
	}
}

void GetRobotConfiguration(igtl::Matrix4x4& matrix, kuka::Robot* robot, RobotParameters* iiwa,std::shared_ptr<SharedState> shared_state)
{
	static auto t1 = std::chrono::steady_clock::now();
	static double _qOld[NUMBER_OF_JOINTS];
	bool is_initialized = shared_state->is_initialized.load();
	if(is_initialized){
		auto robot_state = shared_state->robot_state.load();
		auto _qCurr = robot_state.getMeasuredJointPosition();
		memcpy(_qOld, _qCurr, NUMBER_OF_JOINTS * sizeof(double));
		//curan::utils::cout << "the joints are: \n";
		for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        	iiwa->q[i] = _qCurr[i];
    	}
		auto t2 = std::chrono::steady_clock::now();
		auto sampleTimeChrono = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);
		double sampleTime = (sampleTimeChrono.count()< 1) ? 0.001 : sampleTimeChrono.count()/1000.0;	
		t1 = t2;
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
	        iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
    	}
		//curan::utils::cout << "\n";
	
		static Vector3d p_0_cur = Vector3d::Zero(3, 1);
		static Matrix3d R_0_7 = Matrix3d::Zero(3, 3);
		static Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric

	    robot->getMassMatrix(iiwa->M, iiwa->q);
	    iiwa->M(6, 6) = 45 * iiwa->M(6, 6);                                       // Correct mass of last body to avoid large accelerations
	    iiwa->Minv = iiwa->M.inverse();
	    robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
	    robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7);              // 3x1 position of flange (body = 7), expressed in base coordinates
	    robot->getRotationMatrix(R_0_7, iiwa->q, NUMBER_OF_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates
	

		p_0_cur *= 1000; 
		matrix[0][0] = R_0_7(0, 0);
		matrix[1][0] = R_0_7(1, 0);
		matrix[2][0] = R_0_7(2, 0);

		matrix[0][1] = R_0_7(0, 1);
		matrix[1][1] = R_0_7(1, 1);
		matrix[2][1] = R_0_7(2, 1);

		matrix[0][2] = R_0_7(0, 2);
		matrix[1][2] = R_0_7(1, 2);
		matrix[2][2] = R_0_7(2, 2);

		matrix[3][0] = 0.0;
		matrix[3][1] = 0.0;
		matrix[3][2] = 0.0;
		matrix[3][3] = 1.0;

		matrix[0][3] = p_0_cur(0,0);
		matrix[1][3] = p_0_cur(1,0);
		matrix[2][3] = p_0_cur(2,0);
	} else {
		double _qCurr[NUMBER_OF_JOINTS];
 	  	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
			_qCurr[i] = 0.0;
		}
		memcpy(_qOld, _qCurr, NUMBER_OF_JOINTS * sizeof(double));
		//curan::utils::cout << "the joints are: \n";
	    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
	        iiwa->q[i] = _qCurr[i];
	    }
		auto t2 = std::chrono::steady_clock::now();
		auto sampleTimeChrono = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1);
		double sampleTime = (sampleTimeChrono.count()< 1) ? 0.001 : sampleTimeChrono.count()/1000.0;	
		t1 = t2;
 	   for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
 	       iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
	    }
		//curan::utils::cout << "\n";
	
		static Vector3d p_0_cur = Vector3d::Zero(3, 1);
		static Matrix3d R_0_7 = Matrix3d::Zero(3, 3);
		static Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric

 	   robot->getMassMatrix(iiwa->M, iiwa->q);
 	   iiwa->M(6, 6) = 45 * iiwa->M(6, 6);                                       // Correct mass of last body to avoid large accelerations
 	   iiwa->Minv = iiwa->M.inverse();
 	   robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
 	   robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7);              // 3x1 position of flange (body = 7), expressed in base coordinates
 	   robot->getRotationMatrix(R_0_7, iiwa->q, NUMBER_OF_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates
	

		p_0_cur *= 1000; 
		matrix[0][0] = R_0_7(0, 0);
		matrix[1][0] = R_0_7(1, 0);
		matrix[2][0] = R_0_7(2, 0);

		matrix[0][1] = R_0_7(0, 1);
		matrix[1][1] = R_0_7(1, 1);
		matrix[2][1] = R_0_7(2, 1);

		matrix[0][2] = R_0_7(0, 2);
		matrix[1][2] = R_0_7(1, 2);
		matrix[2][2] = R_0_7(2, 2);

		matrix[3][0] = 0.0;
		matrix[3][1] = 0.0;
		matrix[3][2] = 0.0;
		matrix[3][3] = 1.0;

		matrix[0][3] = p_0_cur(0,0);
		matrix[1][3] = p_0_cur(1,0);
		matrix[2][3] = p_0_cur(2,0);
	}
}

int main(int argc, char* argv[]) {
	try{
	std::cout << "How to use: \nCall the executable as DataCollection filename, \ne.g. : DataCollection robot_movement\n";
	if(argc!=2){
		std::cout << "Must provide at least one argument to the executable\n";
		return 1;
	}
	std::string filename{argv[1]};
	
	auto robot_flag = curan::utilities::Flag::make_shared_flag();
	robot_flag->set();

	auto shared_state = std::make_shared<SharedState>();
	shared_state->is_initialized.store(false);
	auto robot_functional_control = [shared_state, robot_flag]() {
		robot_control(shared_state, robot_flag);
	};

	std::thread thred_robot_control{ robot_functional_control };

	//now that the control is running in parallel we can record the 
	//readings from the shared state into a structure (still undecided if it should be a file or not)

	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);                      // Select the robot here

	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>(); // myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	double toolMass = 0.0;                                                                     // No tool for now
	Vector3d toolCOM = Vector3d::Zero(3, 1);
	Matrix3d toolInertia = Matrix3d::Zero(3, 3);
	std::unique_ptr<ToolData> myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);
	robot->attachToolToRobotModel(myTool.get());
	while(true){
		GetRobotConfiguration(matrix, robot.get(), iiwa.get(), shared_state);
	}

	robot_flag->clear();
	thred_robot_control.join();
	return 0;
	} catch(std::exception& e){
		std::cout << "main Exception : " << e.what() << std::endl;
	}
}
