#include "link_demo.h"

constexpr size_t n_joints = 7;

bool process_joint_message(info_solve_registration &state, const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
{
	Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric
    Vector3d p_0_cur;
    Matrix3d R_0_7;
	if (er)
		return true;
	static std::array<double,n_joints> _qOld = message->angles;
	//std::cout << "angles received: ";
	for (int i = 0; i < n_joints; i++){
		state.iiwa->q[i] = message->angles[i];
		state.iiwa->qDot[i] = (message->angles[i] - _qOld[i]) / 0.005;
		//std::printf(" %f ",message->angles[i]);
	}
	//std::cout << "\n";

	_qOld = message->angles;
	state.robot->getMassMatrix(state.iiwa->M, state.iiwa->q);
	state.iiwa->M(6, 6) = 45 * state.iiwa->M(6, 6); // Correct mass of last body to avoid large accelerations
	state.iiwa->Minv = state.iiwa->M.inverse();
	state.robot->getCoriolisAndGravityVector(state.iiwa->c, state.iiwa->g, state.iiwa->q, state.iiwa->qDot);
	state.robot->getWorldCoordinates(p_0_cur, state.iiwa->q, pointPosition, 7); // 3x1 position of flange (body = 7), expressed in base coordinates
	state.robot->getRotationMatrix(R_0_7, state.iiwa->q, n_joints);		// 3x3 rotation matrix of flange, expressed in base coordinates
	
	for (size_t joint_index = 0; joint_index < curan::communication::FRIMessage::n_joints; ++joint_index)
		state.robot_render->cast<curan::renderable::SequencialLinks>()->set(joint_index, message->angles[joint_index]);

	Eigen::Matrix<double,4,4> mat_current = Eigen::Matrix<double,4,4>::Identity();
	mat_current.block(0,0,3,3) = R_0_7;
	mat_current.block(0,3,3,1) = p_0_cur;
	if(state.robot_client_commands_volume_init.load()){
		state.moving_homogenenous.update_matrix(mat_current);
		auto transform = vsg::translate(0.0,0.0,0.0);
		for(size_t col = 0; col < 4; ++col)
			for(size_t row = 0; row < 4; ++row)
				transform(col,row) = mat_current(row,col);
		state.volume_moving->update_transform(transform);
	}
		
	return false;
}

int communication(info_solve_registration &state)
{
	// Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation Matrix, ...)
	kuka::Robot::robotName myName(kuka::Robot::LBRiiwa); // Select the robot here

	auto robot = std::make_unique<kuka::Robot>(myName); // myLBR = Model
	auto iiwa = std::make_unique<RobotParameters>();	// myIIWA = Parameters as inputs for model and control, e.g., q, qDot, c, g, M, Minv, J, ...

	double toolMass = 0.0; // No tool for now
	Vector3d toolCOM = Vector3d::Zero(3, 1);
	Matrix3d toolInertia = Matrix3d::Zero(3, 3);
	std::unique_ptr<ToolData> myTool = std::make_unique<ToolData>(toolMass, toolCOM, toolInertia);
	robot->attachToolToRobotModel(myTool.get());

	state.robot = std::move(robot);
	state.iiwa = std::move(iiwa);

	asio::io_context context;
	curan::communication::interface_fri fri_interface;
	curan::communication::Client::Info fri_construction{context, fri_interface};
	asio::ip::tcp::resolver fri_resolver(context);
	auto fri_endpoints = fri_resolver.resolve("172.31.1.148", std::to_string(50010));
	fri_construction.endpoints = fri_endpoints;
	curan::communication::Client fri_client{fri_construction};

	auto lam_fri = [&](const size_t &protocol_defined_val, const std::error_code &er, std::shared_ptr<curan::communication::FRIMessage> message)
	{
		try
		{
			if (process_joint_message(state, protocol_defined_val, er, message))
				context.stop();
		}
		catch (...)
		{
			std::cout << "Exception was thrown\n";
		}
	};
	auto fri_connectionstatus = fri_client.connect(lam_fri);
	if (!fri_connectionstatus)
	{
		throw std::runtime_error("missmatch between communication interfaces");
	}

	context.run();
	return 0;
}