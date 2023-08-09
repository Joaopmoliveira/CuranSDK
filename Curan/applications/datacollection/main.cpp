#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"

#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"

#include <iostream>
#include <thread>
#include "utils/Logger.h"
#include "utils/Flag.h"
#include "utils/SafeQueue.h"

#include "MyLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"
#include <csignal>

// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset  = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x,y));
        else {
            Data[Offset] = ImVec2(x,y);
            Offset =  (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset  = 0;
        }
    }
};

std::atomic<std::array<double,NUMBER_OF_JOINTS>> robot_joint_config;

void interface(vsg::CommandBuffer& cb){
    ImGui::Begin("Joint Angles"); // Create a window called "Hello, world!" and append into it.
	static std::array<ScrollingBuffer,NUMBER_OF_JOINTS> buffers;
    static float t = 0;
    t += ImGui::GetIO().DeltaTime;
	auto local_copy = robot_joint_config.load();
    
    static float history = 10.0f;
    ImGui::SliderFloat("History",&history,1,30,"%.1f s");

    static ImPlotAxisFlags flags = ImPlotAxisFlags_NoTickLabels;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1,150))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1,t - history, t, ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1,0,1);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL,0.5f);
		for(size_t index = 0; index < NUMBER_OF_JOINTS ; ++index){
			std::string loc = "joint "+std::to_string(index);
            buffers[index].AddPoint(t,(float)local_copy[index]);
			ImPlot::PlotLine(loc.data(), &buffers[index].Data[0].x, &buffers[index].Data[0].y, buffers[index].Data.size(), 0, buffers[index].Offset, 2 * sizeof(float));
		}
        ImPlot::EndPlot();
    }
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
    ImGui::End();
}

constexpr unsigned short DEFAULT_PORTID = 30200;

curan::utilities::SafeQueue<KUKA::FRI::LBRState> recordings;


void signal_handler(int signal)
{
    recordings.invalidate();
}

void robot_control(curan::utilities::SafeQueue<KUKA::FRI::LBRState>& to_record,std::shared_ptr<curan::utilities::Flag> flag) {
	try{
	curan::utilities::cout << "Lauching robot control thread\n";
	MyLBRClient client = MyLBRClient(to_record);
	KUKA::FRI::UdpConnection connection;
	KUKA::FRI::ClientApplication app(connection, client);
	app.connect(DEFAULT_PORTID, NULL);
	bool success = app.step();
	if(success)
		curan::utilities::cout << "Established first message between robot and host\n";
	else
	curan::utilities::cout << "Failed to established first message between robot and host\n";
	while (success && flag->value())
		success = app.step();
	app.disconnect();
	return;
	} catch(...){
		std::cout << "robot control exception\n";
		return;
	}
}

void render_robot_scene(std::atomic<std::array<double,NUMBER_OF_JOINTS>>& robot_config){
	curan::renderable::Window::Info info;
   info.api_dump = false;
   info.display = "";
   info.full_screen = false;
   info.is_debug = false;
   info.screen_number = 0;
   info.title = "myviewer";
   curan::renderable::Window::WindowSize size{2000, 1200};
   info.window_size = size;
   curan::renderable::Window window{info};


   std::filesystem::path robot_path = CURAN_COPIED_RESOURCE_PATH"/models/lbrmed/arm.json";
   curan::renderable::SequencialLinks::Info create_info;
   create_info.convetion = vsg::CoordinateConvention::Y_UP;
   create_info.json_path = robot_path;
   create_info.number_of_links = 8;
   auto robot = curan::renderable::SequencialLinks::make(create_info);
   window << robot;
   while(!recordings.is_invalid()){
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		auto local = robot_config.load();
		for(size_t joint_index = 0 ; joint_index < NUMBER_OF_JOINTS; ++joint_index)
        	robot->cast<curan::renderable::SequencialLinks>()->set(joint_index,local[joint_index]);
   }
};

void GetRobotConfiguration(Eigen::Matrix<double,4,4>& matrix, kuka::Robot* robot, RobotParameters* iiwa,KUKA::FRI::LBRState robot_state)
{
	auto sampleTime = robot_state.getSampleTime();
	static double _qOld[NUMBER_OF_JOINTS];
	auto _qCurr = robot_state.getMeasuredJointPosition();
	memcpy(_qOld, _qCurr, NUMBER_OF_JOINTS * sizeof(double));
	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        iiwa->q[i] = _qCurr[i];
    }
	for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
        iiwa->qDot[i] = (_qCurr[i] - _qOld[i]) / sampleTime;
    }
	
	static Vector3d p_0_cur = Vector3d::Zero(3, 1);
	static Matrix3d R_0_7 = Matrix3d::Zero(3, 3);
	static Vector3d pointPosition = Vector3d(0, 0, 0.045); // Point on center of flange for MF-Electric

	robot->getMassMatrix(iiwa->M, iiwa->q);
    iiwa->M(6, 6) = 45 * iiwa->M(6, 6);                                       // Correct mass of last body to avoid large accelerations
    iiwa->Minv = iiwa->M.inverse();
	robot->getCoriolisAndGravityVector(iiwa->c, iiwa->g, iiwa->q, iiwa->qDot);
	robot->getWorldCoordinates(p_0_cur, iiwa->q, pointPosition, 7);              // 3x1 position of flange (body = 7), expressed in base coordinates
    robot->getRotationMatrix(R_0_7, iiwa->q, NUMBER_OF_JOINTS);                                // 3x3 rotation matrix of flange, expressed in base coordinates
	
	matrix(0, 0) = R_0_7(0, 0);
	matrix(1, 0) = R_0_7(1, 0);
	matrix(2, 0) = R_0_7(2, 0);

	matrix(0, 1) = R_0_7(0, 1);
	matrix(1, 1) = R_0_7(1, 1);
	matrix(2, 1) = R_0_7(2, 1);

	matrix(0, 2) = R_0_7(0, 2);
	matrix(1, 2) = R_0_7(1, 2);
	matrix(2, 2) = R_0_7(2, 2);

	matrix(3, 0) = 0.0;
	matrix(3, 1) = 0.0;
	matrix(3, 2) = 0.0;
	matrix(3, 3) = 1.0;

	matrix(0, 3) = p_0_cur(0,0);
	matrix(1, 3) = p_0_cur(1,0);
	matrix(2, 3) = p_0_cur(2,0);
}

int main(int argc, char* argv[]) {
	// Install a signal handler
    std::signal(SIGINT, signal_handler);
	try{
	std::cout << "How to use: \nCall the executable as DataCollection filename, \ne.g. : DataCollection robot_movement\n";
	if(argc!=2){
		std::cout << "Must provide at least one argument to the executable\n";
		return 1;
	}
	std::string filename{argv[1]};
	
	auto robot_flag = curan::utilities::Flag::make_shared_flag();
	robot_flag->set();

	auto robot_functional_control = [robot_flag]() {
		robot_control(recordings, robot_flag);
	};

	std::thread thred_robot_control{ robot_functional_control };
	auto robot_render = [&]() {
		render_robot_scene(robot_joint_config);
	};

	std::thread thred_robot_render{ robot_render };

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
	KUKA::FRI::LBRState state;
	Eigen::Matrix<double,4,4> local_mat;

	std::list<Eigen::Matrix<double,4,4>> list_of_homogenenous_readings;
	while(!recordings.is_invalid()){
		if(recordings.wait_and_pop(state)){
			GetRobotConfiguration(local_mat,robot.get(),iiwa.get(),state);
			list_of_homogenenous_readings.emplace_back(local_mat);
			std::array<double,7> joint_config;
			auto _qCurr = state.getMeasuredJointPosition();
			memcpy(joint_config.data(), _qCurr, NUMBER_OF_JOINTS * sizeof(double));
			robot_joint_config.store(joint_config);
		}
	}

	robot_flag->clear();
	thred_robot_control.join();
	thred_robot_render.join();

	nlohmann::json data_to_record;
	std::string name = "datapoint";
	std::ofstream file_with_data(filename);
	if(!file_with_data){
		std::cout << "failure to open file\n";
		return 1;
	}
	size_t recording_number = 1;
	for(const auto& homogeneous : list_of_homogenenous_readings){
		std::stringstream ss;
		ss << homogeneous;
		data_to_record[name+std::to_string(recording_number)] = ss.str();
		++recording_number;
	}
	file_with_data << data_to_record;
	return 0;
	} catch(std::exception& e){
		std::cout << "main Exception : " << e.what() << std::endl;
	}
}
