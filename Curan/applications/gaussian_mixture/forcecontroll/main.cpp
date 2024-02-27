#include "communication/ProtoFRI.h"
#include "communication/ProtoIGTL.h"
#include "communication/Server.h"

#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/Sphere.h"
#include "rendering/Box.h"
#include "rendering/Window.h"

#include "utils/Flag.h"
#include "utils/Logger.h"
#include "utils/SafeQueue.h"
#include <iostream>
#include <thread>

#include "MyLBRClient.h"
#include "friClientApplication.h"
#include "friUdpConnection.h"
#include <csignal>

/** TODO:
 * 2. Check velocities from the dataset
 * 3. Add the contact plane
 * 4. Add the incoming data in world coordinates, with the equilibrium point
 * denoted
 * 5. Check for other ways of simulating the force control without the robot
 * 6. Improve forcecontroll's code: readibility, remove unnecessary code/logic,
 * etc.
 */

constexpr unsigned short DEFAULT_PORTID = 30200;

std::atomic<bool> progress = true;

void signal_handler(int signal) { progress.store(false); }

// utility structure for realtime plot
struct ScrollingBuffer {
    int MaxSize;
    int Offset;
    ImVector<ImVec2> Data;
    ScrollingBuffer(int max_size = 2000) {
        MaxSize = max_size;
        Offset = 0;
        Data.reserve(MaxSize);
    }
    void AddPoint(float x, float y) {
        if (Data.size() < MaxSize)
            Data.push_back(ImVec2(x, y));
        else {
            Data[Offset] = ImVec2(x, y);
            Offset = (Offset + 1) % MaxSize;
        }
    }
    void Erase() {
        if (Data.size() > 0) {
            Data.shrink(0);
            Offset = 0;
        }
    }
};

std::atomic<std::array<double, NUMBER_OF_JOINTS>> robot_torques;
std::atomic<std::array<double, 3>> end_effector_velocity;

void addPlane(std::string const &path, curan::renderable::Window &window) {
    try {
        auto fullData = nlohmann::json::parse(std::ifstream(path));
        Eigen::Vector4d normal;
        Eigen::Vector3d origin;

        size_t i = 0;
        for (auto const &d : fullData["config"]["normal"]) {
            normal(i) = d;
            i++;
        }

        i = 0;
        for (auto const &d : fullData["config"]["origin"]) {
            origin(i) = d;
            i++;
        }

        curan::renderable::Box::Info planeInfo;
        planeInfo.builder = vsg::Builder::create();
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        vsg::ref_ptr<vsg::vec4Array> positions, color;
        // todo: finish setting up the plane: 
        // -> check how the Box is implemented
        // -> Position Box centered at the origin
        // -> Rotate it to align with the normal
        // -> Scale it to resemble a plane


    } catch (std::exception const &e) {
        std::cout << "Plane wasn't implemented";
        return;
    }
}

void addWorldData(std::string path, curan::renderable::Window &window) {
    try {
        auto fullData = nlohmann::json::parse(std::ifstream(path));

        auto worldData = fullData["worldData"];

        // The last point in the world data can be denoted as the equilibrium
        // point by default
        auto eqPoint = *(worldData.end() - 1);
        if (fullData["eqPoint"].is_object() &&
            fullData["eqPoint"].contains("world")) {
            eqPoint = fullData["eqPoint"]["world"];
        }

        curan::renderable::Sphere::Info sphereInfo{};
        auto builder = vsg::Builder::create();
        vsg::GeometryInfo geomInfo;
        vsg::StateInfo stateInfo;
        vsg::ref_ptr<vsg::vec4Array> positions, color;

        for (auto &wd : worldData) {
            sphereInfo = {};
            sphereInfo.builder = builder;
            sphereInfo.geomInfo = {};
            sphereInfo.stateInfo = {};
            sphereInfo.position = {wd[0], wd[1], wd[2]};

            positions = vsg::vec4Array::create(1);
            sphereInfo.geomInfo.dx = {.01, 0.0, 0.0};
            sphereInfo.geomInfo.dy = {0.0, .01, 0.0};
            sphereInfo.geomInfo.dz = {0.0, 0.0, .01};

            color = vsg::vec4Array::create(1);
            sphereInfo.geomInfo.colors = color;
            for (auto &c : *color) {
                c.set(1.0, 0.0, 0.0, 1.0);
            }

            window << curan::renderable::Sphere::make(sphereInfo);
        }

        {
            sphereInfo = {};
            sphereInfo.builder = builder;
            sphereInfo.geomInfo = {};
            sphereInfo.stateInfo = {};
            sphereInfo.position = {eqPoint[0], eqPoint[1], eqPoint[2]};

            positions = vsg::vec4Array::create(1);
            sphereInfo.geomInfo.dx = {.03, 0.0, 0.0};
            sphereInfo.geomInfo.dy = {0.0, .03, 0.0};
            sphereInfo.geomInfo.dz = {0.0, 0.0, .03};

            color = vsg::vec4Array::create(1);
            sphereInfo.geomInfo.colors = color;
            for (auto &c : *color) {
                c.set(0.0, 0.0, 0.0, 1.0);
            }

            window << curan::renderable::Sphere::make(sphereInfo);
        }

    } catch (std::exception &e) {
        std::cout << "There is no world data available\n";
        return;
    }
}

void interface(vsg::CommandBuffer &cb,
               std::shared_ptr<SharedState> shared_state) {
    ImGui::Begin("End-Effector Velocity"); // Create a window called "Hello,
                                           // world!" and append into it.
    static std::array<ScrollingBuffer, 3> eev;
    auto local_copy_eev = shared_state->velocity.load();
    static float covariance = shared_state->sigma.load();
    static float history_eev = 10.0f;
    ImGui::SliderFloat("History", &history_eev, 1, 30, "%.1f s");
    ImGui::SliderFloat("Covariance", &covariance, 1e-5, 1, "%f");
    shared_state->sigma.store(covariance);
    static bool stop_buffer = false;
    ImGui::Checkbox("Stop buffer", &stop_buffer);
    static float t2 = 0;
    t2 += ImGui::GetIO().DeltaTime * !stop_buffer;

    auto flags = ImPlotAxisFlags_AuxDefault;

    if (ImPlot::BeginPlot("##Scrolling", ImVec2(-1, -1))) {
        ImPlot::SetupAxes(NULL, NULL, flags, flags);
        ImPlot::SetupAxisLimits(ImAxis_X1, t2 - history_eev, t2,
                                ImGuiCond_Always);
        ImPlot::SetupAxisLimits(ImAxis_Y1, -10, 10);
        ImPlot::SetNextFillStyle(IMPLOT_AUTO_COL, 0.5f);
        std::string loc = "";
        std::array<std::string, 3> vel_labels{"x", "y", "z"};
        for (size_t index = 0; index < 3; ++index) {
            loc = "velocity " + vel_labels[index];
            if (!stop_buffer)
                eev[index].AddPoint(t2, (float)local_copy_eev[index]);
            ImPlot::PlotLine(loc.data(), &eev[index].Data[0].x,
                             &eev[index].Data[0].y, eev[index].Data.size(), 0,
                             eev[index].Offset, 2 * sizeof(float));
        }
        ImPlot::EndPlot();
    }
    ImGui::End();
}

void robot_control(std::shared_ptr<SharedState> shared_state,
                   std::shared_ptr<curan::utilities::Flag> flag,
                   MyLBRClient &client) {
    try {
        curan::utilities::cout << "Lauching robot control thread\n";
        // std::cout << *shared_state->sigma.load() << "\n";
        KUKA::FRI::UdpConnection connection{2};
        KUKA::FRI::ClientApplication app(connection, client);
        app.connect(DEFAULT_PORTID, NULL);
        bool success = true;
        while (flag->value())
            success = app.step();
        app.disconnect();
        return;
    } catch (std::exception &e) {
        std::cout << "robot control exception\n" << e.what() << "\n";
        return;
    }
}

// https://stackoverflow.com/a/17223443
std::string return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

int main(int argc, char *argv[]) {
    // Install a signal handler
    std::signal(SIGINT, signal_handler);
    try {
        std::shared_ptr<curan::utilities::Flag> robot_flag =
            curan::utilities::Flag::make_shared_flag();
        robot_flag->set();

        auto shared_state = std::make_shared<SharedState>();
        shared_state->is_initialized.store(false);

        auto pipelinePath =
            CURAN_COPIED_RESOURCE_PATH "/forcecontrol/robot-gmm4-3D.json";
        std::string outputPath = return_current_time_and_date() + "_force_control.log";
        MyLBRClient client = MyLBRClient(shared_state);
        client.setPipeline(pipelinePath);
        client.setSharpness();
        client.setOutputPath(outputPath);

        auto robot_functional_control = [shared_state, robot_flag, &client]() {
            robot_control(shared_state, robot_flag, client);
        };

        std::thread thred_robot_control{robot_functional_control};

        curan::renderable::Window::Info info;
        curan::renderable::ImGUIInterface::Info info_gui{
            [shared_state](vsg::CommandBuffer &cb) {
                interface(cb, shared_state);
            }};
        auto ui_interface = curan::renderable::ImGUIInterface::make(info_gui);
        info.api_dump = false;
        info.display = "";
        info.full_screen = false;
        info.is_debug = false;
        info.screen_number = 0;
        info.imgui_interface = ui_interface;
        info.title = "myviewer";
        curan::renderable::Window::WindowSize size{2000, 1200};
        info.window_size = size;
        curan::renderable::Window window{info};

        std::filesystem::path robot_path =
            CURAN_COPIED_RESOURCE_PATH "/models/lbrmed/arm.json";
        curan::renderable::SequencialLinks::Info create_info;
        create_info.convetion = vsg::CoordinateConvention::Y_UP;
        create_info.json_path = robot_path;
        create_info.number_of_links = 8;
        auto robot = curan::renderable::SequencialLinks::make(create_info);
        window << robot;

        addWorldData(pipelinePath, window);

        // Use of KUKA Robot Library/robot.h (M, J, World Coordinates, Rotation
        // Matrix, ...) Select the robot here
        kuka::Robot::robotName myName(kuka::Robot::LBRiiwa);

        // myLBR = Model
        auto robot_control = std::make_unique<kuka::Robot>(myName);

        // myIIWA = Parameters as inputs for model and control, e.g., q, qDot,
        // c, g, M, Minv, J, ...
        auto iiwa = std::make_unique<RobotParameters>();

        // Point on center of flange for MF-Electric
        // Positions and orientations
        Vector3d pointPosition = Vector3d(0, 0, 0.045);

        Vector3d p_0_cur = Vector3d::Zero(3, 1);
        Matrix3d R_0_7 = Matrix3d::Zero(3, 3);

        while (progress.load()) {
            if (!window.run_once())
                progress = false;
            if (shared_state->is_initialized) {
                auto local_state = shared_state->robot_state.load();
                for (size_t joint_index = 0; joint_index < NUMBER_OF_JOINTS;
                     ++joint_index) {
                    robot->cast<curan::renderable::SequencialLinks>()->set(
                        joint_index,
                        local_state.getMeasuredJointPosition()[joint_index]);
                    robot_torques.store(shared_state->joint_torques.load());
                }
                end_effector_velocity.store(shared_state->velocity.load());

                // Get robot measurements
                shared_state->is_initialized.store(true);
                static bool first_time = true;
                if (first_time) {
                    first_time = false;
                    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
                        iiwa->q[i] = local_state.getMeasuredJointPosition()[i];
                        iiwa->qDot[i] = 0.0;
                    }
                } else {
                    for (int i = 0; i < NUMBER_OF_JOINTS; i++) {
                        iiwa->qDot[i] =
                            (local_state.getMeasuredJointPosition()[i] -
                             iiwa->q[i]) /
                            local_state.getSampleTime();
                        iiwa->q[i] = local_state.getMeasuredJointPosition()[i];
                    }
                }

                robot_control->getMassMatrix(iiwa->M, iiwa->q);
                iiwa->M(6, 6) =
                    45 * iiwa->M(6, 6); // Correct mass of last body to avoid
                                        // large accelerations
                iiwa->Minv = iiwa->M.inverse();
                robot_control->getCoriolisAndGravityVector(iiwa->c, iiwa->g,
                                                           iiwa->q, iiwa->qDot);
                robot_control->getWorldCoordinates(
                    p_0_cur, iiwa->q, pointPosition,
                    7); // 3x1 position of flange (body = 7), expressed in base
                        // coordinates
                robot_control->getRotationMatrix(
                    R_0_7, iiwa->q,
                    NUMBER_OF_JOINTS); // 3x3 rotation matrix of flange,
                                       // expressed in base coordinates
            }
        }
        robot_flag->clear();
        thred_robot_control.join();
        return 0;
    } catch (std::exception &e) {
        std::cout << "main Exception : " << e.what() << std::endl;
    }
}
