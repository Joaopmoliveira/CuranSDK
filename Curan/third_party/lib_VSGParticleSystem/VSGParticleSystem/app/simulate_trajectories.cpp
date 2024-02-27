#include <cmath>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <vsg/all.h>
#include <vsgParticleSystem/all.h>
#include <vsgXchange/all.h>

void visualizeTrajectory(vsg::CommandLine &arguments) {
    auto options = vsg::Options::create();
    options->add(vsgXchange::all::create());
    options->paths = vsg::getEnvPaths("VSG_FILE_PATH");

    auto windowTraits = vsg::WindowTraits::create();
    windowTraits->windowTitle = "Particle System";

    windowTraits->debugLayer = arguments.read({"--debug", "-d"});
    windowTraits->apiDumpLayer = arguments.read({"--api", "-a"});
    if (arguments.read({"--fullscreen", "--fs"}))
        windowTraits->fullscreen = true;
    arguments.read("--screen", windowTraits->screenNum);
    arguments.read("--display", windowTraits->display);

    auto basis = vsg::Group::create();
    auto root = vsg::Group::create();
    basis->addChild(root);
    auto builder = vsg::ResolutionBuilder::create();
    builder->options = options;

    vsg::GeometryInfo geomInfo;
    geomInfo.dx.set(1.0, 0.0, 0.0);
    geomInfo.dy.set(0.0, 1.0, 0.0);
    geomInfo.dz.set(0.0, 0.0, 1.0);
    size_t numOfObjects = 1000;
    geomInfo.positions = vsg::vec4Array::create(numOfObjects);
    auto colors = vsg::vec4Array::create(numOfObjects);
    geomInfo.colors = colors;
    for (auto &c : *colors)
        c.set(1.0, 1.0, 1.0, 1.0);

    vsg::StateInfo stateInfo;
    stateInfo.blending = true;
}

void addStateLine(std::ostringstream &os, double const &t,
                  vsgps::State const &state) {
    os << t;
    for (const double &s : state) {
        os << "\t" << s;
    }
    os << "\n";
}

// https://stackoverflow.com/a/17223443
std::string return_current_time_and_date() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char **argv) {
    // set up defaults and read command line arguments to override them
    vsg::CommandLine arguments(&argc, argv);

    if (arguments.errors())
        return arguments.writeErrorMessages(std::cerr);

    auto dynFile = arguments.value<vsg::Path>(
        "../../data/dynamics/2023_10_04_demo_v2-3D.json", "--dyn");

    auto simName =
        arguments.value<std::string>("simulate_trajectories", "--name");

    auto tp = vsgps::pipelineFromJson(dynFile);

    std::vector<vsgps::State> states;
    int dimension = tp.getAdditionalOperations().dynamics.getDimension();
    vsgps::State state(dimension);
    if (auto initStateFilePath = arguments.value<vsg::Path>(
            "../../data/initState/trajectories-start.json", "--init-state");
        initStateFilePath != "") {
        std::ifstream initStateFile(initStateFilePath);
        auto data = nlohmann::json::parse(initStateFile);
        for (auto const &d : data) {
            state.setZero();

            for (int i = 0; i < dimension && i < d.size(); i++)
                state[i] = (double)d[i];

            states.push_back(state);
        }
    } else {
        size_t numOfParticles = arguments.value<size_t>(1, "-n");
        switch (arguments.value<size_t>(0, "-m")) {
        case 1: {
            double dtheta = 2.0 * vsg::PI / ((double)numOfParticles);
            for (size_t i = 0; i < numOfParticles; i++) {
                state.setZero();
                state(Eigen::seq(0, 2)) << std::cos(i * dtheta),
                    std::sin(i * dtheta), .01;
                states.push_back(state);
            }
            break;
        }
        case 0:
        default:
            std::srand(std::time(nullptr));
            for (size_t i = 0; i < numOfParticles; i++) {
                state.setZero();
                for (auto &s : state)
                    s = double(std::rand()) / double(RAND_MAX);
                states.push_back(state);
            }
            break;
        }
    }
    
    double t = 0, dt = 0.01, normLimit = 1e-5, tLimit = 50;
    vsgps::State eqPoint = tp.getAdditionalInformation().eqPoint.world;
    std::ostringstream os;
    os << std::fixed << std::setprecision(6);
    vsg::info("Initial States:");
    for (auto &s : states) {
        t = 0;
        std::cout << "State " << (&s - &states[0]) + 1 << ":\n";
        std::cout << s.transpose() << "\n";
        do {
            os << (int)(((&s - &states[0]) % states.size()) + 1) << "\t";
            addStateLine(os, t, s);
            s += tp(s) * dt;
            t += dt;
        } while (t < tLimit && (s - eqPoint).norm() > normLimit);
    }

    std::string fname = "vsgps-results-" + simName + "-" +
                        return_current_time_and_date() + ".csv";
    std::ofstream outputFile(fname);
    outputFile << os.str();
    vsg::info("Trajectory saved on ", fname);
    outputFile.close();

    return 0;
}

/*
 * Plano para a trajetória:
 * * [Done] Saber a dinâmica => include "Dynamics.h" && json com dinâmica
 * * [Done] Saber estado inicial => por CLI ou por ficheiro
 * * [Done] Calcular mudança de estado a cada 0.001 s
 * * [Done] Salvar para um ficheiro
 */