#include <iostream>
#include <vsgParticleSystem/all.h>

int main(int argc, char const *argv[]) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <path to model's json file> "
                  << "<initial state's json file>"
                  << "" << std::endl;
        return 1;
    }
    std::string modelPath = argv[1];
    std::string initialPath = argv[2];

    vsgps::World2Disp w2d = vsgps::World2Disp::fromJson(modelPath);
    vsgps::Disp2Force d2f = vsgps::Disp2Force::fromJson(modelPath);
    vsgps::Force2Deform sp = vsgps::Force2Deform::fromJson(modelPath);
    vsgps::Deform2Norm normalization = vsgps::Deform2Norm::fromJson(modelPath);
    vsgps::GravityField g = vsgps::GravityField::fromJson(modelPath);
    vsgps::Dynamics dynamics = vsgps::Dynamics::fromJson(modelPath);
    vsgps::SecurityRegion sr = vsgps::SecurityRegion::fromJson(modelPath);
    vsgps::TPipeline tp;

    tp << w2d;
    tp << d2f;
    tp << sp;
    tp << normalization;
    tp.setAdditionalOperations({dynamics, g, sr});
    tp.setAdditionalInformation(modelPath);

    std::ofstream log("logging-steps.log");
    log << std::fixed << std::setprecision(6);
    log << w2d << d2f << sp << normalization << g << dynamics << sr;
    tp.setPipeline([&](vsgps::Transformations &trans, vsgps::AddOp &ao,
                       vsgps::State const &xi) {
        vsgps::State xiDisp(xi.size()), xiDeformed(xi.size()),
            xiNorm(xi.size());
        xiDisp.setZero();
        xiDeformed.setZero();
        xiNorm.setZero();

        // world -> disp
        (*trans[vsgps::World2Disp::label])(xi, xiDisp);
        ao.sr.validate(xiDisp);
        log << "World      -> Disp       : ";
        vsgps::printVector(log, xiDisp);

        // disp -> force
        (*trans[vsgps::Disp2Force::label])(xiDisp, xiDeformed);
        log << "Disp       -> Force      : ";
        vsgps::printVector(log, xiDeformed);

        // force -> deformed
        (*trans[vsgps::Force2Deform::label])(xiDeformed);
        log << "Force      -> Deformed   : ";
        vsgps::printVector(log, xiDeformed);

        // deformed -> normalized
        (*trans[vsgps::Deform2Norm::label])(xiDeformed, xiNorm);
        log << "Deformed   -> Normalized : ";
        vsgps::printVector(log, xiNorm);

        // dxi = f(xi)
        vsgps::State dxi = ao.dynamics(xiNorm);
        log << "dxi        =  f(xi)      : ";
        vsgps::printVector(log, dxi);

        // normalized -> deformed
        (*trans[vsgps::Deform2Norm::label]).invVel(dxi);
        log << "Normalized -> Deformed   : ";
        vsgps::printVector(log, dxi);

        // deformed -> force
        (*trans[vsgps::Force2Deform::label]).invVel(xiDeformed, dxi);
        log << "Deformed   -> Force      : ";
        vsgps::printVector(log, dxi);

        // force -> disp
        (*trans[vsgps::Disp2Force::label]).invVel(dxi);
        log << "Force      -> Disp       : ";
        vsgps::printVector(log, dxi);

        // correct with gravity field
        dxi = ao.gf(xiDisp, dxi);
        log << "Disp       -> Gravity    : ";
        vsgps::printVector(log, dxi);

        // disp -> world
        (*trans[vsgps::World2Disp::label]).invVel(dxi);
        log << "Gravity    -> World      : ";
        vsgps::printVector(log, dxi);

        return dxi;
    });

    log << "\n#################################################################"
           "\n\n";
    vsgps::State initialState = vsgps::State::Zero(3, 1);
    auto initialData = nlohmann::json::parse(std::ifstream(initialPath));
    int index = 0;
    log << "Initial state: [";
    for (auto const &i : initialData[3]) {
        initialState[index++] = (double)i;
        log << i;
        if (index - 1 != initialData[0].size() - 1)
            log << ", ";
    }
    log << "]\n";

    log << "\nSimulation:\n\n";
    double t = 0, dt = 0.01, normLimit = 1e-5, tLimit = 50;
    vsgps::State eqPoint = tp.getAdditionalInformation().eqPoint.world;
    vsgps::State state = initialState;
    do {
        log << "t                        = " << t << "\n";
        state += tp(state) * dt;
        t += dt;
        log << "\n";
    } while (t < tLimit && (state - eqPoint).norm() > normLimit);

    return 0;
}
