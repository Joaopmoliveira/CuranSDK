#include <fstream>
#include <vsgParticleSystem/transformation/Normalization.h>
#include <vsgParticleSystem/transformation/SoftPlus.h>
#include <vsgParticleSystem/transformation/TPipeline.h>
#include <vsgParticleSystem/transformation/World2Force.h>
#include <vsgParticleSystem/utils/stringUtils.h>

namespace vsgps {

void TPipeline::operator()(State const &xi, State &dxi) {
    this->forward(transformations, xi, dxi);
}

// dxi = f(xi)
State TPipeline::operator()(State const &xi) {
    return this->pipeline(transformations, addOp, xi);
}

void TPipeline::inv(State const &xi, State &dxi) {
    this->inverse(transformations, xi, dxi);
}

void TPipeline::setForward(TransformationsCallback const &f) {
    this->forward = f;
}

void TPipeline::setInverse(TransformationsCallback const &i) {
    this->inverse = i;
}

void TPipeline::setPipeline(PipelineCallback const &p) { this->pipeline = p; }

void TPipeline::setAdditionalOperations(AddOp const &a) { addOp = a; }
AddOp const &TPipeline::getConstAdditionalOperations() const { return addOp; }
AddOp &TPipeline::getAdditionalOperations() { return addOp; }

void TPipeline::setAdditionalInformation(AddInfo const &ai) { addInfo = ai; }

State retrieveEqPoint(nlohmann::json const &data, int dim) {
    State eqPoint = State::Zero(dim, 1);
    int i = 0;
    for (auto const &d : data) {
        eqPoint(i, 0) = d;
        i++;
    }
    return eqPoint;
}

void TPipeline::setAdditionalInformation(std::string source) {
    replaceAll(source, "\\", "/");
    auto data = nlohmann::json::parse(std::ifstream(source))["eqPoint"];
    int dim = nlohmann::json::parse(std::ifstream(source))["dimension"];
    addInfo = {{retrieveEqPoint(data["world"], dim),
                retrieveEqPoint(data["disp"], dim),
                retrieveEqPoint(data["force"], dim),
                retrieveEqPoint(data["deformed"], dim),
                retrieveEqPoint(data["normalized"], dim)}};
}
AddInfo TPipeline::getAdditionalInformation() const { return addInfo; }

TPipeline pipelineFromJson(std::string path) {
    TPipeline tp;
    tp << World2Disp::fromJson(path);
    tp << Disp2Force::fromJson(path);
    tp << Force2Deform::fromJson(path);
    tp << Deform2Norm::fromJson(path);
    tp.setAdditionalOperations({Dynamics::fromJson(path),
                                GravityField::fromJson(path),
                                SecurityRegion::fromJson(path)});
    tp.setAdditionalInformation(path);

    tp.setPipeline([](Transformations &trans, AddOp &ao, State const &xi) {
        State xiDisp(xi.size()), xiDeformed(xi.size()), xiNorm(xi.size());
        xiDisp.setZero();
        xiDeformed.setZero();
        xiNorm.setZero();
        // std::ofstream logPipeline("logPipeline.csv", std::ios_base::app);
        // logPipeline << std::setprecision(17);
        // printVector(logPipeline, xi);

        // world -> disp
        (*trans[World2Disp::label])(xi, xiDisp);
        ao.sr.validate(xiDisp);
        // printVector(logPipeline, xiDisp);

        // disp -> force
        (*trans[Disp2Force::label])(xiDisp, xiDeformed);
        // printVector(logPipeline, xiDeformed);

        // force -> deformed
        (*trans[Force2Deform::label])(xiDeformed);
        // printVector(logPipeline, xiDeformed);

        // deformed -> normalized
        (*trans[Deform2Norm::label])(xiDeformed, xiNorm);
        // printVector(logPipeline, xiNorm);

        // dxi = f(xi)
        State dxi = ao.dynamics(xiNorm);
        // printVector(logPipeline, dxi);

        // normalized -> deformed
        (*trans[Deform2Norm::label]).invVel(dxi);
        // printVector(logPipeline, dxi);

        // deformed -> force
        (*trans[Force2Deform::label]).invVel(xiDeformed, dxi);
        // printVector(logPipeline, dxi);

        // force -> disp
        (*trans[Disp2Force::label]).invVel(dxi);
        // printVector(logPipeline, dxi);

        // correct with gravity field
        dxi = ao.gf(xiDisp, dxi);
        // printVector(logPipeline, dxi);

        // disp -> world
        (*trans[World2Disp::label]).invVel(dxi);
        // printVector(logPipeline, dxi);

        // logPipeline.close();

        return dxi;
    });

    return tp;
}

} // namespace vsgps
