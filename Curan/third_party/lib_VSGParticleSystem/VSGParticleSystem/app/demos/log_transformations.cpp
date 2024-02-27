#include <vsgParticleSystem/all.h>

int main(int argc, char const *argv[])
{
    std::string path = "../../data/dynamics/2023_10_04_demo_v2-3D.json";

    vsgps::World2Disp w2d = vsgps::World2Disp::fromJson(path);
    vsgps::Disp2Force d2f = vsgps::Disp2Force::fromJson(path);
    vsgps::SoftPlus sp;
    sp.fromJson(path);
    vsgps::Normalization normalization;
    normalization.fromJson(path);
    vsgps::GravityField g;
    g.fromJson(path);

    std::ofstream log("../../transformations.log");
    log << w2d << d2f << sp << normalization << g;
    std::cout << w2d << d2f << sp << normalization << g;
    return 0;
}
