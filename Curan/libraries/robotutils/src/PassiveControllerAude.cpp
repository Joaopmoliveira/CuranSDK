#include "robotutils/PassiveControllerAude.h"

namespace curan {
namespace robotic {

    PassiveControllerData::PassiveControllerData(const std::string& model_file){
        {
            std::ifstream modelfile{model_file};
            modelfile >> model;
        }
    };

    EigenState&& PassiveControllerData::update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state){
        auto velocity = model.likeliest(state.translation);
    };
}
}