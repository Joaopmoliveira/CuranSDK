#ifndef CURAN_HAND_GUIDANCE_
#define CURAN_HAND_GUIDANCE_        

#include "LBRController.h"

namespace curan {
namespace robotic {

struct HandGuidance : public UserData{
    HandGuidance();
    EigenState&& update(kuka::Robot* robot, RobotParameters* iiwa, EigenState&& state) override;
};

}
}

#endif