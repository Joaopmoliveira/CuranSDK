#ifndef CURAN_HAND_GUIDANCE_
#define CURAN_HAND_GUIDANCE_        

#include "LBRController.h"

namespace curan {
namespace robotic {

EigenState handguidance(void* user_pointer,kuka::Robot* robot, RobotParameters* iiwa, EigenState state);

}
}

#endif