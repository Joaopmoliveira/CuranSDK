#ifndef CURAN_PASSIVE_CONTROLLER_AUDE_
#define CURAN_PASSIVE_CONTROLLER_AUDE_        

#include "LBRController.h"

namespace curan {
namespace robotic {

EigenState passiveControllerAude(void* user_pointer,kuka::Robot* robot, RobotParameters* iiwa, EigenState state);

}
}

#endif