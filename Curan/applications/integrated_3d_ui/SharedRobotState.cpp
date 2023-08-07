#include "SharedRobotState.h"

SharedRobotState::SharedRobotState() : commit_senpuko(false){

}

std::shared_ptr<SharedRobotState> SharedRobotState::make_shared(){
    return std::shared_ptr<SharedRobotState>(new SharedRobotState());
}