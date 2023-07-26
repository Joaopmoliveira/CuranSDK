#ifndef LINK_DEMO_HEADER_D
#define LINK_DEMO_HEADER_D

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "SharedRobotState.h"

int communication(std::shared_ptr<SharedRobotState> state);

#endif