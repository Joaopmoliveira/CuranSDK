#ifndef LINK_DEMO_HEADER_D
#define LINK_DEMO_HEADER_D

#include <vsg/all.h>
#include <vsgXchange/all.h>
#include <iostream>
#include "SharedRobotState.h"
#include <iostream>
#include "Robot.h"
#include "ToolData.h"
#include "robotParameters.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/SequencialLinks.h"
#include "rendering/DynamicTexture.h"
#include <asio.hpp>
#include "communication/Client.h"
#include "communication/Server.h"
#include "communication/ProtoIGTL.h"
#include "communication/ProtoFRI.h"

int communication(std::shared_ptr<SharedRobotState> state,asio::io_context& context);

#endif