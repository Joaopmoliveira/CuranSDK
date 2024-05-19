#ifndef CURAN_INTEROPERATIVE_HEADER_FILE_
#define CURAN_INTEROPERATIVE_HEADER_FILE_

#include "MessageProcessing.h"
#include "userinterface/widgets/IconResources.h"

std::unique_ptr<curan::ui::Container> create_main_page(curan::ui::IconResources& resources, ProcessingMessage& in_volume) ;

#endif