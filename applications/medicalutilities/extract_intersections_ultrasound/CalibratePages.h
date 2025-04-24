#ifndef CURAN_CALIBRATEPAGES_HEADER_FILE_
#define CURAN_CALIBRATEPAGES_HEADER_FILE_

#include "MessageProcessing.h"
#include "userinterface/widgets/IconResources.h"

curan::ui::Page create_main_page(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);

#endif