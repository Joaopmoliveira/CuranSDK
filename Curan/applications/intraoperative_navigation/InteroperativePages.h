#ifndef CURAN_CALIBRATEPAGES_HEADER_FILE_
#define CURAN_CALIBRATEPAGES_HEADER_FILE_

#include "MessageProcessing.h"
#include "userinterface/widgets/IconResources.h"

ProcessingMessage create_main_page(curan::ui::IconResources& resources,std::unique_ptr<curan::ui::Container>& container, InputImageType::Pointer in_volume);

#endif