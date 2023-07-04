#ifndef CURAN_CALIBRATEPAGES_HEADER_FILE_
#define CURAN_CALIBRATEPAGES_HEADER_FILE_

#include "MessageProcessing.h"

std::unique_ptr<curan::ui::Overlay> create_filtercontroler_overlay(ProcessingMessage* processing);


std::unique_ptr<curan::ui::Overlay> create_options_overlay(ProcessingMessage* processing);


std::unique_ptr<curan::ui::Page> create_main_page(ConfigurationData& data, ProcessingMessage* processing);

#endif