#ifndef CURAN_CALIBRATEPAGES_HEADER_FILE_
#define CURAN_CALIBRATEPAGES_HEADER_FILE_

#include "MessageProcessing.h"

std::shared_ptr<curan::ui::Overlay> create_filtercontroler_overlay(std::shared_ptr<ProcessingMessage>& processing);


std::shared_ptr<curan::ui::Overlay> create_options_overlay(std::shared_ptr<ProcessingMessage>& processing);


std::shared_ptr<curan::ui::Page> create_main_page(ConfigurationData& data, std::shared_ptr<ProcessingMessage>& processing);

#endif