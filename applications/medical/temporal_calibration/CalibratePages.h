#ifndef CURAN_CALIBRATEPAGES_HEADER_FILE_
#define CURAN_CALIBRATEPAGES_HEADER_FILE_

#include "MessageProcessing.h"
#include "userinterface/widgets/IconResources.h"

std::unique_ptr<curan::ui::Overlay> create_ransac_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> create_segmentation_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> create_aquisition_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> create_options_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);
std::unique_ptr<curan::ui::Overlay> create_results_overlay(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);


curan::ui::Page create_main_page(std::shared_ptr<ProcessingMessage>& processing,curan::ui::IconResources& resources);

#endif