#define _USE_MATH_DEFINES
#include <cmath>
#include "ceres/ceres.h"
#include "optimization/WireCalibration.h"
#include <nlohmann/json.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "CalibratePages.h"
#include "utils/ModifyXMLField.h"

#include <random>


/*
This executable requires:

1. A file located at "CURAN_COPIED_RESOURCE_PATH"/plus_config/plus_spacial_calib_robot_xml/robot_image.xml"" 
which is used to trigger the plus server

And it generates:

1. A modified file  "CURAN_COPIED_RESOURCE_PATH"/plus_config/plus_spacial_calib_robot_xml/robot_image.xml"
with the new temporal calibration
2. A file called temporal_calibration.json which contains information about the results of the executed temporal
calibration 
*/

int main(int argc, char* argv[]) {
	using namespace curan::ui;
	
	std::unique_ptr<Context> context = std::make_unique<Context>();;
	DisplayParams param{ std::move(context)};
	std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
	IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

	std::shared_ptr<ProcessingMessage> processing;
	auto page = create_main_page(processing,resources);

	page.update_page(viewer.get());

	ConfigDraw config{&page};
	processing->config = &config;
	
	while (!glfwWindowShouldClose(viewer->window)) {
		auto start = std::chrono::high_resolution_clock::now();
		SkSurface* pointer_to_surface = viewer->getBackbufferSurface();
		SkCanvas* canvas = pointer_to_surface->getCanvas();
		if (viewer->was_updated()) {
	    	page.update_page(viewer.get());
			viewer->update_processed();
		}
		page.draw(canvas);
		auto signals = viewer->process_pending_signals();
		if (!signals.empty())
			page.propagate_signal(signals.back(), &config);
		glfwPollEvents();

		bool val = viewer->swapBuffers();
		if (!val)
			std::cout << "failed to swap buffers\n";
		auto end = std::chrono::high_resolution_clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
	}

	processing->attempt_stop();
	std::cout << "trying to stop communication\n" << std::endl;

	if(processing->calibration_value){ // if the calibration was performed successefully then we can replace the temporal calibration in the xml file
		const std::string device_id{"ROBOT"};
    	switch(curan::utilities::modify_xml_field_in_place(CURAN_COPIED_RESOURCE_PATH"/plus_config/plus_spacial_calib_robot_xml/robot_image.xml","LocalTimeOffsetSec","Id",device_id,(-1e-3)*(*processing->calibration_value),{"PlusConfiguration","DataCollection"})){
        case curan::utilities::ErrorCode::SUCCESS:
			return 0;
        break;
        case curan::utilities::ErrorCode::CHILD_NOT_FOUND:
            std::cout << "child not found in XML file" << std::endl;
        break;
        case curan::utilities::ErrorCode::INVALID_ARGUMENT:
            std::cout << "invalid argument present in XML file" << std::endl; 
        break;
        case curan::utilities::ErrorCode::PATH_NOT_FOUND:
            std::cout << "path to XML file not found" << std::endl;
        break;
        case curan::utilities::ErrorCode::ATRIBUTE_NOT_FOUND:
            std::cout << "device id: " << device_id << " - device not found" << std::endl;
        break;
        default:
            std::cout << "unknown error" << std::endl;
    	}
	}

	return 1;
}