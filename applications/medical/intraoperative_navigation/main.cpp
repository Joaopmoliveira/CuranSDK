#include "InteroperativePages.h"
#include <nlohmann/json.hpp>
#include "utils/Reader.h"
#include "utils/FileStructures.h"

int main()
{
    try
    {
        curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json"};
        Eigen::Matrix<double, 3, 1> desired_direction = trajectory_data.target()-trajectory_data.entry();
        desired_direction.normalize();

        curan::utilities::NeedleCalibrationData needle_calibration_data{CURAN_COPIED_RESOURCE_PATH"/needle_calibration.json"}; 

        curan::utilities::RegistrationData registration_data{CURAN_COPIED_RESOURCE_PATH"/registration.json"};

        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 1800, 1000};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

        InputImageType::Pointer input_volume;
        if (input_volume.GetPointer() == nullptr){
            std::cout << "failed to read the dicom image\n";
            return 1;
        }	
        
	    ProcessingMessage processing{nullptr,input_volume};
        processing.target = trajectory_data.target();
        processing.entry_point = trajectory_data.entry();
        processing.desired_rotation = trajectory_data.desired_direction();
        processing.registration = registration_data.moving_to_fixed_transform();
        processing.needle_calibration = needle_calibration_data.needle_calibration();

        Page page{create_main_page(resources,processing),SK_ColorBLACK};
	    page.update_page(viewer.get());

	    ConfigDraw config{&page};

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
	    processing.attempt_stop();
	    std::cout << "trying to stop communication\n" << std::endl;
        return 0;
    }
    catch (std::exception &e)
    {
        std::cout << "Failed: " << e.what() << std::endl;
        return 1;
    }
}