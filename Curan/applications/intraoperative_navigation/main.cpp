#include "InteroperativePages.h"
#include <nlohmann/json.hpp>

int main()
{
    try
    {
        nlohmann::json trajectory_data;
	    std::ifstream in(CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json");
        if(!in.is_open()){
            std::cout << "failure to find the trajectory specification file";
            return 1;
        }
	    in >> trajectory_data;

        std::string timestamp = trajectory_data["timestamp"];
    	std::string target = trajectory_data["target"];
	    std::string entry = trajectory_data["entry"];
	    
        Eigen::Matrix<double, 3, 1> desired_target_point;
        Eigen::Matrix<double, 3, 3> desired_direction;

        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();
        ;
        DisplayParams param{std::move(context), 1800, 1000};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        InputImageType::Pointer input_volume = load_dicom();
        if (input_volume.GetPointer() == nullptr)
            return 1;
	    std::shared_ptr<ProcessingMessage> processing;
	    auto page = create_main_page(data,processing,resources);

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
	    processing->attempt_stop();
	    std::cout << "trying to stop communication\n" << std::endl;
        return 0;
    }
    catch (std::exception &e)
    {
        std::cout << "Failed: " << e.what() << std::endl;
        return 1;
    }
}