#include "InteroperativePages.h"
#include <nlohmann/json.hpp>
#include "utils/Reader.h"

int main()
{
    try
    {
        Eigen::Matrix<double, 3, 1> desired_target_point;
        Eigen::Matrix<double, 3, 3> desired_orientation;
        Eigen::Matrix<double, 3, 1> desired_direction;
        {
            nlohmann::json trajectory_data;
	        std::ifstream in(CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json");
            if(!in.is_open()){
                std::cout << "failure to find the trajectory specification file";
                return 1;
            }
    	    in >> trajectory_data;
            std::stringstream ss;
        	std::string target = trajectory_data["target"];
            ss << target;
            auto eigen_target = curan::utilities::convert_matrix(ss);
            ss = std::stringstream{};
	        std::string entry = trajectory_data["entry"];
            ss << entry;
            auto eigen_entry = curan::utilities::convert_matrix(ss);
            assert(eigen_target.cols()==1 && eigen_target.rows()==3);
            assert(eigen_entry.cols()==1 && eigen_entry.rows()==3);
            Eigen::Matrix<double, 3, 1> vectorized_eigen_entry;
            for(size_t i = 0; i < 3; ++i){
                desired_target_point[i] = eigen_target(i,0);
                vectorized_eigen_entry[i] = eigen_entry(i,0);
            }
            desired_direction = desired_target_point-vectorized_eigen_entry;
            desired_direction.normalize();
        }

        {
            nlohmann::json trajectory_data;
	        std::ifstream in(CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json");
            if(!in.is_open()){
                std::cout << "failure to find the trajectory specification file";
                return 1;
            }
    	    in >> trajectory_data;
            std::stringstream ss;
        	std::string target = trajectory_data["target"];
            ss << target;
            auto eigen_target = curan::utilities::convert_matrix(ss);
            ss = std::stringstream{};
	        std::string entry = trajectory_data["entry"];
            ss << entry;
            auto eigen_entry = curan::utilities::convert_matrix(ss);
            assert(eigen_target.cols()==1 && eigen_target.rows()==3);
            assert(eigen_entry.cols()==1 && eigen_entry.rows()==3);
            Eigen::Matrix<double, 3, 1> vectorized_eigen_entry;
            for(size_t i = 0; i < 3; ++i){
                desired_target_point[i] = eigen_target(i,0);
                vectorized_eigen_entry[i] = eigen_entry(i,0);
            }
            desired_direction = desired_target_point-vectorized_eigen_entry;
            desired_direction.normalize();
        }

        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 1800, 1000};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

        InputImageType::Pointer input_volume = load_dicom();
        if (input_volume.GetPointer() == nullptr){
            std::cout << "failed to read the dicom image\n";
            return 1;
        }
	    ProcessingMessage processing{nullptr,input_volume};
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