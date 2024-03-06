#include "load_volume.h"
#include "user_interface_definition.h"
#include <nlohmann/json.hpp>

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 2200, 1200};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        auto volume = get_volume(CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524");
        if (!volume)
            return 1;

        Application data_application{*volume, resources};

        curan::ui::Page page{std::move(data_application.main_page()), SK_ColorBLACK};

        ConfigDraw config{&page};

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();
            if (viewer->was_updated())
            {
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

	    auto return_current_time_and_date = [](){
	        auto now = std::chrono::system_clock::now();
    	    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    	    std::stringstream ss;
    	    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
   		    return ss.str();
	    };

        Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");


        if(data_application.final_first_point && data_application.final_second_point && data_application.final_third_point){
            nlohmann::json trajectory_specification;
	        trajectory_specification["timestamp"] = return_current_time_and_date();
            {
        	    std::stringstream target;
	            target << (*data_application.final_first_point).format(CleanFmt) << std::endl;
	            trajectory_specification["target"] = target.str();
            }
            {
        	    std::stringstream entry;
	            entry << (*data_application.final_third_point).format(CleanFmt) << std::endl;
	            trajectory_specification["entry"] = entry.str();
            }

    	    // write prettified JSON to another file
	        std::ofstream o(CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json");
	        o << trajectory_specification;
	        std::cout << trajectory_specification << std::endl;
        } else {
            std::cout << "the points required to specify the trajectory \nwere not specified, please specify them" << std::endl;
        }

        return 0;
    }
    catch (const std::exception &e)
    {
        std::cout << "Exception thrown:" << e.what() << "\n";
    }
    catch (...)
    {
        std::cout << "Failed to create window for unknown reason\n";
        return 1;
    }
}