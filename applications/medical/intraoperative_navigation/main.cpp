#include "InteroperativePages.h"
#include <nlohmann/json.hpp>
#include "utils/Reader.h"
#include "utils/FileStructures.h"

int main(){
    try{
        curan::utilities::TrajectorySpecificationData trajectory_data{CURAN_COPIED_RESOURCE_PATH"/trajectory_specification.json"};
        curan::utilities::NeedleCalibrationData needle_calibration_data{CURAN_COPIED_RESOURCE_PATH"/needle_calibration.json"}; 
        curan::utilities::RegistrationData registration_data{CURAN_COPIED_RESOURCE_PATH"/registration_specification.json"};

        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 1800, 1000};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));
        IconResources resources{CURAN_COPIED_RESOURCE_PATH"/images"};

        auto image_reader_moving = ReaderType::New();
        image_reader_moving->SetFileName(trajectory_data.path_to_original_image());
        try{
            image_reader_moving->Update();
        } catch(...){
            std::cout << "Could not load moving image from: " << trajectory_data.path_to_original_image() << std::endl;
        }

        InputImageType::Pointer input_volume = image_reader_moving->GetOutput();
        if (input_volume.GetPointer() == nullptr){
            std::cout << "failed to read the dicom image\n";
            return 1;
        }	
        Eigen::Matrix<double,4,4> transformation_to_sensor_base = Eigen::Matrix<double,4,4>::Identity();
        transformation_to_sensor_base(0,3) = input_volume->GetOrigin()[0]*1e-3;
        transformation_to_sensor_base(1,3) = input_volume->GetOrigin()[1]*1e-3;
        transformation_to_sensor_base(2,3) = input_volume->GetOrigin()[2]*1e-3;
    
        auto direction = input_volume->GetDirection();
        for(size_t r = 0; r < 3; ++r)
            for(size_t c = 0; c < 3; ++c)
                transformation_to_sensor_base(r,c) = direction(r,c);
        
        Eigen::Matrix<double,4,4> registered_transformation = registration_data.moving_to_fixed_transform()*transformation_to_sensor_base;
        for(size_t r = 0; r < 3; ++r)
            for(size_t c = 0; c < 3; ++c)
                direction(r,c) = registered_transformation(r,c);  

        
        
        input_volume->SetDirection(direction);   
        auto origin = input_volume->GetOrigin();  
        origin[0] = 1e3*registered_transformation(0,3);
        origin[1] = 1e3*registered_transformation(1,3);
        origin[2] = 1e3*registered_transformation(2,3);
        input_volume->SetOrigin(origin);

        auto convert_to_world = [&](Eigen::Vector3d point_in_moving){
            Eigen::Vector4d in_moving_coordinates = Eigen::Vector4d::Ones();
            in_moving_coordinates[0] = 1e-3*point_in_moving[0];
            in_moving_coordinates[1] = 1e-3*point_in_moving[1];
            in_moving_coordinates[2] = 1e-3*point_in_moving[2];
            Eigen::Vector4d in_world_coordinates = 1e3*(registration_data.moving_to_fixed_transform()*in_moving_coordinates);
            return in_world_coordinates.block<3,1>(0,0);
        };

        ProcessingMessageInfo process_info{};
        process_info.f_desired_rotation = registration_data.moving_to_fixed_transform().block<3,3>(0,0)*trajectory_data.desired_direction();
        process_info.f_in_processed_viwer = nullptr;
        process_info.f_in_volume = input_volume;
        process_info.f_needle_calibration = needle_calibration_data.needle_calibration();
        process_info.f_target = convert_to_world(trajectory_data.target());
        process_info.f_needle_calibration(0,3) *= 1e3;
        process_info.f_needle_calibration(1,3) *= 1e3;
        process_info.f_needle_calibration(2,3) *= 1e3;

	    ProcessingMessage processing{process_info};

        std::cout << "f_desired_rotation : \n" << processing.desired_rotation << std::endl;
        std::cout << "f_needle_calibration : \n" << processing.needle_calibration << std::endl;
        std::cout << "f_target : \n" << processing.target << std::endl;

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