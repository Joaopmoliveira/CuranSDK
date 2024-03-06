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

        while (!glfwWindowShouldClose(viewer->window))
        {
            auto start = std::chrono::high_resolution_clock::now();
            SkSurface *pointer_to_surface = viewer->getBackbufferSurface();
            SkCanvas *canvas = pointer_to_surface->getCanvas();

            canvas->clear(SK_ColorBLACK);
            double translation_scalling_factor = 0.5;
            needle_tip_in_volume_frame[0] = volume_size[0] * volume_spacing[0] * 0.5+translation_scalling_factor*volume_size[0] * volume_spacing[0]*std::sin(mimic_monotonic_timer + 2.34); //-image_size*image_spacing*0.5;
            needle_tip_in_volume_frame[1] = volume_size[1] * volume_spacing[1] * 0.5+translation_scalling_factor*volume_size[1] * volume_spacing[1]*std::sin(mimic_monotonic_timer + 1.34); // -image_size*image_spacing*0.5;//ccc-100.0;
            needle_tip_in_volume_frame[2] = volume_size[2] * volume_spacing[2] * (ccc % 200)/200.0;

            needle_tip = volume_directionn * needle_tip_in_volume_frame + originn;

            double orientation_scalling_factor = 0.3;
            image_orientation_angles[0] = orientation_scalling_factor * std::sin(mimic_monotonic_timer + 2.34);  // zzz/100.0 - 1.25;
            image_orientation_angles[1] = orientation_scalling_factor * std::sin(mimic_monotonic_timer + 1.234); // zzz/100.0 - 1.25;
            image_orientation_angles[2] = orientation_scalling_factor * std::sin(mimic_monotonic_timer + 0.1234);

            obtain_rot_matrix_by_angles(R_ImageToVolume, image_orientation_angles);

            R_ImageToWorld = volume_directionn * R_ImageToVolume;

            auto world_frame_difference_error = desired_direction.col(2)-R_ImageToWorld.col(2);

            double phy = world_frame_difference_error.transpose()*desired_direction.col(0);
            double theta = world_frame_difference_error.transpose()*desired_direction.col(1);
            
            OutputImageType::Pointer slice = resampler(input_volume, needle_tip, R_ImageToWorld, image_size, image_spacing);
            OutputImageType::IndexType coordinate_of_needle_in_image_position;
            OutputImageType::PointType needle_tip_ikt_coordinates{{needle_tip[0], needle_tip[1], needle_tip[2]}};
            slice->TransformPhysicalPointToIndex(needle_tip_ikt_coordinates, coordinate_of_needle_in_image_position);
            OutputImageType::SizeType size_itk = slice->GetLargestPossibleRegion().GetSize();
            auto buff = curan::utilities::CaptureBuffer::make_shared(slice->GetBufferPointer(), slice->GetPixelContainer()->Size() * sizeof(OutputPixelType), slice);
            curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1]};

            auto inner_projection = R_ImageToWorld.col(2).transpose()*desired_direction.col(2);
            auto projected_unto_plane = ((needle_tip-desired_target_point)*R_ImageToWorld.col(2).transpose());
            auto d = (std::abs(inner_projection(0,0))>1e-5) ? projected_unto_plane(0,0)/(inner_projection(0,0)) : 100000.0;

            Eigen::Matrix<double,3,1> real_intersection = desired_target_point+desired_direction.col(2)*d;
            OutputImageType::IndexType index_coordinate_of_perfect_traject_in_needle_plane;
            OutputImageType::PointType coordinate_of_perfect_traject_in_needle_plane{{real_intersection[0], real_intersection[1], real_intersection[2]}};
            slice->TransformPhysicalPointToIndex(coordinate_of_perfect_traject_in_needle_plane,index_coordinate_of_perfect_traject_in_needle_plane);

            auto surface_height = canvas->getSurface()->height();
            auto surface_width = canvas->getSurface()->width();

            SkPaint greenPaint;
            greenPaint.setAntiAlias(true);
            greenPaint.setColor(SK_ColorGREEN);

            SkPaint redPaint;
            redPaint.setAntiAlias(true);
            redPaint.setColor(SK_ColorRED);

            canvas->drawImage(wrapper.image, surface_width / 2.0 - coordinate_of_needle_in_image_position[0], surface_height / 2.0 - coordinate_of_needle_in_image_position[1]);
            canvas->drawCircle(SkPoint::Make(surface_width / 2.0, surface_height / 2.0), 10, greenPaint); 

            Level level;
            double bubble_position = (surface_width-60.0) / 2.0 * ( 1 + theta)+30.0;
            level.draw(canvas,bubble_position,Level::LevelOrientation::horizontal);

            Level levelvert;
            double bubble_position_vert = (surface_height-60.0) / 2.0 * ( 1 + phy)+30.0;
            levelvert.draw(canvas,bubble_position_vert,Level::LevelOrientation::vertical);

            auto error = (desired_target_point-needle_tip).norm();
            greenPaint.setStroke(true);
            const SkScalar intervals[] = {10.0f, 5.0f, 2.0f, 5.0f};
            size_t count = sizeof(intervals) / sizeof(intervals[0]);
            greenPaint.setPathEffect(SkDashPathEffect::Make(intervals, count, 0.0f));
            canvas->drawCircle(SkPoint::Make(surface_width / 2.0, surface_height / 2.0), error, greenPaint); 

            canvas->drawCircle(SkPoint::Make(surface_width / 2.0 - coordinate_of_needle_in_image_position[0]+index_coordinate_of_perfect_traject_in_needle_plane[0], surface_height / 2.0 - coordinate_of_needle_in_image_position[1]+index_coordinate_of_perfect_traject_in_needle_plane[1]), 10, redPaint); 

            auto signals = viewer->process_pending_signals();

            glfwPollEvents();

            bool val = viewer->swapBuffers();
            if (!val)
                std::cout << "failed to swap buffers\n";
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }
        return 0;
    }
    catch (std::exception &e)
    {
        std::cout << "Failed: " << e.what() << std::endl;
        return 1;
    }
}