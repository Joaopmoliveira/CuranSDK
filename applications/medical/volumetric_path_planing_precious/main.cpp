#define STB_IMAGE_IMPLEMENTATION
#include "UserInterface.h"
#include <nlohmann/json.hpp>
#include <time.h>
#include "itkImageDuplicator.h"
#include "utils/FileStructures.h"
#include "utils/DateManipulation.h"

template <typename TImage,typename InclusionPolicy>
std::tuple<typename TImage::Pointer,itk::Image<unsigned char,3>::Pointer> DeepCopyWithInclusionPolicy(InclusionPolicy&& inclusion_policy,typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    auto mask = itk::Image<unsigned char,3>::New();
    mask->SetRegions(input->GetLargestPossibleRegion());
    mask->SetDirection(input->GetDirection());
    mask->SetSpacing(input->GetSpacing());
    mask->SetOrigin(input->GetOrigin());
    mask->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());
    itk::ImageRegionIterator<itk::Image<unsigned char,3>> maskIterator(mask, mask->GetLargestPossibleRegion());

    for(;!inputIterator.IsAtEnd(); ++inputIterator,++outputIterator,++maskIterator ){
        if(inclusion_policy((double)(inputIterator.GetIndex()[0]),(double)(inputIterator.GetIndex()[1]),(double)(inputIterator.GetIndex()[2]))){
            outputIterator.Set(inputIterator.Get());
            maskIterator.Set(255);
        }else{
            outputIterator.Set(0);
            maskIterator.Set(0);
        }
    }
    return {output,mask};
}

int main()
{
    try
    {
        using namespace curan::ui;
        IconResources resources{CURAN_COPIED_RESOURCE_PATH "/images"};
        std::unique_ptr<Context> context = std::make_unique<Context>();

        DisplayParams param{std::move(context), 2200, 1200};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        std::mutex mut;

        Application data_application{resources, CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha", mut};
        curan::ui::Page page{std::move(data_application.main_page()), SK_ColorBLACK};

        ConfigDraw config{&page};

        data_application.ptr_config = &config;

        std::atomic<bool> function_value = false;

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
            for (auto &&signal : viewer->process_pending_signals())
                page.propagate_signal(signal, &config);
            glfwPollEvents();

            bool val = viewer->swapBuffers();
            if (!val)
                std::cout << "failed to swap buffers\n";
            auto end = std::chrono::high_resolution_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds(16) - std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        }

        function_value = true;

        if (data_application.final_first_point && data_application.final_second_point && data_application.final_third_point)
        {
            Eigen::Vector3d desired_dir_z = *data_application.final_first_point-*data_application.final_third_point;
            Eigen::Vector3d desired_dir_x = *data_application.final_first_point-*data_application.final_second_point;
            desired_dir_z.normalize();
            desired_dir_x.normalize();

            Eigen::Vector3d desired_dir_y = desired_dir_x.cross(desired_dir_z);
            desired_dir_y.normalize();
            desired_dir_x = desired_dir_y.cross(desired_dir_z);

            Eigen::Matrix<double,3,3> desired_orientation;
            desired_orientation.block<3,1>(0,0) = desired_dir_x;
            desired_orientation.block<3,1>(0,1) = desired_dir_y;
            desired_orientation.block<3,1>(0,2) = desired_dir_z;

            curan::utilities::TrajectorySpecificationData trajectory_specification{curan::utilities::formated_date<std::chrono::system_clock>(std::chrono::system_clock::now()),*data_application.final_first_point,*data_application.final_third_point,desired_orientation,data_application.path};

            std::ofstream o(CURAN_COPIED_RESOURCE_PATH "/trajectory_specification.json");
            o << trajectory_specification;
            std::cout << trajectory_specification << std::endl;
        }
        else
        {
            std::cout << "the points required to specify the trajectory \nwere not specified, please specify them" << std::endl;
        }

        if (data_application.map[ORIGINAL_VOLUME].get_volume().IsNull())
        {
            std::cout << "the original volume is null, thus no area is contained in it" << std::endl;
            return 1;
        }

        auto geometries = data_application.map[ORIGINAL_VOLUME].geometries();

        // we are going to print the itk image whole and the masked image
        std::vector<std::array<double, 6>> internals;
        internals.reserve(geometries.size());
        for (const auto &geom : geometries)
        {
            std::array<double, 6> boundaries;
            boundaries[0] = std::numeric_limits<double>::max();  // min x - 0
            boundaries[1] = std::numeric_limits<double>::max();  // min y - 1
            boundaries[2] = std::numeric_limits<double>::max();  // min z - 2
            boundaries[3] = -std::numeric_limits<double>::max(); // max x - 3
            boundaries[4] = -std::numeric_limits<double>::max(); // max y - 4
            boundaries[5] = -std::numeric_limits<double>::max(); // max z - 5
            for (const auto &vert : geom.geometry.vertices){
                boundaries[0] = std::min((double)vert[0], boundaries[0]);
                boundaries[1] = std::min((double)vert[1], boundaries[1]); //
                boundaries[2] = std::min((double)vert[2], boundaries[2]);
                boundaries[3] = std::max((double)vert[0], boundaries[3]);
                boundaries[4] = std::max((double)vert[1], boundaries[4]); //
                boundaries[5] = std::max((double)vert[2], boundaries[5]);
            }
            boundaries[0] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetLargestPossibleRegion().GetSize()[0];
            boundaries[1] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetLargestPossibleRegion().GetSize()[1]; //
            boundaries[2] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetLargestPossibleRegion().GetSize()[2];
            boundaries[3] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetLargestPossibleRegion().GetSize()[0];
            boundaries[4] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetLargestPossibleRegion().GetSize()[1]; //
            boundaries[5] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetLargestPossibleRegion().GetSize()[2];
            internals.push_back(boundaries);
        }


        auto evaluate_if_pixel_inside_mask = [&](double in_x, double in_y, double in_z)
        {
            for (const auto &boundary : internals){
                if ((in_x > boundary[0] && in_x < boundary[3]) && 
                    (in_y > boundary[1] && in_y < boundary[4]) &&
                    (in_z > boundary[2] && in_z < boundary[5])){
                        return true;
                    }
                    
            }
            return false;
        };

        auto [masked_output_image,mask_to_use] = DeepCopyWithInclusionPolicy<ImageType>(evaluate_if_pixel_inside_mask,data_application.map[ORIGINAL_VOLUME].get_volume());

        {
            auto writer = itk::ImageFileWriter<ImageType>::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/original_volume.mha");
            writer->SetInput(data_application.map[ORIGINAL_VOLUME].get_volume());
            writer->Update();
        }

        {
            auto writer = itk::ImageFileWriter<ImageType>::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/masked_volume.mha");
            writer->SetInput(masked_output_image);
            writer->Update();
        }

        {
            auto writer = itk::ImageFileWriter<itk::Image<unsigned char,3>>::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/mask.mha");
            writer->SetInput(mask_to_use);
            writer->Update();
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