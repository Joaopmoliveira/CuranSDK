#define STB_IMAGE_IMPLEMENTATION
#include "UserInterface.h"
#include "LoadVolume.h"
#include <nlohmann/json.hpp>
#include <time.h>
#include "itkImageDuplicator.h"

void load_all_files_in_directory(Application &app_data, curan::ui::ConfigDraw *drawing_data, std::atomic<bool> &stop_value, std::mutex &mut)
{
    std::vector<std::string> uids_to_load = get_representative_uids(app_data.path);
    for (const auto &uid : uids_to_load)
    {
        std::optional<ImageType::Pointer> image = get_representative_series_image(app_data.path, uid);
        if (stop_value.load())
            return;

        if (image)
        {
            std::lock_guard<std::mutex> g{mut};
            app_data.loaded.push_back({*image, uid});
        }
        if (stop_value.load())
            return;
    }
};

template <typename TImage,typename InclusionPolicy>
typename TImage::Pointer DeepCopyWithInclusionPolicy(InclusionPolicy&& policy,typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIteratorWithIndex<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd()){
        if(policy(outputIt.GetIndex()[0],outputIt.GetIndex()[1],outputIt.GetIndex()[2]))
            outputIterator.Set(inputIterator.Get());
        else
            outputIterator.Set(0);
        ++inputIterator;
        ++outputIterator;
    }

    return  output;
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

        Application data_application{resources, CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524", mut};

        curan::ui::Page page{std::move(data_application.main_page()), SK_ColorBLACK};

        ConfigDraw config{&page};

        data_application.ptr_config = &config;

        std::atomic<bool> function_value = false;

        curan::utilities::Job job{"load files in backend", [&]()
                                  {
                                      load_all_files_in_directory(data_application, &config, function_value, mut);
                                  }};
        data_application.pool->submit(job);

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

        auto return_current_time_and_date = []()
        {
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
            return ss.str();
        };

        Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ", ", "\n", " ", " ");

        if (data_application.final_first_point && data_application.final_second_point && data_application.final_third_point)
        {
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
            boundaries[0] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetRequestedRegion().GetSize()[0];
            boundaries[1] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetRequestedRegion().GetSize()[1]; //
            boundaries[2] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetRequestedRegion().GetSize()[2];
            boundaries[3] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetRequestedRegion().GetSize()[0];
            boundaries[4] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetRequestedRegion().GetSize()[1]; //
            boundaries[5] *= data_application.map[ORIGINAL_VOLUME].get_volume()->GetRequestedRegion().GetSize()[2];
            internals.push_back(boundaries);
        }

        auto evaluate_if_pixel_inside_mask = [&](double in_x, double in_y, double in_z)
        {
            for (const auto &boundary : internals)
                if ((in_x > boundary[0] && in_x < boundary[3]) && (in_y > boundary[1] && in_y < boundary[4]) && (in_z > boundary[2] && in_z < boundary[5]))
                    return true;
            return false;
        };

        ImageType::Pointer masked_output_image = DeepCopyWithInclusionPolicy<ImageType>(evaluate_if_pixel_inside_mask,data_application.map[ORIGINAL_VOLUME].get_volume());

        using WriterType = itk::ImageFileWriter<ImageType>;

        {
            auto writer = WriterType::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/original_volume.mha");
            writer->SetInput(data_application.map[ORIGINAL_VOLUME].get_volume());
            writer->Update();
        }

        {
            auto writer = WriterType::New();
            writer->SetFileName(CURAN_COPIED_RESOURCE_PATH "/masked_volume.mha");
            writer->SetInput(masked_output_image);
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