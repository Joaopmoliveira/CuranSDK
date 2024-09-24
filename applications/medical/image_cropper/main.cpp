#define STB_IMAGE_IMPLEMENTATION
#include "UserInterface.h"
#include <nlohmann/json.hpp>
#include <time.h>
#include "itkImageDuplicator.h"
#include <string_view>


template <typename TImage,typename InclusionPolicy>
typename TImage::Pointer DeepCopyWithInclusionPolicy(InclusionPolicy&& inclusion_policy,typename TImage::Pointer input)
{
    typename TImage::Pointer output = TImage::New();
    output->SetRegions(input->GetLargestPossibleRegion());
    output->SetDirection(input->GetDirection());
    output->SetSpacing(input->GetSpacing());
    output->SetOrigin(input->GetOrigin());
    output->Allocate();

    itk::ImageRegionConstIteratorWithIndex<TImage> inputIterator(input, input->GetLargestPossibleRegion());
    itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

    while (!inputIterator.IsAtEnd()){
        if(inclusion_policy((double)(inputIterator.GetIndex()[0]),(double)(inputIterator.GetIndex()[1]),(double)(inputIterator.GetIndex()[2]))){
            outputIterator.Set(inputIterator.Get());
        }else
            outputIterator.Set(0);
        ++inputIterator;
        ++outputIterator;
    }

    return  output;
}

//#define "/us_image1_"
//#define "/ct_image1_"

#define FILE_NAME "/us_image1_"

#define FULL_VOLUME CURAN_COPIED_RESOURCE_PATH  FILE_NAME"full_volume.mha"
#define CROPPED_VOLUME CURAN_COPIED_RESOURCE_PATH FILE_NAME"cropepd_volume.mha"

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

        // CURAN_COPIED_RESOURCE_PATH"/precious_phantom/precious_phantom.mha"
        // "C:/Dev/NeuroNavigation/volumes/reconstruction_results5.mha"

        Application data_application{resources,"C:/Dev/NeuroNavigation/volumes/reconstruction_results5.mha", mut};

        curan::ui::Page page{std::move(data_application.main_page()), SK_ColorBLACK};

        ConfigDraw config{&page};

        data_application.ptr_config = &config;

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

        ImageType::Pointer masked_output_image = DeepCopyWithInclusionPolicy<ImageType>(evaluate_if_pixel_inside_mask,data_application.map[ORIGINAL_VOLUME].get_volume());
        using WriterType = itk::ImageFileWriter<ImageType>;

        {
            auto writer = WriterType::New();
            writer->SetFileName(FULL_VOLUME);
            writer->SetInput(data_application.map[ORIGINAL_VOLUME].get_volume());
            writer->Update();
        }

        {
            auto writer = WriterType::New();
            writer->SetFileName(CROPPED_VOLUME);
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