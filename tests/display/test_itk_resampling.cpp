#define STB_IMAGE_IMPLEMENTATION
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkResampleImageFilter.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"

using PixelType = unsigned char;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image<PixelType, Dimension>;

void function(curan::ui::ImageDisplay *image_display, curan::ui::ImageDisplay *resized_image_display)
{
    using ImageReaderType = itk::ImageFileReader<ImageType>;

    std::string dirName{CURAN_COPIED_RESOURCE_PATH "/dicom_sample/mri_brain/233.dcm"};
    using ImageReaderTypeFloat = itk::ImageFileReader<itk::Image<float, Dimension>>;
    auto ImageFloatReader = ImageReaderTypeFloat::New();
    ImageFloatReader->SetFileName(dirName);

    auto ImageReader = ImageReaderType::New();


    ImageReader->SetFileName(dirName);
    try
    {
        ImageFloatReader->Update();
        ImageReader->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        return;
    }

    ImageType::Pointer pointer_to_block_of_memory = ImageReader->GetOutput();
    ImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();
    auto buff = curan::utilities::CaptureBuffer::make_shared(pointer_to_block_of_memory->GetBufferPointer(), pointer_to_block_of_memory->GetPixelContainer()->Size() * sizeof(PixelType), pointer_to_block_of_memory);
    curan::ui::ImageWrapper wrapper{buff, size_itk[0], size_itk[1]};
    image_display->update_image(wrapper);

    std::cout << "(input) origin : " << pointer_to_block_of_memory->GetOrigin() << std::endl;
    std::cout << "(input) size : " << pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize() << std::endl;
    std::cout << "(input) spacing : " << pointer_to_block_of_memory->GetSpacing() << std::endl;
    std::cout << "(input) direction : " << pointer_to_block_of_memory->GetDirection() << std::endl;

    using FilterType = itk::ResampleImageFilter<ImageType, ImageType>;
    auto filter = FilterType::New();

    using InterpolatorType = itk::LinearInterpolateImageFunction<ImageType, double>;
    auto interpolator = InterpolatorType::New();
    filter->SetInterpolator(interpolator);
    filter->SetDefaultPixelValue(120);

    auto input = pointer_to_block_of_memory;
    auto size = input->GetLargestPossibleRegion().GetSize();
    auto spacing = input->GetSpacing();
    double minimum_spacing = std::min(std::min(spacing[0], spacing[1]), spacing[2]);
    double maximum_size = std::max(std::max(size[0], size[1]), size[2]);
    auto new_spacing = spacing;
    new_spacing[0] = minimum_spacing;
    new_spacing[1] = minimum_spacing;
    new_spacing[2] = minimum_spacing;

    auto out_size = size;
    out_size[0] = maximum_size;
    out_size[1] = maximum_size;
    out_size[2] = maximum_size;

    filter->SetInput(input);
    filter->SetSize(out_size);
    itk::Point<double,3>::VectorType new_origin{{-10.0,0.1,0.0}};
    auto old_origin = pointer_to_block_of_memory->GetOrigin() ;
    old_origin += new_origin;
    filter->SetOutputOrigin(old_origin);
    filter->SetOutputSpacing(new_spacing);
    itk::Matrix<double,3,3> new_direction;
    double angle = 0.1;
    new_direction(0,0) = std::cos(angle);
    new_direction(1,0) = std::sin(angle);
    new_direction(2,0) = 0.0;

    new_direction(0,1) = -std::sin(angle);
    new_direction(1,1) =  std::cos(angle);
    new_direction(2,1) = 0.0;

    new_direction(0,2) = 0.0;
    new_direction(1,2) = 0.0;
    new_direction(2,2) = 1.0;
    filter->SetOutputDirection(pointer_to_block_of_memory->GetDirection()*new_direction);

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject& e)
    {
        std::string result = "Failure to update the filter"+std::string{e.what()};
        std::cout << result;
        return;
    }

    auto output = filter->GetOutput();

    std::cout << "(output) origin : " << output->GetOrigin() << std::endl;
    std::cout << "(output) size : " << output->GetLargestPossibleRegion().GetSize() << std::endl;
    std::cout << "(output) spacing : " << output->GetSpacing() << std::endl;
    std::cout << "(output) direction : " << output->GetDirection() << std::endl;

    itk::Point<double,3> size_x{{(double)size[0]*input->GetSpacing()[0]*0.5,0.0,0.0}};
    itk::Point<double,3> size_y{{0.0,(double)size[1]*input->GetSpacing()[1]*0.5,0.0}};
    itk::Point<double,3> size_z{{0.0,0.0,(double)size[2]*input->GetSpacing()[2]*0.5}};

    std::cout << "size_x x : \n" << (double)size[0] << " " << input->GetSpacing()[0] << " " << 0.5e-3 << " "  << (double)size[0]*input->GetSpacing()[0]*0.5e-3 << std::endl;
    std::cout << "size_y y : \n" << (double)size[1] << " " << input->GetSpacing()[1] << " " << 0.5e-3 << " "  << (double)size[1]*input->GetSpacing()[0]*0.5e-3 << std::endl;
    std::cout << "size_z z : \n" << (double)size[2] << " " << input->GetSpacing()[2] << " " << 0.5e-3 << " "  << (double)size[2]*input->GetSpacing()[0]*0.5e-3 << std::endl;

    auto vector_form_origin = itk::Point<double,3>::VectorType{{input->GetOrigin()[0],input->GetOrigin()[1],input->GetOrigin()[2]}};

    std::cout << "vector_form_origin : " << vector_form_origin << std::endl;

    auto transformed_size_x_relative_origin = pointer_to_block_of_memory->GetDirection()*size_x;
    auto transformed_size_x = transformed_size_x_relative_origin;
    transformed_size_x += vector_form_origin;
    auto transformed_size_y_relative_origin = pointer_to_block_of_memory->GetDirection()*size_y;
    auto transformed_size_y = transformed_size_y_relative_origin;
    transformed_size_y += vector_form_origin;
    auto transformed_size_z_relative_origin = pointer_to_block_of_memory->GetDirection()*size_z;
    auto transformed_size_z = transformed_size_z_relative_origin;
    transformed_size_z += vector_form_origin;

    std::cout << "transformed x : \n" << transformed_size_x << std::endl;
    std::cout << "transformed y : \n" << transformed_size_y << std::endl;
    std::cout << "transformed z : \n" << transformed_size_z << std::endl;

    std::cout << "transformed_size_x_relative_origin x : \n" << transformed_size_x_relative_origin << std::endl;
    std::cout << "transformed_size_y_relative_origin y : \n" << transformed_size_y_relative_origin << std::endl;
    std::cout << "transformed_size_z_relative_origin z : \n" << transformed_size_z_relative_origin << std::endl;

    std::array<double,8> minimum_values;
    for(size_t i = 0; i<3 ; ++i){
        
    }

    ImageType::SizeType out_size_itk = output->GetLargestPossibleRegion().GetSize();
    auto out_buff = curan::utilities::CaptureBuffer::make_shared(output->GetBufferPointer(), output->GetPixelContainer()->Size() * sizeof(PixelType), output);
    curan::ui::ImageWrapper out_wrapper{out_buff, out_size_itk[0], out_size_itk[1]};
    resized_image_display->update_image(out_wrapper);
}

int main()
{
    try
    {
        using namespace curan::ui;
        std::unique_ptr<Context> context = std::make_unique<Context>();
        ;
        DisplayParams param{std::move(context), 1800, 1200};
        std::unique_ptr<Window> viewer = std::make_unique<Window>(std::move(param));

        std::unique_ptr<ImageDisplay> image_display = ImageDisplay::make();
        std::unique_ptr<ImageDisplay> resampled_image_display = ImageDisplay::make();
        ImageDisplay *pointer_to = image_display.get();
        ImageDisplay *pointer_to_resampled = resampled_image_display.get();
        auto container = Container::make(Container::ContainerType::LINEAR_CONTAINER, Container::Arrangement::HORIZONTAL);
        *container << std::move(image_display) << std::move(resampled_image_display);
        curan::ui::Page page{std::move(container), SK_ColorBLACK};

        auto call = [=]()
        {
            function(pointer_to,pointer_to_resampled);
        };

        std::thread image_generator(call);

        page.update_page(viewer.get());

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
        image_generator.join();
        return 0;
    }
    catch (std::exception &e)
    {
        std::cout << "Failed: " << e.what() << std::endl;
        return 1;
    }
}