#include "itkImage.h"
#include "itkImageFileReader.h"

#include "itkResampleImageFilter.h"

#include "itkEuler3DTransform.h"
#include "itkNearestNeighborInterpolateImageFunction.h"
#include "userinterface/widgets/ConfigDraw.h"
#include "userinterface/Window.h"
#include "userinterface/widgets/ImageDisplay.h"
#include "userinterface/widgets/Page.h"
#include "userinterface/widgets/IconResources.h"
#include <iostream>
#include "itkImage.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkExtractImageFilter.h"
#include "itkLinearInterpolateImageFunction.h"
#include "itkScaleTransform.h"
#include "itkAffineTransform.h"
#include "itkEuler3DTransform.h"
#include "rendering/Volume.h"
#include "rendering/Window.h"
#include "rendering/Renderable.h"
#include "rendering/DynamicTexture.h"
#include <optional>
#include <chrono>
#include <thread>

constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;

using InterPixelType = float;

using InputImageType = itk::Image<InterPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;

using InterImageType = itk::Image<InterPixelType, Dimension_out>;

// using TransformType = itk::AffineTransform<double, Dimension_in>;
using TransformType = itk::Euler3DTransform<double>;

using ReaderType = itk::ImageFileReader<InputImageType>;

void updateBaseTexture2D(vsg::vec4Array2D &image, OutputImageType::Pointer image_to_render)
{
    try
    {
        int x, y, z;
        auto input = image_to_render;
        auto size = input->GetLargestPossibleRegion().GetSize();
        x = size[0];
        y = size[1];
        z = size[2];

        unsigned char *scaller_buffer = (unsigned char *)image_to_render->GetBufferPointer();

        for (size_t r = 0; r < image.height(); ++r)
        {
            using value_type = typename vsg::vec4Array2D::value_type;
            value_type *ptr = &image.at(0, r);
            for (size_t c = 0; c < image.width(); ++c)
            {
                auto val = *scaller_buffer / 255.0;
                ptr->r = val;
                ptr->g = val;
                ptr->b = val;
                ptr->a = 1.0f;
                ++ptr;
                ++scaller_buffer;
            }
        }
    }
    catch (std::exception &e)
    {
        std::cout << "exception : " << e.what() << std::endl;
        ;
    }
}

void updateBaseTexture3D(vsg::floatArray3D &image, InputImageType::Pointer image_to_render)
{
    using FilterType = itk::CastImageFilter<InputImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(image_to_render);

    using RescaleType = itk::RescaleIntensityImageFilter<InputImageType, InputImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(filter->GetOutput());
    rescale->SetOutputMinimum(0.0);
    rescale->SetOutputMaximum(1.0);

    try
    {
        rescale->Update();
    }
    catch (const itk::ExceptionObject &e)
    {
        std::cerr << "Error: " << e << std::endl;
        throw std::runtime_error("error");
    }

    InputImageType::Pointer out = rescale->GetOutput();

    using IteratorType = itk::ImageRegionIteratorWithIndex<InputImageType>;
    IteratorType outputIt(out, out->GetRequestedRegion());
    for (outputIt.GoToBegin(); !outputIt.IsAtEnd(); ++outputIt)
    {
        InputImageType::IndexType idx = outputIt.GetIndex();
        image.set(idx[0], idx[1], idx[2], outputIt.Get());
    }
}



void resampler(curan::renderable::DynamicTexture *texture, InputImageType::Pointer volume, itk::Point<double,3> image_origin, itk::Point<double,3> image_orientation_angles)
{
    using FilterType = itk::ResampleImageFilter<InputImageType, OutputImageType>;
    auto filter = FilterType::New();

    // using InterpolatorType = itk::LinearInterpolateImageFunction<InputImageType, double>;
    using InterpolatorType = itk::NearestNeighborInterpolateImageFunction<InputImageType, double>;
    auto interpolator = InterpolatorType::New();
    filter->SetInterpolator(interpolator);
    filter->SetDefaultPixelValue(100);

    auto input = volume;
    auto size = input->GetLargestPossibleRegion().GetSize();
    auto spacing = input->GetSpacing();
    double minimum_spacing = std::min(std::min(spacing[0], spacing[1]), spacing[2]);
    double maximum_size = std::max(std::max(size[0], size[1]), size[2]);
    auto new_spacing = spacing;
    new_spacing[0] = minimum_spacing;
    new_spacing[1] = minimum_spacing;
    new_spacing[2] = 0.00001;

    auto out_size = size;
    out_size[0] = maximum_size;
    out_size[1] = maximum_size;
    out_size[2] = 1;

    filter->SetInput(input);

    filter->SetOutputSpacing(new_spacing);
    filter->SetSize(out_size); 

    auto old_origin = volume->GetOrigin();
    filter->SetOutputOrigin(old_origin);

    filter->SetOutputDirection(volume->GetDirection());

    TransformType::Pointer transform = TransformType::New();

    itk::Point<double, 3> rotation_center;
    rotation_center[0] = old_origin[0];// + (size[0]*spacing[0])/2.0;
    rotation_center[1] = old_origin[1];// + (size[1]*spacing[1])/2.0;
    rotation_center[2] = old_origin[2];// + (size[2]*spacing[2])/2.0;

    transform->SetCenter(rotation_center);
    transform->SetRotation(image_orientation_angles[0], image_orientation_angles[1], image_orientation_angles[2]);

    TransformType::OutputVectorType translation;
    translation[0] = image_origin[0]; // X translation in millimeters
    translation[1] = image_origin[1]; // Y translation in millimeters
    translation[2] = image_origin[2]; // Y translation in millimeters
    transform->SetTranslation(translation);

    TransformType::MatrixType matrix = transform->GetMatrix();
    TransformType::OffsetType offset = transform->GetOffset();

    std::cout << "Transformation matrix: \n" << matrix << std::endl;

    std::cout << "Transformation offset: \n" << offset << std::endl;

    filter->SetTransform(transform);

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &e)
    {
        std::string result = "Failure to update the filter" + std::string{e.what()};
        std::cout << result;
        // return;
    }

    OutputImageType::Pointer output = filter->GetOutput();


    auto image_direction = output->GetDirection();
    // auto image_origin = output->GetOrigin();
    auto image_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    image_homogenenous_transformation(3, 0) = (old_origin[0] + translation[0]) / 1000.0;
    image_homogenenous_transformation(3, 1) = (old_origin[1] + translation[1]) / 1000.0;
    image_homogenenous_transformation(3, 2) = (old_origin[2] + translation[2]) / 1000.0;

    auto final_direction = image_direction * matrix;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            image_homogenenous_transformation(col, row) = final_direction(row, col);
            //image_homogenenous_transformation(col, row) = final_direction(col, row);

    texture->update_transform(image_homogenenous_transformation);

    auto updater_image = [output](vsg::vec4Array2D &image)
    { updateBaseTexture2D(image, output); };
    texture->update_texture(updater_image);
}

int main(int argc, char *argv[])
{
    auto reader = ReaderType::New();
    std::string dirName_input{CURAN_COPIED_RESOURCE_PATH "/precious_phantom/precious_phantom.mha"};
    reader->SetFileName(dirName_input);

    try
    {
        reader->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        // return;
    }

    InputImageType::Pointer pointer_to_block_of_memory = reader->GetOutput();
    InputImageType::SizeType size_itk = pointer_to_block_of_memory->GetLargestPossibleRegion().GetSize();

    auto input = pointer_to_block_of_memory;
    auto size = input->GetLargestPossibleRegion().GetSize();
    auto spacing = input->GetSpacing();

    curan::renderable::Window::Info info;
    info.api_dump = false;
    info.display = "";
    info.full_screen = false;
    info.is_debug = false;
    info.screen_number = 0;
    info.title = "myviewer";
    curan::renderable::Window::WindowSize size_1{1000, 800};
    info.window_size = size_1;
    curan::renderable::Window window{info};

    curan::renderable::Volume::Info volumeinfo;
    volumeinfo.width = size.GetSize()[0];
    volumeinfo.height = size.GetSize()[1];
    volumeinfo.depth = size.GetSize()[2];
    volumeinfo.spacing_x = spacing[0];
    volumeinfo.spacing_y = spacing[1];
    volumeinfo.spacing_z = spacing[2];

    auto volume_to_render = curan::renderable::Volume::make(volumeinfo);
    window << volume_to_render;

    auto direction = pointer_to_block_of_memory->GetDirection();
    auto origin = pointer_to_block_of_memory->GetOrigin();
    auto fixed_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    fixed_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
    fixed_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
    fixed_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            fixed_homogenenous_transformation(col, row) = direction(row, col);

    volume_to_render->cast<curan::renderable::Volume>()->update_transform(fixed_homogenenous_transformation);

    auto casted_volume_fixed = volume_to_render->cast<curan::renderable::Volume>();
    auto updater = [pointer_to_block_of_memory](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer_to_block_of_memory); };
    casted_volume_fixed->update_volume(updater);

    curan::renderable::DynamicTexture::Info infotexture;

    auto size_2 = input->GetLargestPossibleRegion().GetSize();
    auto spacing_2 = input->GetSpacing();
    double minimum_spacing = std::min(std::min(spacing_2[0], spacing_2[1]), spacing_2[2]);
    double maximum_size = std::max(std::max(size_2[0], size_2[1]), size_2[2]);
    auto new_spacing = spacing_2;
    new_spacing[0] = minimum_spacing;
    new_spacing[1] = minimum_spacing;
    new_spacing[2] = 0.00001;

    auto out_size = size_2;
    out_size[0] = maximum_size;
    out_size[1] = maximum_size;
    out_size[2] = 1;

    infotexture.height = out_size[1];
    infotexture.width = out_size[0];
    infotexture.spacing = {new_spacing[0] / 1000, new_spacing[1] / 1000, new_spacing[2] / 1000};
    infotexture.origin = {0.0, 0.0, 0.0};
    infotexture.builder = vsg::Builder::create();

    auto dynamic_texture = curan::renderable::DynamicTexture::make(infotexture);
    window << dynamic_texture;
    auto casted_image = dynamic_texture->cast<curan::renderable::DynamicTexture>();

    std::atomic<bool> continue_running = true;

    std::thread run_slice_extractor{[&]()
                                    {
                                        while (continue_running)
                                        {
                                            itk::Point<double,3> image_origin;
                                            itk::Point<double,3> image_orientation_angles;
                                            //for (size_t zzz = 0; zzz < 200; ++zzz) {
                                            for (size_t aaa = 0; aaa < 200; ++aaa) {
                                            //for (size_t bbb = 0; bbb < 200; ++bbb) {
                                            //for (size_t ccc = 0; ccc < 200; ++ccc) {
                                             
                                                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                                                image_origin[0] = 0.0;
                                                image_origin[1] = 0.0;
                                                image_origin[2] = 100.0;

                                                image_orientation_angles[1] = aaa/100.0 - 1.0;
                                                image_orientation_angles[0] = aaa/100.0 - 1.0;
                                                image_orientation_angles[2] = 0.0;



                                                resampler(casted_image, pointer_to_block_of_memory, image_origin, image_orientation_angles);
                                            }//}}}
                                        }
                                    }};

    window.run();
    continue_running = false;
    run_slice_extractor.join();

    return EXIT_SUCCESS;
}