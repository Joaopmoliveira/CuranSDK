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


void create_volume(InputImageType::Pointer image) {
//Sphere
    itk::Matrix<double> image_orientation;
    itk::Point<double> image_origin;

 	image_orientation[0][0] = 1.0;
 	image_orientation[1][0] = 0.0;
 	image_orientation[2][0] = 0.0;

	image_orientation[0][1] = 0.0;
 	image_orientation[1][1] = 1.0;
 	image_orientation[2][1] = 0.0;

 	image_orientation[0][2] = 0.0;
 	image_orientation[1][2] = 0.0;
 	image_orientation[2][2] = 1.0;

 	image_origin[0] = 0.0;
 	image_origin[1] = 0.0;
 	image_origin[2] = 0.0;

    InputImageType::SpacingType spacing;
    spacing[0] = 1.0;
 	spacing[1] = 1.0;
 	spacing[2] = 1.0;


    InputImageType::SizeType size = { { 100, 100, 100 } };
    InputImageType::IndexType index = { { 0, 0, 0 } };
    InputImageType::IndexType max_index = { { 99, 99, 99 } };
    InputImageType::RegionType region;
    region.SetSize(size);
    region.SetIndex(index);
    image->SetRegions(region);
    image->SetDirection(image_orientation);
    image->SetSpacing(spacing);
    image->SetOrigin(image_origin);
    image->Allocate(true); // initialize buffer to zero

    using Iterator = itk::ImageRegionIterator<InputImageType>;

    InputImageType::IndexType idx = {{0,0,0}};

    image->TransformIndexToPhysicalPoint(idx,image_origin);

    Iterator it(image, region);

    itk::Point<double,3> sphere_center;
    sphere_center[0] = 0.0;
    sphere_center[1] = 0.0;
    sphere_center[2] = 0.0;
    
    itk::Point<double,3> current_pixel;

    itk::Point<double,3> max_pixel_position;
    image->TransformIndexToPhysicalPoint(max_index, max_pixel_position);

    auto max_radius = max_pixel_position[0]*max_pixel_position[0] + max_pixel_position[1]*max_pixel_position[1] + max_pixel_position[2]*max_pixel_position[2];

    std::cout << "maximum radius: " << max_radius << std::endl;

    for (it.GoToBegin(); !it.IsAtEnd(); ++it)
    {
        InputImageType::IndexType current_idx = it.GetIndex();
        image->TransformIndexToPhysicalPoint(current_idx,current_pixel);
        double squared_sphere_radious = ((current_pixel[0]-sphere_center[0])*(current_pixel[0]-sphere_center[0])+(current_pixel[1]-sphere_center[1])*(current_pixel[1]-sphere_center[1])+(current_pixel[2]-sphere_center[2])*(current_pixel[2]-sphere_center[2]));
        it.Set(squared_sphere_radious/max_radius*255.0);
    }

    };



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
    filter->SetDefaultPixelValue(0);

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

    /* std::cout << "Transformation matrix: \n" << matrix << std::endl;

    std::cout << "Transformation offset: \n" << offset << std::endl; */

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

    InputImageType::Pointer pointer_to_block_of_memory = InputImageType::New();
    create_volume(pointer_to_block_of_memory);

    //InputImageType::Pointer pointer_to_block_of_memory = reader->GetOutput();

    auto input_volume = pointer_to_block_of_memory;
    auto volume_size = input_volume->GetLargestPossibleRegion().GetSize();
    auto volume_spacing = input_volume->GetSpacing();

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
    volumeinfo.width = volume_size.GetSize()[0];
    volumeinfo.height = volume_size.GetSize()[1];
    volumeinfo.depth = volume_size.GetSize()[2];
    volumeinfo.spacing_x = volume_spacing[0];
    volumeinfo.spacing_y = volume_spacing[1];
    volumeinfo.spacing_z = volume_spacing[2];

    auto volume_to_render = curan::renderable::Volume::make(volumeinfo);
    window << volume_to_render;

    auto direction = input_volume->GetDirection();
    auto origin = input_volume->GetOrigin();
    auto fixed_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    fixed_homogenenous_transformation(3, 0) = origin[0] / 1000.0;
    fixed_homogenenous_transformation(3, 1) = origin[1] / 1000.0;
    fixed_homogenenous_transformation(3, 2) = origin[2] / 1000.0;

    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            fixed_homogenenous_transformation(col, row) = direction(row, col);

    volume_to_render->cast<curan::renderable::Volume>()->update_transform(fixed_homogenenous_transformation);

    auto casted_volume = volume_to_render->cast<curan::renderable::Volume>();
    auto updater = [pointer_to_block_of_memory](vsg::floatArray3D &image)
    { updateBaseTexture3D(image, pointer_to_block_of_memory); };
    casted_volume->update_volume(updater);





    auto size_2 = input_volume->GetLargestPossibleRegion().GetSize();
    auto spacing_2 = input_volume->GetSpacing();
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

    curan::renderable::DynamicTexture::Info infotexture;
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
                                            for (size_t aaa = 0; aaa < 100; ++aaa) {
                                            //for (size_t bbb = 0; bbb < 200; ++bbb) {
                                            //for (size_t ccc = 0; ccc < 200; ++ccc) {
                                             
                                                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                                                image_origin[0] = 0.0;
                                                image_origin[1] = 0.0;
                                                image_origin[2] = aaa;

                                                image_orientation_angles[0] = 0.0;
                                                image_orientation_angles[1] = 0.0;
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