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
#include "Mathematics/IntrPlane3OrientedBox3.h"
#include "itkGDCMImageIO.h"
#include "itkGDCMSeriesFileNames.h"
#include "itkImageSeriesReader.h"


constexpr unsigned int Dimension_in = 3;
constexpr unsigned int Dimension_out = 3;
using InputPixelType = unsigned char;
using OutputPixelType = unsigned char;
using DicomPixelType = unsigned short;

using InterPixelType = float;

using InputImageType = itk::Image<InterPixelType, Dimension_in>;
using OutputImageType = itk::Image<OutputPixelType, Dimension_out>;
using DICOMImageType = itk::Image<DicomPixelType, Dimension_in>;

using InterImageType = itk::Image<InterPixelType, Dimension_out>;

// using TransformType = itk::AffineTransform<double, Dimension_in>;
using TransformType = itk::Euler3DTransform<double>;

using ReaderType = itk::ImageFileReader<InputImageType>;


void load_dicom(InputImageType::Pointer& image) {
    
    using ReaderTypeDicom = itk::ImageSeriesReader<DICOMImageType>;
    auto reader = ReaderTypeDicom::New();

    using ImageIOType = itk::GDCMImageIO;
    auto dicomIO = ImageIOType::New();

    reader->SetImageIO(dicomIO);

    using NamesGeneratorType = itk::GDCMSeriesFileNames;
    auto nameGenerator = NamesGeneratorType::New();

    nameGenerator->SetUseSeriesDetails(true);
    nameGenerator->AddSeriesRestriction("0008|0021");

    std::string dirName_input{CURAN_COPIED_RESOURCE_PATH "/dicom_sample/ST983524"};
    nameGenerator->SetDirectory(dirName_input);

    using SeriesIdContainer = std::vector<std::string>;

    const SeriesIdContainer &seriesUID = nameGenerator->GetSeriesUIDs();

    auto seriesItr = seriesUID.begin();
    auto seriesEnd = seriesUID.end();
    while (seriesItr != seriesEnd)
    {
        std::cout << seriesItr->c_str() << std::endl;
        ++seriesItr;
    }

    std::string seriesIdentifier;
    seriesIdentifier = seriesUID.begin()->c_str();

    using FileNamesContainer = std::vector<std::string>;
    FileNamesContainer fileNames;

    fileNames = nameGenerator->GetFileNames(seriesIdentifier);

    reader->SetFileNames(fileNames);

    using RescaleType = itk::RescaleIntensityImageFilter<DICOMImageType, DICOMImageType>;
    auto rescale = RescaleType::New();
    rescale->SetInput(reader->GetOutput());
    rescale->SetOutputMinimum(0);
    rescale->SetOutputMaximum(itk::NumericTraits<InputPixelType>::max());

    std::cout << "aqui" << std::endl;

    using FilterType = itk::CastImageFilter<DICOMImageType, InputImageType>;
    auto filter = FilterType::New();
    filter->SetInput(rescale->GetOutput());

    try
    {
        filter->Update();
    }
    catch (const itk::ExceptionObject &ex)
    {
        std::cout << ex << std::endl;
        //return std::nullopt;
    }

    image = filter->GetOutput();
};


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



void resampler(curan::renderable::DynamicTexture *texture, OutputImageType::Pointer output, InputImageType::Pointer volume, itk::Point<double,3> needle_tip, itk::Matrix<double,3> image_orientation)
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
    auto new_origin = old_origin;
    new_origin[0] = old_origin[0] + needle_tip[0];
    new_origin[1] = old_origin[1] + needle_tip[1];
    new_origin[2] = old_origin[2] + needle_tip[2];
    filter->SetOutputOrigin(new_origin);

        

    using TransformType = itk::IdentityTransform<double, 3>;
    auto transform = TransformType::New();

    filter->SetOutputDirection(image_orientation);

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

    output = filter->GetOutput();

    auto new_origin2 = output->GetOrigin();
    auto image_homogenenous_transformation = vsg::translate(0.0, 0.0, 0.0);
    image_homogenenous_transformation(3, 0) = (new_origin[0]) / 1000.0;
    image_homogenenous_transformation(3, 1) = (new_origin[1]) / 1000.0;
    image_homogenenous_transformation(3, 2) = (new_origin[2]) / 1000.0;

    auto image_final_direction = output->GetDirection();
    for (size_t row = 0; row < 3; ++row)
        for (size_t col = 0; col < 3; ++col)
            image_homogenenous_transformation(col, row) = image_final_direction(row, col);

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


    /* InputImageType::Pointer pointer_to_block_of_memory = InputImageType::New();
    create_volume(pointer_to_block_of_memory); */

    /* InputImageType::Pointer pointer_to_block_of_memory = reader->GetOutput(); */

    InputImageType::Pointer pointer_to_block_of_memory;
    load_dicom(pointer_to_block_of_memory);


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

    OutputImageType::Pointer output_slice;

    std::atomic<bool> continue_running = true;

    std::thread run_slice_extractor{[&]()
                                    {
                                        while (continue_running)
                                        {
                                            itk::Point<double,3> needle_tip;
                                            itk::Point<double,3> image_orientation_angles;
                                            //for (size_t zzz = 0; zzz < 200; ++zzz) {
                                            /* for (size_t aaa = 0; aaa < 200; ++aaa) {
                                            for (size_t bbb = 0; bbb < 200; ++bbb) {
                                            for (size_t ccc = 0; ccc < 200; ++ccc) { */
                                             
                                                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                                                needle_tip[0] = 0.0*1.0;
                                                needle_tip[1] = 0.0;//ccc-100.0;
                                                needle_tip[2] = 80.0;//aaa * 1.0 + 100.0;

                                                image_orientation_angles[0] = 0.0;//aaa/100.0 - 1.0; 
                                                image_orientation_angles[1] = 0.0001415926535/2.0;//aaa/100.0 - 1.0;//
                                                image_orientation_angles[2] = 0.0;

                                                TransformType::Pointer transform = TransformType::New();
                                                //transform->SetCenter(rotation_center);
                                                transform->SetRotation(image_orientation_angles[0], image_orientation_angles[1], image_orientation_angles[2]);

                                                resampler(casted_image, output_slice, pointer_to_block_of_memory, needle_tip, transform->GetMatrix());

                                                /* if (continue_running == false) {
                                                    break;
                                                }

                                            } if (continue_running == false) {
                                                    break;
                                                }}
                                            if (continue_running == false) {
                                                    break;
                                                }}        */                                    
                                        }
                                    }};

    window.run();
    continue_running = false;
    run_slice_extractor.join();

    return EXIT_SUCCESS;

    std::cout << "Aqui 1 \n" << std::endl;
//////////////// pixel correspondance verification /////////////////////////////////////////////////////////////////////////////

    using Iterator2 = itk::ImageRegionIterator<OutputImageType>;


    std::cout << "Aqui 2 \n" << std::endl;

    Iterator2 itr(output_slice, output_slice->GetRequestedRegion());

    std::cout << "Aqui 3 \n" << std::endl;


    InputImageType::IndexType volume_idx;
    itk::Point<double,3> current_pixel;

    std::cout << "Aqui 4 \n" << std::endl;

    double volume_pixel_value;
    double slice_pixel_value;

    std::cout << "Aqui 5 \n" << std::endl;

    double max_pixel_value_difference = 0.0;


    for (itr.GoToBegin(); !itr.IsAtEnd(); ++itr)
    {
        std::cout << "Aqui 6 \n" << std::endl;
        InputImageType::IndexType current_idx = itr.GetIndex();
        output_slice->TransformIndexToPhysicalPoint(current_idx, current_pixel);

        slice_pixel_value = output_slice->GetPixel(current_idx);

        pointer_to_block_of_memory->TransformPhysicalPointToIndex(current_pixel, volume_idx);

        volume_pixel_value = pointer_to_block_of_memory->GetPixel(volume_idx);
        
        if (std::abs(slice_pixel_value-volume_pixel_value) > max_pixel_value_difference) {
            max_pixel_value_difference = std::abs(slice_pixel_value-volume_pixel_value);
        }
    }
    
    
    std::cout << "Max pixel difference: " << max_pixel_value_difference << std::endl;

    return EXIT_SUCCESS;
}